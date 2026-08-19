/*
 * LG webOS Starfish hardware-decode backend for mpv.
 *
 * This does not decode anything itself -- it forwards each access unit to
 * StarfishMediaAPIs::Feed() (via misc/starfish_bridge.h) and lets Starfish's
 * own black-box GStreamer pipeline decode and present directly to the
 * hardware video plane, entirely on its own internal clock. Because of that
 * last point, this is architecturally closer to ad_spdif.c (forward
 * compressed data immediately, don't try to own timing) than to vd_lavc.c or
 * a "decode now, present later at a chosen time" hwdec like
 * vo_mediacodec_embed -- Starfish gives no "decode now, present later" split
 * to hook into. Read ad_spdif.c first if this file is confusing; it's the
 * closer relative.
 *
 * The frames this emits are placeholders (see dummy_img below) -- carrying
 * real pts so mpv's own EOF/stats/AV-sync bookkeeping has something to
 * observe, but no real pixel data. The intended VO is --vo=null; nothing
 * downstream of this filter is ever meant to actually draw anything.
 *
 * H264/HEVC need their MP4-style (avcC) extradata converted to Annex B
 * before Starfish will accept them (matching what every other known-working
 * Starfish integration does); VP8/VP9/AV1 are fed as demuxed, no conversion.
 *
 * init() below deliberately does not require codec->lav_codecpar except
 * inside the H264/HEVC branch, where it's genuinely unavoidable (needed by
 * av_bsf_init). Its own doc comment in demux/stheader.h says it's only "set
 * by demux_{lavf,mkv,raw}", and on-device testing found it unreliably
 * populated here regardless of demuxer -- intermittently null even for
 * files that worked moments before. Codec ID and display dimensions come
 * from mp_codec_params' own fields instead, which the demuxer populates
 * unconditionally as basic track info, not gated behind FFmpeg-parameter
 * conversion the way lav_codecpar is.
 *
 * Ownership note for the two send paths below, since it's the one thing in
 * this file that's easy to get wrong: for the bitstream-filtered path, the
 * incoming demux_packet is kept alive until *after* the av_bsf_receive_packet
 * drain loop finishes, not freed right after av_bsf_send_packet(). This is
 * deliberate: mp_set_av_packet()-style conversions (see common/av_common.c)
 * generally alias demux_packet->buffer rather than taking an independent
 * reference to it (the AVBufferRef is only carried over if demux_packet's
 * own ->avpacket field happens to be set and its data pointer matches), so
 * there's no guarantee the bsf's *input* side is backed by its own reference
 * -- only the *output* packets it produces are trusted to be independently
 * owned (mp4toannexb has to allocate fresh memory to rewrite NAL structure
 * into Annex B, it can't just alias the avcC-formatted input). Freeing the
 * source only after the drain keeps this safe either way.
 */

#include <stdlib.h>

#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>

#include "common/av_common.h"
#include "common/msg.h"
#include "demux/packet.h"
#include "demux/stheader.h"
#include "filters/f_decoder_wrapper.h"
#include "filters/filter_internal.h"
#include "misc/starfish_bridge.h"
#include "mpv_talloc.h"
#include "common/codecs.h"
#include "video/img_format.h"
#include "video/mp_image.h"

// Small FIFO of access units that survived bitstream filtering (or, for
// codecs with no filter, the single unit from the current input packet) and
// are ready to feed. Almost always 0 or 1 entries deep -- mp4toannexb is a
// 1:1 rewrite for any real-world content -- but this doesn't assume that.
struct pending_unit {
    struct demux_packet *raw; // non-NULL: owns the bytes (no-bsf path)
    AVPacket *bsf_pkt;        // non-NULL: owns the bytes (bsf path)
    int64_t pts_ns;
    struct pending_unit *next;
};

struct priv {
    struct mp_codec_params *codec;
    AVBSFContext *bsf; // NULL unless H264/HEVC
    struct pending_unit *head, *tail;
    struct mp_image *dummy_img; // reused (via mp_image_new_ref) for every output frame
    struct mp_decoder public;
};

static const uint8_t *unit_data(struct pending_unit *u)
{
    return u->raw ? u->raw->buffer : u->bsf_pkt->data;
}

static int unit_size(struct pending_unit *u)
{
    return u->raw ? (int)u->raw->len : u->bsf_pkt->size;
}

static void unit_free(struct pending_unit *u)
{
    if (u->raw)
        talloc_free(u->raw);
    if (u->bsf_pkt)
        av_packet_free(&u->bsf_pkt);
    talloc_free(u);
}

static void queue_push_raw(struct priv *p, struct demux_packet *raw, int64_t pts_ns)
{
    struct pending_unit *u = talloc_zero(p, struct pending_unit);
    u->raw = raw;
    u->pts_ns = pts_ns;
    if (p->tail) {
        p->tail->next = u;
    } else {
        p->head = u;
    }
    p->tail = u;
}

static void queue_push_bsf(struct priv *p, AVPacket *pkt, int64_t pts_ns)
{
    struct pending_unit *u = talloc_zero(p, struct pending_unit);
    u->bsf_pkt = pkt;
    u->pts_ns = pts_ns;
    if (p->tail) {
        p->tail->next = u;
    } else {
        p->head = u;
    }
    p->tail = u;
}

static void queue_free_all(struct priv *p)
{
    struct pending_unit *u = p->head;
    while (u) {
        struct pending_unit *next = u->next;
        unit_free(u);
        u = next;
    }
    p->head = p->tail = NULL;
}

// Tries to feed the unit at the head of the queue. Leaves it there (for a
// later retry) if Starfish isn't ready for it yet.
static void feed_one_from_queue(struct mp_filter *vd)
{
    struct priv *p = vd->priv;
    struct pending_unit *u = p->head;
    if (!u)
        return;

    // No-ops unless a seek reset is actually pending -- see
    // starfish_bridge.h. Safe, and correct, to call unconditionally here:
    // the *first* fed unit after a reset is exactly the one whose pts should
    // be used to re-anchor Starfish's decode-start gate.
    starfish_bridge_prime_after_seek(u->pts_ns);

    const enum starfish_feed_result r =
        starfish_bridge_feed_video(unit_data(u), unit_size(u), u->pts_ns);

    if (r == STARFISH_FEED_RETRY)
        return; // stays at the head; the retry-pacer wakeup will call us again

    const int64_t pts_ns = u->pts_ns;

    p->head = u->next;
    if (!p->head)
        p->tail = NULL;
    unit_free(u);

    struct mp_image *img = mp_image_new_ref(p->dummy_img);
    if (img) {
        img->pts = pts_ns >= 0 ? (double)pts_ns / 1e9 : 0.0;
        mp_pin_in_write(vd->ppins[1], MAKE_FRAME(MP_FRAME_VIDEO, img));
    }

    mp_filter_internal_mark_progress(vd);
}

static void vd_webos_starfish_process(struct mp_filter *vd)
{
    struct priv *p = vd->priv;

    // Something's already waiting on Starfish -- retry it before accepting
    // more input, don't grow the queue while stalled.
    if (p->head) {
        feed_one_from_queue(vd);
        return;
    }

    if (!mp_pin_can_transfer_data(vd->ppins[1], vd->ppins[0]))
        return;

    struct mp_frame inframe = mp_pin_out_read(vd->ppins[0]);
    if (inframe.type == MP_FRAME_EOF) {
        mp_pin_in_write(vd->ppins[1], inframe);
        return;
    } else if (inframe.type == MP_FRAME_NONE) {
        return;
    } else if (inframe.type != MP_FRAME_PACKET) {
        MP_ERR(vd, "unknown frame type\n");
        mp_filter_internal_mark_failed(vd);
        return;
    }

    struct demux_packet *mpkt = inframe.data;
    const int64_t pts_ns = (mpkt->pts == MP_NOPTS_VALUE) ? -1 : (int64_t)(mpkt->pts * 1e9);

    if (p->bsf) {
        AVPacket *avpkt = av_packet_alloc();
        if (!avpkt) {
            talloc_free(mpkt);
            mp_filter_internal_mark_progress(vd);
            return;
        }
        // Deliberately not using mp_set_av_packet() here -- see the file
        // header comment on why we keep mpkt itself as the owner instead.
        avpkt->data = mpkt->buffer;
        avpkt->size = (int)mpkt->len;
        avpkt->pts = avpkt->dts = 0; // unused; we track timing via pts_ns above
        if (mpkt->keyframe)
            avpkt->flags |= AV_PKT_FLAG_KEY;

        if (av_bsf_send_packet(p->bsf, avpkt) < 0) {
            MP_ERR(vd, "av_bsf_send_packet failed\n");
            av_packet_free(&avpkt);
            talloc_free(mpkt);
            mp_filter_internal_mark_progress(vd);
            return;
        }
        av_packet_free(&avpkt); // send_packet only reads from it; mpkt stays alive below

        for (;;) {
            AVPacket *filtered = av_packet_alloc();
            if (!filtered)
                break;
            if (av_bsf_receive_packet(p->bsf, filtered) < 0) {
                av_packet_free(&filtered);
                break; // needs more input, or nothing buffered right now
            }
            queue_push_bsf(p, filtered, pts_ns);
        }

        talloc_free(mpkt); // safe now -- see file header comment
    } else {
        queue_push_raw(p, mpkt, pts_ns);
    }

    feed_one_from_queue(vd);
}

static void vd_webos_starfish_reset(struct mp_filter *vd)
{
    struct priv *p = vd->priv;
    queue_free_all(p);
    if (p->bsf)
        av_bsf_flush(p->bsf); // see StarfishMediaVideoPlayer's H265 seek bug this fixes
    starfish_bridge_notify_video_reset();
}

static void vd_webos_starfish_destroy(struct mp_filter *vd)
{
    struct priv *p = vd->priv;
    // Must happen before anything else: once unregistered, the retry-pacer
    // thread will no longer call mp_filter_wakeup() on this filter, which is
    // what makes the rest of this teardown safe.
    starfish_bridge_unregister_video_filter(vd);
    queue_free_all(p);
    av_bsf_free(&p->bsf);
}

static const struct mp_filter_info vd_webos_starfish_filter = {
    .name = "vd_webos_starfish",
    .priv_size = sizeof(struct priv),
    .process = vd_webos_starfish_process,
    .reset = vd_webos_starfish_reset,
    .destroy = vd_webos_starfish_destroy,
};

static bool init(struct mp_filter *vd, struct mp_codec_params *codec)
{
    struct priv *p = vd->priv;

    // Deliberately not requiring codec->lav_codecpar up front here (see the
    // file header comment) -- codec ID and dimensions come from
    // mp_codec_params' own fields, which don't depend on it. It's only
    // actually needed below, and only for H264/HEVC specifically.
    const int av_codec_id = mp_codec_to_av_codec_id(codec->codec);
    if (av_codec_id == AV_CODEC_ID_NONE) {
        MP_ERR(vd, "Could not map codec '%s' to an AVCodecID\n",
               codec->codec ? codec->codec : "(null)");
        return false;
    }

    const int width = codec->disp_w;
    const int height = codec->disp_h;
    if (width <= 0 || height <= 0) {
        MP_ERR(vd, "No usable video dimensions\n");
        return false;
    }

    if (av_codec_id == AV_CODEC_ID_H264 || av_codec_id == AV_CODEC_ID_HEVC) {
        // mp4toannexb needs a real AVCodecParameters -- to know whether the
        // source is already Annex B or length-prefixed (avcC/hvcC) and to
        // seed the NAL length size from it. Unlike codec ID/dimensions
        // above, there's no equivalent on mp_codec_params to fall back to;
        // if this is null here, that's what to chase (a demuxer/timing
        // issue, not something else feedable around the way the other
        // fields were).
        if (!codec->lav_codecpar) {
            MP_ERR(vd, "No AVCodecParameters available for bitstream filtering\n");
            return false;
        }

        const char *bsf_name =
            av_codec_id == AV_CODEC_ID_H264 ? "h264_mp4toannexb" : "hevc_mp4toannexb";
        const AVBitStreamFilter *bsf = av_bsf_get_by_name(bsf_name);
        if (!bsf || av_bsf_alloc(bsf, &p->bsf) < 0 ||
            avcodec_parameters_copy(p->bsf->par_in, codec->lav_codecpar) < 0)
        {
            MP_ERR(vd, "Failed to set up bitstream filter '%s'\n", bsf_name);
            return false;
        }
        p->bsf->time_base_in = codec->native_tb_den > 0
                                   ? (AVRational){codec->native_tb_num, codec->native_tb_den}
                                   : (AVRational){1, 1000000};
        if (av_bsf_init(p->bsf) < 0) {
            MP_ERR(vd, "Failed to initialize bitstream filter '%s'\n", bsf_name);
            return false;
        }
    }

    p->dummy_img = mp_image_alloc(IMGFMT_420P, 64, 64);
    if (!p->dummy_img) {
        MP_ERR(vd, "Failed to allocate placeholder image\n");
        return false;
    }
    talloc_steal(p, p->dummy_img);

    starfish_bridge_register_video(av_codec_id, width, height, codec->fps, codec->reliable_fps);
    starfish_bridge_register_video_filter(vd);

    return true;
}

static struct mp_decoder *create(struct mp_filter *parent, struct mp_codec_params *codec,
                                 const char *decoder)
{
    struct mp_filter *vd = mp_filter_create(parent, &vd_webos_starfish_filter);
    if (!vd)
        return NULL;

    mp_filter_add_pin(vd, MP_PIN_IN, "in");
    mp_filter_add_pin(vd, MP_PIN_OUT, "out");

    vd->log = mp_log_new(vd, parent->log, NULL);

    struct priv *p = vd->priv;
    p->codec = codec;
    p->public.f = vd;

    if (!init(vd, codec)) {
        talloc_free(vd);
        return NULL;
    }

    return &p->public;
}

// One shared decoder name across all supported codecs -- there's no
// sub-choice to make the way vd_lavc has to pick among several real FFmpeg
// decoder implementations, so this just needs to exist for
// reinit_decoder()'s mp_select_decoders() lookup to find us.
static void add_decoders(struct mp_decoder_list *list)
{
    mp_add_decoder(list, "h264", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "hevc", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "vp8", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "vp9", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "av1", "webos_starfish", "LG Starfish hardware decode");
}

const struct mp_decoder_fns vd_webos_starfish = {
    .create = create,
    .add_decoders = add_decoders,
};
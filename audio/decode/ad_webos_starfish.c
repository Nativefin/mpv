/*
 * LG webOS Starfish hardware-decode backend for mpv -- audio half.
 *
 * Structurally this is ad_spdif.c's own pattern (forward compressed access
 * units immediately, don't decode, don't try to own timing) rather than
 * anything modeled on ad_lavc.c -- see vd_webos_starfish.c's file header for
 * why that's the right model here. No bitstream filtering is needed for any
 * of the audio codecs Starfish accepts (see starfish_bridge.cpp's codec
 * table), so unlike the video side this only ever needs to track a single
 * pending unit, not a small FIFO.
 *
 * The frames this emits are one-sample silent placeholders -- carrying real
 * pts so mpv's own EOF/stats/AV-sync bookkeeping has something to observe,
 * but no real audio. The intended AO is --ao=null; nothing downstream of
 * this filter is ever meant to actually play anything.
 *
 * init() deliberately does not require codec->lav_codecpar. Its own doc
 * comment in demux/stheader.h says it's only "set by demux_{lavf,mkv,raw}",
 * and on-device testing found it reliably NULL here regardless of demuxer --
 * every mp_codec_params field this actually needs (codec name, channels,
 * samplerate, extradata) is populated unconditionally by the demuxer
 * instead, so that's what's used. Only the numeric codec profile (used to
 * refine AC3/EAC3-Atmos and DTS-ES/DTS-HD-MA-X into their more specific
 * Starfish codec strings -- see starfish_bridge.cpp) has no equivalent
 * outside lav_codecpar, so that one field is read opportunistically and
 * left at 0 (no refinement) if it isn't there.
 */

#include <libavcodec/avcodec.h>

#include "audio/aframe.h"
#include "audio/chmap.h"
#include "audio/format.h"
#include "common/av_common.h"
#include "common/msg.h"
#include "demux/packet.h"
#include "demux/stheader.h"
#include "filters/f_decoder_wrapper.h"
#include "filters/filter_internal.h"
#include "misc/starfish_bridge.h"
#include "mpv_talloc.h"
#include "common/codecs.h"

struct priv {
    struct mp_codec_params *codec;
    struct demux_packet *pending; // at most one -- held across process() calls for retry
    struct mp_aframe *dummy_fmt;  // template for the silent placeholder frames we emit
    struct mp_decoder public;
};

static void ad_webos_starfish_process(struct mp_filter *ad)
{
    struct priv *p = ad->priv;

    if (!p->pending) {
        if (!mp_pin_can_transfer_data(ad->ppins[1], ad->ppins[0]))
            return;

        struct mp_frame inframe = mp_pin_out_read(ad->ppins[0]);
        if (inframe.type == MP_FRAME_EOF) {
            mp_pin_in_write(ad->ppins[1], inframe);
            return;
        } else if (inframe.type == MP_FRAME_NONE) {
            return;
        } else if (inframe.type != MP_FRAME_PACKET) {
            MP_ERR(ad, "unknown frame type\n");
            mp_filter_internal_mark_failed(ad);
            return;
        }
        p->pending = inframe.data;
    }

    const int64_t pts_ns =
        (p->pending->pts == MP_NOPTS_VALUE) ? -1 : (int64_t)(p->pending->pts * 1e9);

    const enum starfish_feed_result r =
        starfish_bridge_feed_audio(p->pending->buffer, (int)p->pending->len, pts_ns);

    if (r == STARFISH_FEED_RETRY)
        return; // stays pending; the retry-pacer wakeup will call us again

    talloc_free(p->pending);
    p->pending = NULL;

    struct mp_aframe *frame = mp_aframe_new_ref(p->dummy_fmt);
    if (frame) {
        mp_aframe_set_pts(frame, pts_ns >= 0 ? (double)pts_ns / 1e9 : 0.0);
        mp_pin_in_write(ad->ppins[1], MAKE_FRAME(MP_FRAME_AUDIO, frame));
    }

    mp_filter_internal_mark_progress(ad);
}

static void ad_webos_starfish_reset(struct mp_filter *ad)
{
    struct priv *p = ad->priv;
    if (p->pending) {
        talloc_free(p->pending);
        p->pending = NULL;
    }
    starfish_bridge_notify_audio_reset();
}

static void ad_webos_starfish_destroy(struct mp_filter *ad)
{
    struct priv *p = ad->priv;
    // Must happen first -- see the identical note in vd_webos_starfish.c's destroy.
    starfish_bridge_unregister_audio_filter(ad);
    if (p->pending) {
        talloc_free(p->pending);
        p->pending = NULL;
    }
}

static const struct mp_filter_info ad_webos_starfish_filter = {
    .name = "ad_webos_starfish",
    .priv_size = sizeof(struct priv),
    .process = ad_webos_starfish_process,
    .reset = ad_webos_starfish_reset,
    .destroy = ad_webos_starfish_destroy,
};

static bool init(struct mp_filter *ad, struct mp_codec_params *codec)
{
    struct priv *p = ad->priv;

    // Deliberately not requiring codec->lav_codecpar here (see the file
    // header comment) -- mp_codec_params carries everything Load() needs
    // for audio directly, unconditionally populated by every demuxer, not
    // gated behind FFmpeg-parameter conversion the way lav_codecpar is.
    const int av_codec_id = mp_codec_to_av_codec_id(codec->codec);
    if (av_codec_id == AV_CODEC_ID_NONE) {
        MP_ERR(ad, "Could not map codec '%s' to an AVCodecID\n",
               codec->codec ? codec->codec : "(null)");
        return false;
    }

    const int channels = codec->channels.num;
    const int sample_rate = codec->samplerate;
    if (channels <= 0 || sample_rate <= 0) {
        MP_ERR(ad, "No usable audio format info\n");
        return false;
    }

    // profile has no equivalent on mp_codec_params (codec_profile there is
    // a display string, not the numeric AV_PROFILE_* value StarfishBridge
    // matches against) -- opportunistic only. Fine if lav_codecpar isn't
    // available: it only gates a codec-string refinement (Atmos channel
    // bump, DTSE/DTSX) in BuildAudioContentsLocked, not whether audio works
    // at all -- see starfish_bridge.cpp.
    const int profile = codec->lav_codecpar ? codec->lav_codecpar->profile : 0;

    p->dummy_fmt = mp_aframe_create();
    mp_aframe_set_format(p->dummy_fmt, AF_FORMAT_S16);
    mp_aframe_set_rate(p->dummy_fmt, 8000);
    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, 1);
    mp_aframe_set_chmap(p->dummy_fmt, &chmap);
    if (!mp_aframe_alloc_data(p->dummy_fmt, 1)) {
        MP_ERR(ad, "Failed to allocate placeholder audio frame\n");
        return false;
    }
    mp_aframe_set_silence(p->dummy_fmt, 0, 1);
    talloc_steal(p, p->dummy_fmt);

    starfish_bridge_register_audio(av_codec_id, channels, sample_rate, codec->extradata,
                                   codec->extradata_size, profile);
    starfish_bridge_register_audio_filter(ad);

    return true;
}

static struct mp_decoder *create(struct mp_filter *parent, struct mp_codec_params *codec,
                                 const char *decoder)
{
    struct mp_filter *ad = mp_filter_create(parent, &ad_webos_starfish_filter);
    if (!ad)
        return NULL;

    mp_filter_add_pin(ad, MP_PIN_IN, "in");
    mp_filter_add_pin(ad, MP_PIN_OUT, "out");

    ad->log = mp_log_new(ad, parent->log, NULL);

    struct priv *p = ad->priv;
    p->codec = codec;
    p->public.f = ad;

    if (!init(ad, codec)) {
        talloc_free(ad);
        return NULL;
    }

    return &p->public;
}

// See vd_webos_starfish.c's add_decoders -- same reasoning for using one
// shared decoder name across every codec Starfish accepts here.
static void add_decoders(struct mp_decoder_list *list)
{
    mp_add_decoder(list, "ac3", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "eac3", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "ac4", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "dts", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "opus", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "mp3", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "aac", "webos_starfish", "LG Starfish hardware decode");
    mp_add_decoder(list, "aac_latm", "webos_starfish", "LG Starfish hardware decode");
}

const struct mp_decoder_fns ad_webos_starfish = {
    .create = create,
    .add_decoders = add_decoders,
};
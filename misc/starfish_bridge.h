#pragma once

// Shared entry point into a single StarfishMediaAPIs pipeline instance, used
// by three independent callers: your application (session lifecycle, pause,
// position queries), video/decode/vd_webos_starfish.c, and
// audio/decode/ad_webos_starfish.c. This header is plain C so both mpv's C
// decoder files and your (presumably C++) app can include it directly; the
// implementation (starfish_bridge.cpp) is C++ and is the only file that
// actually touches StarfishMediaAPIs, player-factory/custompipeline.hpp, or
// player-factory/customplayer.hpp.
//
// Why a bridge at all, instead of vd/ad_webos_starfish.c calling
// StarfishMediaAPIs directly: Starfish's Load() is a single whole-pipeline
// call describing both the video AND audio codec at once, but mpv creates
// (and may destroy/recreate) the video and audio decoders independently, in
// whatever order its own track-selection logic decides, and Starfish has no
// audio-only mode. Something has to sit above both decoders and hold the one
// StarfishMediaAPIs instance they both feed into -- this is that something.
// It also keeps the private LG headers (custompipeline.hpp/customplayer.hpp,
// needed for sendSegmentEvent()) and all C++ out of mpv's own C build.
//
// Threading model, so callers get the locking right:
//  - App-facing functions (begin/end_session, set_paused, the getters):
//    call these from one consistent thread (e.g. wherever you already drive
//    libmpv), not concurrently with each other.
//  - Decoder-facing register_*/feed_*/notify_*_reset/prime_after_seek:
//    called from whatever thread mpv drives that decoder's process()/reset()
//    on. The bridge internally serializes all of this itself (one mutex
//    covers all shared state) -- callers don't need their own locking.
//  - Starfish's own callback fires on Starfish's internal thread, independent
//    of both of the above. Also handled internally.

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum starfish_feed_result {
    STARFISH_FEED_OK = 0,    // consumed -- free the unit, move to the next one
    STARFISH_FEED_RETRY = 1, // not consumed -- keep the unit, try again later
                              // (BufferFull, feed-ahead throttle, or Load()
                              // hasn't completed yet); a wakeup will arrive
                              // via the function set in
                              // starfish_bridge_set_wakeup_fn() once retrying
                              // is likely to be worth it, but it's a coarse
                              // ~10ms poke, not a precise signal -- retrying
                              // and getting STARFISH_FEED_RETRY again is
                              // normal and not an error.
    STARFISH_FEED_DROP = 2,  // consumed but discarded (bad/negative pts, or
                              // Feed() returned something unrecognized) --
                              // free the unit, move to the next one
};

// Called once, from mpv's own C code (see the shim added near
// reinit_decoder() in filters/f_decoder_wrapper.c), before anything else in
// this header is used. Lets the bridge's background retry-pacer thread (see
// STARFISH_FEED_RETRY above) nudge a stalled decoder filter without this
// header -- or the app that includes it -- needing to know mp_filter's real
// definition. filter_handle is whatever you passed to
// starfish_bridge_register_{video,audio}_filter().
typedef void (*starfish_wakeup_fn)(void* filter_handle);
void starfish_bridge_set_wakeup_fn(starfish_wakeup_fn fn);

// ---------------------------------------------------------------------
// App-facing: session lifecycle, playback state, queries
// ---------------------------------------------------------------------

// Call once, after you've created the exported window (windowId is that
// window's id, e.g. from SDL_webOSCreateExportedWindow) and before telling
// libmpv to load a URL. Both strings are copied internally; you don't need
// to keep them alive after this call returns.
void starfish_bridge_begin_session(const char* app_id, const char* window_id, int platformCode);

// Call this FIRST when tearing down, before mpv_terminate_destroy() (or
// whatever you use to destroy the mpv_handle) -- not after, and not instead
// of starfish_bridge_end_session() below, both are required and in this
// order. Stops the background retry-pacer thread, which otherwise keeps
// calling into mpv's filter graph on its own timer independent of mpv's own
// teardown -- letting it keep running during mpv's destruction is what
// causes quitting to hang. Safe to call even if playback never started.
void starfish_bridge_prepare_shutdown(void);

// Call SECOND, after mpv_terminate_destroy() has returned (so both decoder
// filters have already run their .destroy callback and unregistered
// themselves -- see the ordering note above register_*_filter below).
// Blocks briefly (bounded, ~1s) waiting for a clean Unload().
void starfish_bridge_end_session(void);

// Mirror mpv's own `pause` property here, e.g. from an mpv_observe_property
// callback in your app. Starfish runs its own internal clock, independent of
// mpv's -- mpv pausing its pipeline does not by itself pause Starfish's.
void starfish_bridge_set_paused(bool paused);

bool starfish_bridge_is_loaded(void);
bool starfish_bridge_has_ended(void);   // true after Starfish's own ENDOFSTREAM event
int64_t starfish_bridge_get_position_ns(void);

// Valid until the next bridge call on any thread. Copy it if you need it to
// live longer.
const char* starfish_bridge_get_last_error(void);

// ---------------------------------------------------------------------
// mpv-core-facing: codec support queries, called only from
// filters/f_decoder_wrapper.c's reinit_decoder(), before a decoder exists
// ---------------------------------------------------------------------

// codec is mp_codec_params->codec (mpv/FFmpeg's short codec name, e.g.
// "h264", "av1", "opus") -- the string form, since at driver-selection time
// that's what reinit_decoder() has; lav_codecpar may not be populated yet
// for every demuxer at this point, so this deliberately doesn't require it.
bool starfish_bridge_video_codec_supported(const char* codec);
bool starfish_bridge_audio_codec_supported(const char* codec);

// ---------------------------------------------------------------------
// Decoder-facing: call only from vd_webos_starfish.c / ad_webos_starfish.c
// ---------------------------------------------------------------------

// filter_handle: your struct mp_filter*, opaque to the bridge. Register at
// the end of create(), *after* you know create() is going to succeed;
// unregister at the very start of destroy(), before any other teardown --
// a queued wakeup firing into a filter mid-teardown (or freed) is a
// use-after-free.
void starfish_bridge_register_video_filter(void* filter_handle);
void starfish_bridge_unregister_video_filter(void* filter_handle);
void starfish_bridge_register_audio_filter(void* filter_handle);
void starfish_bridge_unregister_audio_filter(void* filter_handle);

// Call once from each decoder's create(). av_codec_id is a real enum
// AVCodecID (cast to int, so this header doesn't need
// <libavcodec/avcodec.h>) -- get it via mp_codec_to_av_codec_id(codec->codec)
// (common/av_common.h), not from codec->lav_codecpar->codec_id. That field's
// own doc comment in demux/stheader.h says it's only "set by
// demux_{lavf,mkv,raw}", and on-device testing found it unreliably
// populated regardless of demuxer -- mp_codec_params' own fields
// (codec/channels/samplerate/extradata/disp_w/disp_h) are what every
// demuxer populates unconditionally; lav_codecpar is only genuinely needed
// for H264/HEVC's bitstream filter setup (see vd_webos_starfish.c). Width/
// height should be codec->disp_w/disp_h. extradata may be NULL/0. profile
// is the AVCodecParameters profile field (0/unknown if you don't have one,
// e.g. because lav_codecpar wasn't available -- see ad_webos_starfish.c).
//
// This may block briefly (bounded, a few seconds -- see kAudioRegisterWait
// in starfish_bridge.cpp): if this is the video side and no audio has
// registered yet, it waits for an audio decoder to show up (mpv creates
// video/audio decoders independently and in no guaranteed order, but
// Load() needs both codecs at once if the file has audio at all) before
// proceeding either way. Don't call this from a latency-sensitive path --
// create() is fine, it's a one-time cost per session. If audio ever goes
// missing intermittently even with this wait, that's a sign the timeout
// heuristic isn't wide enough for your content/hardware and it's worth
// replacing this whole wait with something deterministic instead -- e.g.
// have the app hold registered video/audio info without triggering Load()
// at all until it observes MPV_EVENT_FILE_LOADED (documented as firing
// once decoding has started, i.e. after every selected track's decoder
// already exists), then calls an explicit "commit" function.
void starfish_bridge_register_video(int av_codec_id, int width, int height,
                                    double fps, bool fps_reliable);
void starfish_bridge_register_audio(int av_codec_id, int channels,
                                    int sample_rate, const uint8_t* extradata,
                                    int extradata_size, int profile);

// Attempts to feed one access unit. data must stay valid and unmodified
// until this returns something other than STARFISH_FEED_RETRY -- if you get
// STARFISH_FEED_RETRY, call again later with the *same* data/size/pts, don't
// advance to the next unit yet.
enum starfish_feed_result starfish_bridge_feed_video(const uint8_t* data, int size,
                                                      int64_t pts_ns);
enum starfish_feed_result starfish_bridge_feed_audio(const uint8_t* data, int size,
                                                      int64_t pts_ns);

// Call from both decoders' .reset callback (mpv calls .reset on every seek).
// Safe to call from both even for the same seek -- the underlying flush()
// only actually happens once per reset cycle.
void starfish_bridge_notify_video_reset(void);
void starfish_bridge_notify_audio_reset(void);

// Call once, from vd_webos_starfish.c's process(), for the first video unit
// fed after starfish_bridge_notify_video_reset() -- landed_pts_ns must be
// that unit's own pts (after any bitstream filtering), not your seek
// target; AVSEEK_FLAG_BACKWARD can land slightly before it. No-ops if
// nothing is pending (i.e. you call it more than once per reset, or before
// any reset happened) -- safe to call defensively.
void starfish_bridge_prime_after_seek(int64_t landed_pts_ns);

#ifdef __cplusplus
}
#endif
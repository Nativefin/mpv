#include "starfish_bridge.h"

#include <atomic>
#include <cinttypes>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include <starfish-media-pipeline/StarfishMediaAPIs.h>

// Needed only for sendSegmentEvent() inside PrimeAfterSeekLocked() -- see the
// comment there. Not part of the public StarfishMediaAPIs.h dump; matches
// the exact cast chain already validated in StarfishMediaVideoPlayer.
#include <player-factory/custompipeline.hpp>
#include <player-factory/customplayer.hpp>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/rational.h>
}

namespace
{

// ---------------------------------------------------------------------
// Codec name table -- identical in spirit to the CODECS map from
// StarfishMediaVideoPlayer.cpp, just living here now that Load() is built
// in one place shared by both decoders instead of duplicated per-app.
// ---------------------------------------------------------------------

const std::map<AVCodecID, std::string>& CodecNames()
{
  static const std::map<AVCodecID, std::string> map = {
      {AV_CODEC_ID_H264, "H264"},   {AV_CODEC_ID_HEVC, "H265"},
      {AV_CODEC_ID_VP8, "VP8"},     {AV_CODEC_ID_VP9, "VP9"},
      {AV_CODEC_ID_AV1, "AV1"},     {AV_CODEC_ID_AC3, "AC3"},
      {AV_CODEC_ID_EAC3, "AC3 PLUS"}, {AV_CODEC_ID_AC4, "AC4"},
      {AV_CODEC_ID_DTS, "DTS"},     {AV_CODEC_ID_OPUS, "OPUS"},
      {AV_CODEC_ID_MP3, "MP3"},     {AV_CODEC_ID_AAC, "AAC"},
      {AV_CODEC_ID_AAC_LATM, "AAC"},
  };
  return map;
}

bool IsSupportedVideoCodec(AVCodecID id)
{
  switch (id)
  {
    case AV_CODEC_ID_H264:
    case AV_CODEC_ID_HEVC:
    case AV_CODEC_ID_VP8:
    case AV_CODEC_ID_VP9:
    case AV_CODEC_ID_AV1:
      return true;
    default:
      return false;
  }
}

bool IsSupportedAudioCodec(AVCodecID id)
{
  switch (id)
  {
    case AV_CODEC_ID_AC3:
    case AV_CODEC_ID_EAC3:
    case AV_CODEC_ID_AC4:
    case AV_CODEC_ID_DTS:
    case AV_CODEC_ID_OPUS:
    case AV_CODEC_ID_MP3:
    case AV_CODEC_ID_AAC:
    case AV_CODEC_ID_AAC_LATM:
      return true;
    default:
      return false;
  }
}

// mpv/FFmpeg short codec-name strings for the same two sets, used only by
// the two *_codec_supported() queries (see starfish_bridge.h for why those
// take a string instead of an AVCodecID).
bool NameInSet(const char* name, std::initializer_list<const char*> set)
{
  if (!name)
    return false;
  for (const char* n : set)
  {
    if (std::strcmp(name, n) == 0)
      return true;
  }
  return false;
}

constexpr int64_t kMaxFeedAheadNs = 1'600'000'000; // 1.6s, see prior art throughout this project
constexpr auto kAudioRegisterWait = std::chrono::milliseconds(500);
constexpr auto kUnloadWait = std::chrono::seconds(1);
constexpr auto kPacerInterval = std::chrono::milliseconds(10);

std::string Base64Encode(const uint8_t* data, size_t len)
{
  if (!data || len == 0)
    return {};

  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string out;
  out.reserve(((len + 2) / 3) * 4);

  size_t i = 0;
  for (; i + 3 <= len; i += 3)
  {
    const uint32_t n =
        (uint32_t(data[i]) << 16) | (uint32_t(data[i + 1]) << 8) | uint32_t(data[i + 2]);
    out += table[(n >> 18) & 0x3F];
    out += table[(n >> 12) & 0x3F];
    out += table[(n >> 6) & 0x3F];
    out += table[n & 0x3F];
  }

  const size_t rem = len - i;
  if (rem == 1)
  {
    const uint32_t n = uint32_t(data[i]) << 16;
    out += table[(n >> 18) & 0x3F];
    out += table[(n >> 12) & 0x3F];
    out += "==";
  }
  else if (rem == 2)
  {
    const uint32_t n = (uint32_t(data[i]) << 16) | (uint32_t(data[i + 1]) << 8);
    out += table[(n >> 18) & 0x3F];
    out += table[(n >> 12) & 0x3F];
    out += table[(n >> 6) & 0x3F];
    out += '=';
  }

  return out;
}

// Ported verbatim from StarfishMediaVideoPlayer.cpp.
unsigned int ParseAACSampleRate(const uint8_t* data, size_t size)
{
  if (size < 2)
    return 0;

  static const unsigned int sampleRates[16] = {96000, 88200, 64000, 48000, 44100, 32000,
                                               24000, 22050, 16000, 12000, 11025, 8000,
                                               7350,  0,     0,     0};

  constexpr unsigned int AAC_AOT_AAC_LC = 2;
  constexpr unsigned int AAC_AOT_SBR = 5;
  constexpr unsigned int AAC_AOT_PS = 29;
  constexpr size_t MIN_SIZE_FOR_SBR = 5;
  constexpr unsigned int MAX_SAMPLE_INDEX = 15;

  const unsigned int aot = (data[0] >> 3) & 0x1F;
  const unsigned int srIndex = ((data[0] & 0x07) << 1) | (data[1] >> 7);
  unsigned int sampleRate = (srIndex <= MAX_SAMPLE_INDEX) ? sampleRates[srIndex] : 0;

  if ((aot == AAC_AOT_AAC_LC || aot == AAC_AOT_SBR || aot == AAC_AOT_PS) &&
      size >= MIN_SIZE_FOR_SBR)
  {
    const unsigned int extAot = (data[2] >> 3) & 0x1F;
    if (extAot == AAC_AOT_SBR || extAot == AAC_AOT_PS)
    {
      const unsigned int extSrIndex = ((data[2] & 0x07) << 1) | (data[3] >> 7);
      if (extSrIndex <= MAX_SAMPLE_INDEX)
        sampleRate = sampleRates[extSrIndex];
    }
  }

  return sampleRate;
}

// ---------------------------------------------------------------------
// Bridge state -- one session at a time, guarded by g_mutex throughout.
// See the class-level threading note in starfish_bridge.h.
// ---------------------------------------------------------------------

struct VideoInfo
{
  AVCodecID codecId{AV_CODEC_ID_NONE};
  int width{0};
  int height{0};
  double fps{0.0};
  bool fpsReliable{false};
};

struct AudioInfo
{
  AVCodecID codecId{AV_CODEC_ID_NONE};
  int channels{0};
  int sampleRate{0};
  std::vector<uint8_t> extradata;
  int profile{0};
};

std::recursive_mutex g_mutex; // recursive: see the comment above BuildAndSendLoadLocked's Load() call
std::condition_variable_any g_cv;

std::unique_ptr<StarfishMediaAPIs> g_api;
std::string g_appId;
std::string g_windowId;
bool g_sessionActive = false;

bool g_hasVideo = false;
bool g_hasAudio = false;
bool g_audioWaitDone = false;
bool g_loadTriggered = false;
VideoInfo g_videoInfo;
AudioInfo g_audioInfo;

bool g_loaded = false;
bool g_paused = false;
bool g_started = false;
bool g_ended = false;
std::atomic<int64_t> g_currentPtsNs{0};
std::atomic<int64_t> g_fedVideoPtsNs{-1};
std::atomic<int64_t> g_fedAudioPtsNs{-1};

bool g_flushInFlight = false;   // guards against double flush() if both
                                // decoders' .reset fire for the same seek
bool g_needsTimeToDecode = false; // true only between a completed reset and
                                  // the next starfish_bridge_prime_after_seek()

std::string g_lastError;

// Deliberately a *separate* mutex from g_mutex, guarding only filter-pointer
// registration and the wakeup call itself. mp_filter_wakeup() (see
// filters/filter.c) takes an mpv-internal lock while it runs; calling it
// while holding g_mutex risks a cross-library deadlock against mpv's own
// core thread calling back into this bridge (which needs g_mutex) while
// holding that same internal lock or something ordered behind it. This is
// not a hypothetical -- it's what caused pause/seek/quit to hang the first
// time this shipped. Keeping this mutex's critical sections small, and
// never taking g_mutex while holding it (or vice versa), is what actually
// matters here, more than which specific mpv-internal lock is involved.
std::mutex g_filterMutex;
std::condition_variable g_filterCv;
void* g_videoFilter = nullptr;
void* g_audioFilter = nullptr;
int g_videoWakeupInFlight = 0;
int g_audioWakeupInFlight = 0;
starfish_wakeup_fn g_wakeupFn = nullptr;

std::thread g_pacerThread;
std::atomic<bool> g_pacerRunning{false};

void SetErrorLocked(const std::string& msg)
{
  g_lastError = msg;
  fprintf(stderr, "[StarfishBridge] %s\n", msg.c_str());
}

void PacerLoop()
{
  while (g_pacerRunning.load())
  {
    std::this_thread::sleep_for(kPacerInterval);

    void* videoFilter = nullptr;
    void* audioFilter = nullptr;
    starfish_wakeup_fn wakeupFn = nullptr;
    {
      std::lock_guard lock(g_filterMutex);
      wakeupFn = g_wakeupFn;
      if (g_videoFilter)
      {
        videoFilter = g_videoFilter;
        ++g_videoWakeupInFlight;
      }
      if (g_audioFilter)
      {
        audioFilter = g_audioFilter;
        ++g_audioWakeupInFlight;
      }
    }

    // No bridge lock held here -- see the g_filterMutex comment above for
    // why that's load-bearing, not just tidiness. process() itself is cheap
    // to no-op if there's nothing to do, so calling both unconditionally
    // (rather than tracking a separate "is a retry actually pending" flag
    // per lane) is fine -- see STARFISH_FEED_RETRY's doc comment.
    if (wakeupFn)
    {
      if (videoFilter)
        wakeupFn(videoFilter);
      if (audioFilter)
        wakeupFn(audioFilter);
    }

    if (videoFilter || audioFilter)
    {
      std::lock_guard lock(g_filterMutex);
      if (videoFilter)
        --g_videoWakeupInFlight;
      if (audioFilter)
        --g_audioWakeupInFlight;
      g_filterCv.notify_all();
    }
  }
}

void BuildAudioContentsLocked(nlohmann::json& contents)
{
  const auto& names = CodecNames();
  auto it = names.find(g_audioInfo.codecId);
  std::string audioCodec = (it != names.end()) ? it->second : "";

  const uint8_t* extradata = g_audioInfo.extradata.empty() ? nullptr : g_audioInfo.extradata.data();
  const int extradataSize = static_cast<int>(g_audioInfo.extradata.size());

  switch (g_audioInfo.codecId)
  {
    case AV_CODEC_ID_AC3:
    case AV_CODEC_ID_EAC3:
      contents["ac3PlusInfo"]["channels"] = g_audioInfo.channels;
      contents["ac3PlusInfo"]["frequency"] = g_audioInfo.sampleRate / 1000.0;
      if (g_audioInfo.profile == AV_PROFILE_EAC3_DDP_ATMOS)
        contents["ac3PlusInfo"]["channels"] = g_audioInfo.channels + 2;
      break;
    case AV_CODEC_ID_AC4:
      contents["ac4Info"]["channels"] = g_audioInfo.channels;
      contents["ac4Info"]["frequency"] = g_audioInfo.sampleRate / 1000.0;
      break;
    case AV_CODEC_ID_DTS:
      contents["dtsInfo"]["channels"] = g_audioInfo.channels;
      contents["dtsInfo"]["frequency"] = g_audioInfo.sampleRate / 1000.0;
      if (g_audioInfo.profile == AV_PROFILE_DTS_ES)
        audioCodec = "DTSE";
      if (g_audioInfo.profile == AV_PROFILE_DTS_HD_MA_X ||
          g_audioInfo.profile == AV_PROFILE_DTS_HD_MA_X_IMAX)
        audioCodec = "DTSX";
      break;
    case AV_CODEC_ID_OPUS:
      contents["opusInfo"]["channels"] = g_audioInfo.channels;
      contents["opusInfo"]["frequency"] = g_audioInfo.sampleRate / 1000.0;
      contents["opusInfo"]["streamHeader"] = Base64Encode(extradata, extradataSize);
      break;
    case AV_CODEC_ID_AAC:
    case AV_CODEC_ID_AAC_LATM:
    {
      contents["aacInfo"]["channels"] = g_audioInfo.channels;
      contents["aacInfo"]["profile"] = g_audioInfo.profile + 1;
      contents["aacInfo"]["format"] = extradata ? "raw" : "adts";

      double sampleRate = g_audioInfo.sampleRate / 1000.0;
      if (extradata && extradataSize > 0)
      {
        const unsigned int parsedRate = ParseAACSampleRate(extradata, extradataSize);
        if (parsedRate > 0)
          sampleRate = parsedRate / 1000.0;
      }
      contents["aacInfo"]["frequency"] = sampleRate;
      break;
    }
    default:
      break;
  }

  contents["codec"]["audio"] = audioCodec;
}

// Requires g_mutex held. Returns false (and sets g_lastError) only on a
// synchronous Load() failure; the async path (LOADCOMPLETED/timeout) is the
// caller's problem via starfish_bridge_is_loaded()/get_last_error().
bool BuildAndSendLoadLocked();

void TryBeginLoadLocked(std::unique_lock<std::recursive_mutex> &lock)
{
  if (g_loadTriggered || !g_hasVideo)
    return;

  if (!g_hasAudio && !g_audioWaitDone)
  {
    // Bounded wait: give an audio decoder, if this file has one, a chance to
    // register before we commit to a video-only Load(). mpv creates the two
    // decoders independently and in no order this bridge can rely on.
    g_cv.wait_for(lock, kAudioRegisterWait, [] { return g_hasAudio; });
    g_audioWaitDone = true;
    if (g_loadTriggered) // a racing audio registration could have triggered it while we waited
      return;
  }

  g_loadTriggered = true;
  BuildAndSendLoadLocked();
}

bool BuildAndSendLoadLocked()
{
  const auto& names = CodecNames();
  auto vit = names.find(g_videoInfo.codecId);
  const std::string videoCodec = (vit != names.end()) ? vit->second : "";

  nlohmann::json p;
  p["mediaTransportType"] = "BUFFERSTREAM";
  p["option"]["appId"] = g_appId;
  p["option"]["windowId"] = g_windowId;
  p["option"]["needAudio"] = g_hasAudio;
  p["option"]["seekMode"] = "keep-rate";
  p["option"]["useDroppedFrameEvent"] = true;
  p["option"]["transmission"]["contentsType"] = "LIVE";
  p["option"]["transmission"]["trickType"] = "client-side";

  int32_t maxW = 0, maxH = 0, maxFr = 0;
  // Matches the explicit function-pointer cast already required in
  // StarfishMediaVideoPlayer.cpp -- some StarfishMediaAPIs.h revisions carry
  // both the by-value and const-ref overloads active at once, which is
  // otherwise ambiguous.
  if (static_cast<bool (*)(std::string, int32_t*, int32_t*, int32_t*)>(
          &smp::util::getMaxVideoResolution)(videoCodec, &maxW, &maxH, &maxFr))
  {
    p["option"]["adaptiveStreaming"]["adaptiveResolution"] = true;
    p["option"]["adaptiveStreaming"]["maxWidth"] = maxW;
    p["option"]["adaptiveStreaming"]["maxHeight"] = maxH;
    p["option"]["adaptiveStreaming"]["maxFrameRate"] = maxFr;
  }

  p["option"]["externalStreamingInfo"]["streamQualityInfo"] = true;
  p["option"]["externalStreamingInfo"]["streamQualityInfoNonFlushable"] = false;
  p["option"]["externalStreamingInfo"]["streamQualityInfoCorruptedFrame"] = true;

  p["option"]["externalStreamingInfo"]["contents"]["codec"]["video"] = videoCodec;
  p["option"]["externalStreamingInfo"]["contents"]["format"] = "RAW";
  p["option"]["externalStreamingInfo"]["contents"]["provider"] = g_appId;

  if (g_hasAudio)
    BuildAudioContentsLocked(p["option"]["externalStreamingInfo"]["contents"]);

  p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["pauseAtDecodeTime"] = true;
  p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["seperatedPTS"] = true;
  p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["ptsToDecode"] = 0;
  p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["videoWidth"] = g_videoInfo.width;
  p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["videoHeight"] = g_videoInfo.height;
  if (g_videoInfo.fpsReliable && g_videoInfo.fps > 0.0)
  {
    const AVRational fr = av_d2q(g_videoInfo.fps, 100000);
    p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["videoFpsValue"] = fr.num;
    p["option"]["externalStreamingInfo"]["contents"]["esInfo"]["videoFpsScale"] = fr.den;
  }

  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["preBufferByte"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["bufferMinLevel"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["bufferMaxLevel"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["qBufferLevelVideo"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["srcBufferLevelVideo"]["minimum"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["srcBufferLevelVideo"]["maximum"] =
      8 * 1024 * 1024;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["qBufferLevelAudio"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["srcBufferLevelAudio"]["minimum"] = 0;
  p["option"]["externalStreamingInfo"]["bufferingCtrInfo"]["srcBufferLevelAudio"]["maximum"] =
      2 * 1024 * 1024;

  nlohmann::json payloadArgs;
  payloadArgs["args"] = nlohmann::json::array();
  payloadArgs["args"].push_back(p);
  const std::string payload = payloadArgs.dump();

  // Matches StarfishMediaVideoPlayer's established call -- not
  // notifyForeground(); kept consistent with what's already working there.
  if (!g_api->notifyBackground())
    fprintf(stderr, "[StarfishBridge] notifyBackground() returned false, continuing anyway\n");

  // g_mutex is held for this entire call, including by every other function
  // in this file that reaches Load()/Feed()/Play()/Pause()/flush()/Unload()/
  // setTimeToDecode() -- all of them keep g_mutex held across the call into
  // StarfishMediaAPIs. That's deliberate, not an oversight: the leading
  // theory for why create() was hanging (before this was made a recursive
  // mutex) is that the callback below fired synchronously, on this same
  // thread, from inside this same Load() call, before Load() returned --
  // BuildAndSendLoadLocked already held g_mutex at that point, so the
  // callback trying to lock the same (then plain) mutex on the same thread
  // would hang forever rather than fail loudly, which fits everything
  // observed (create() never returning, mpv's core thread wedged, nothing
  // downstream of it able to make progress). g_mutex is a
  // std::recursive_mutex specifically to make that safe regardless of which
  // StarfishMediaAPIs call turns out to re-enter and when -- chosen over
  // auditing and unlocking around every individual call site because
  // "some StarfishMediaAPIs method might call back into us before
  // returning" is a property of an undocumented API this project has
  // already been wrong about the exact shape of more than once; recursion-
  // safety here doesn't depend on knowing which call it is or catching
  // every one by hand.
  if (!g_api->Load(
          payload.c_str(),
          [](int type, int64_t numValue, const char* strValue, void*) {
            std::lock_guard lock(g_mutex);
            switch (type)
            {
              case PF_EVENT_TYPE_FRAMEREADY:
                g_currentPtsNs = numValue;
                g_started = true;
                break;

              case PF_EVENT_TYPE_STR_STATE_UPDATE__LOADCOMPLETED:
                g_loaded = true;
                // g_api can be null here if end_session() raced this
                // callback -- see end_session()'s comment. There's no way
                // to cancel a pending Load(), so this null check is the
                // most this bridge (or the app-level code this project
                // started from) can do about that; treat it as a known,
                // rare edge case rather than a bug to keep chasing.
                if (g_api && !g_api->Play())
                  fprintf(stderr, "[StarfishBridge] Play() failed after LOADCOMPLETED\n");
                break;

              case PF_EVENT_TYPE_STR_STATE_UPDATE__UNLOADCOMPLETED:
                g_loaded = false;
                break;

              case PF_EVENT_TYPE_STR_STATE_UPDATE__PLAYING:
                g_paused = false;
                break;

              case PF_EVENT_TYPE_STR_STATE_UPDATE__PAUSED:
                g_paused = true;
                break;

              case PF_EVENT_TYPE_STR_STATE_UPDATE__ENDOFSTREAM:
                g_ended = true;
                break;

              case PF_EVENT_TYPE_INT_ERROR:
              case PF_EVENT_TYPE_STR_ERROR:
                g_lastError = strValue ? strValue : "(unknown Starfish error)";
                fprintf(stderr,
                        "[StarfishBridge] pipeline error type=%d numValue=%lld strValue=%s\n",
                        type, static_cast<long long>(numValue), strValue ? strValue : "(null)");
                break;

              default:
                break;
            }
            g_cv.notify_all();
          },
          nullptr))
  {
    SetErrorLocked("StarfishMediaAPIs::Load() returned false");
    return false;
  }

  return true;
}

enum starfish_feed_result FeedLocked(const uint8_t* data, int size, int64_t ptsNs, int esData,
                                     std::atomic<int64_t>& fedPtsNs)
{
  if (!g_sessionActive || !g_api || !g_loaded)
    return STARFISH_FEED_RETRY;

  if (ptsNs < 0)
    return STARFISH_FEED_DROP;

  if (g_started)
  {
    const int64_t fed = fedPtsNs.load();
    if (fed >= 0 && fed - g_currentPtsNs.load() > kMaxFeedAheadNs)
      return STARFISH_FEED_RETRY;
  }

  char addrBuf[2 + sizeof(uintptr_t) * 2 + 1];
  std::snprintf(addrBuf, sizeof(addrBuf), "%#" PRIxPTR, reinterpret_cast<uintptr_t>(data));

  nlohmann::json j;
  j["bufferAddr"] = addrBuf;
  j["bufferSize"] = size;
  j["pts"] = ptsNs;
  j["esData"] = esData;

  const std::string result = g_api->Feed(j.dump().c_str());

  if (result.find("Ok") != std::string::npos)
  {
    fedPtsNs = ptsNs;
    return STARFISH_FEED_OK;
  }
  if (result.find("BufferFull") != std::string::npos)
    return STARFISH_FEED_RETRY;

  fprintf(stderr, "[StarfishBridge] Feed() unexpected reply, dropping unit: %s\n", result.c_str());
  return STARFISH_FEED_DROP;
}

void NotifyResetLocked()
{
  if (!g_sessionActive || !g_api || g_flushInFlight)
    return;
  g_flushInFlight = true;

  if (!g_api->flush())
    fprintf(stderr, "[StarfishBridge] flush() returned false\n");

  g_started = false;
  g_fedVideoPtsNs = -1;
  g_fedAudioPtsNs = -1;
  g_needsTimeToDecode = true; // vd_webos_starfish's next fed unit should prime

  g_flushInFlight = false;
}

} // namespace

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

void starfish_bridge_set_wakeup_fn(starfish_wakeup_fn fn)
{
  std::lock_guard lock(g_filterMutex);
  g_wakeupFn = fn;
}

void starfish_bridge_begin_session(const char* app_id, const char* window_id)
{
  std::lock_guard lock(g_mutex);

  g_appId = app_id ? app_id : "";
  g_windowId = window_id ? window_id : "";

  g_hasVideo = g_hasAudio = g_audioWaitDone = g_loadTriggered = false;
  g_videoInfo = VideoInfo{};
  g_audioInfo = AudioInfo{};

  g_loaded = g_paused = g_started = g_ended = false;
  g_currentPtsNs = 0;
  g_fedVideoPtsNs = -1;
  g_fedAudioPtsNs = -1;

  g_flushInFlight = false;
  g_needsTimeToDecode = false; // the fresh Load()'s ptsToDecode=0 covers the first frame, not this

  g_lastError.clear();

  g_api = std::make_unique<StarfishMediaAPIs>();
  g_sessionActive = true;

  {
    std::lock_guard flock(g_filterMutex);
    g_videoWakeupInFlight = 0; // defensive; see PacerLoop -- should already be 0 by this point
    g_audioWakeupInFlight = 0;
  }

  g_pacerRunning = true;
  g_pacerThread = std::thread(PacerLoop);
}

void starfish_bridge_end_session(void)
{
  std::unique_lock lock(g_mutex);
  if (!g_sessionActive)
    return;
  g_sessionActive = false;

  g_pacerRunning = false;
  lock.unlock();
  if (g_pacerThread.joinable())
    g_pacerThread.join();
  lock.lock();

  // By the time the app calls this, mpv should already be fully stopped, so
  // both decoders' destroy() has already run and unregistered their filter
  // pointers -- see the ordering note on register_*_filter in the header.
  if (g_api && g_loaded)
  {
    if (!g_api->Unload())
      fprintf(stderr, "[StarfishBridge] Unload() returned false\n");
    g_cv.wait_for(lock, kUnloadWait, [] { return !g_loaded; });
  }

  // Same residual, undocumented-API-limitation risk as the LOADCOMPLETED
  // null check above: if Load() is still in flight (never got
  // LOADCOMPLETED), there's no way to cancel it, and a very late callback
  // could still fire after this reset. See that comment for the full
  // reasoning; this isn't a new problem introduced here.
  g_api.reset();

  std::lock_guard flock(g_filterMutex);
  g_videoFilter = nullptr;
  g_audioFilter = nullptr;
}

void starfish_bridge_set_paused(bool paused)
{
  std::lock_guard lock(g_mutex);
  if (!g_sessionActive || !g_api || !g_loaded)
    return;

  if (paused)
  {
    if (!g_api->Pause())
      fprintf(stderr, "[StarfishBridge] Pause() failed\n");
  }
  else
  {
    if (!g_api->Play())
      fprintf(stderr, "[StarfishBridge] Play() failed\n");
  }
  // g_paused itself flips from the PLAYING/PAUSED callback, once Starfish
  // confirms the state actually changed, rather than being set optimistically here.
}

bool starfish_bridge_is_loaded(void)
{
  std::lock_guard lock(g_mutex);
  return g_loaded;
}

bool starfish_bridge_has_ended(void)
{
  std::lock_guard lock(g_mutex);
  return g_ended;
}

int64_t starfish_bridge_get_position_ns(void)
{
  return g_currentPtsNs.load();
}

const char* starfish_bridge_get_last_error(void)
{
  std::lock_guard lock(g_mutex);
  return g_lastError.c_str();
}

bool starfish_bridge_video_codec_supported(const char* codec)
{
  return NameInSet(codec, {"h264", "hevc", "vp8", "vp9", "av1"});
}

bool starfish_bridge_audio_codec_supported(const char* codec)
{
  return NameInSet(codec, {"ac3", "eac3", "ac4", "dts", "opus", "mp3", "aac", "aac_latm"});
}

void starfish_bridge_register_video_filter(void* filter_handle)
{
  std::lock_guard lock(g_filterMutex);
  g_videoFilter = filter_handle;
}

void starfish_bridge_unregister_video_filter(void* filter_handle)
{
  std::unique_lock lock(g_filterMutex);
  if (g_videoFilter != filter_handle)
    return;
  g_videoFilter = nullptr; // no new wakeup calls will target this filter
  // Let any wakeup call already in flight for this filter finish before
  // returning -- the caller (vd_webos_starfish_destroy) is about to let mpv
  // free this filter, and mp_filter_wakeup() must not run against freed
  // memory. Bounded by however long mp_filter_wakeup() itself takes; see
  // the g_filterMutex comment above for why this wait doesn't reintroduce
  // the deadlock this replaced.
  g_filterCv.wait(lock, [] { return g_videoWakeupInFlight == 0; });
}

void starfish_bridge_register_audio_filter(void* filter_handle)
{
  std::lock_guard lock(g_filterMutex);
  g_audioFilter = filter_handle;
}

void starfish_bridge_unregister_audio_filter(void* filter_handle)
{
  std::unique_lock lock(g_filterMutex);
  if (g_audioFilter != filter_handle)
    return;
  g_audioFilter = nullptr;
  g_filterCv.wait(lock, [] { return g_audioWakeupInFlight == 0; });
}

void starfish_bridge_register_video(int av_codec_id, int width, int height, double fps,
                                    bool fps_reliable)
{
  std::unique_lock lock(g_mutex);
  g_videoInfo.codecId = static_cast<AVCodecID>(av_codec_id);
  g_videoInfo.width = width;
  g_videoInfo.height = height;
  g_videoInfo.fps = fps;
  g_videoInfo.fpsReliable = fps_reliable;
  g_hasVideo = true;
  g_cv.notify_all();
  TryBeginLoadLocked(lock);
}

void starfish_bridge_register_audio(int av_codec_id, int channels, int sample_rate,
                                    const uint8_t* extradata, int extradata_size, int profile)
{
  std::unique_lock lock(g_mutex);
  g_audioInfo.codecId = static_cast<AVCodecID>(av_codec_id);
  g_audioInfo.channels = channels;
  g_audioInfo.sampleRate = sample_rate;
  g_audioInfo.profile = profile;
  g_audioInfo.extradata.assign(extradata, extradata + (extradata_size > 0 ? extradata_size : 0));
  g_hasAudio = true;
  g_cv.notify_all();
  TryBeginLoadLocked(lock); // in case video already registered and was waiting on us
}

enum starfish_feed_result starfish_bridge_feed_video(const uint8_t* data, int size, int64_t pts_ns)
{
  std::lock_guard lock(g_mutex);
  return FeedLocked(data, size, pts_ns, /*esData=*/1, g_fedVideoPtsNs);
}

enum starfish_feed_result starfish_bridge_feed_audio(const uint8_t* data, int size, int64_t pts_ns)
{
  std::lock_guard lock(g_mutex);
  return FeedLocked(data, size, pts_ns, /*esData=*/2, g_fedAudioPtsNs);
}

void starfish_bridge_notify_video_reset(void)
{
  std::lock_guard lock(g_mutex);
  NotifyResetLocked();
}

void starfish_bridge_notify_audio_reset(void)
{
  std::lock_guard lock(g_mutex);
  NotifyResetLocked();
}

void starfish_bridge_prime_after_seek(int64_t landed_pts_ns)
{
  std::lock_guard lock(g_mutex);
  if (!g_sessionActive || !g_api || !g_needsTimeToDecode)
    return;
  g_needsTimeToDecode = false;

  nlohmann::json timePayload;
  timePayload["position"] = landed_pts_ns;
  if (!g_api->setTimeToDecode(timePayload.dump().c_str()))
    fprintf(stderr, "[StarfishBridge] setTimeToDecode() failed after seek\n");

  auto* player = static_cast<mediapipeline::CustomPlayer*>(g_api->player.get());
  if (player)
  {
    auto pipeline = player->getPipeline();
    if (pipeline)
      static_cast<mediapipeline::CustomPipeline*>(pipeline.get())->sendSegmentEvent();
  }
}
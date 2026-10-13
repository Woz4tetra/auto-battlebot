// H.264 through GStreamer. nvv4l2h264enc is the Jetson's hardware encoder, which FFmpeg cannot
// reach on stock L4T. x264enc is the software encoder for the dashboard preview on a Jetson with no
// NVENC (the Orin Nano), where NVIDIA's FFmpeg build has no libx264. Built only when pkg-config
// finds gstreamer-app-1.0 (AUTO_BATTLEBOT_HAVE_GSTREAMER).
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include "rgbd_camera/h264_backend.hpp"

namespace auto_battlebot {
namespace {
constexpr const char *kNvv4l2Factory = "nvv4l2h264enc";
constexpr const char *kX264Factory = "x264enc";
/** Frames appsrc may hold. Past it encode() waits up to kStallTimeout for the pipeline, then
 *  drops: VideoEncoder's own queue absorbs bursts, and a push into a stalled pipeline must not
 *  hang stop(). The pipeline holds 1920x1200 at 60 fps on the ZED Box, so this is a stall guard. */
constexpr guint64 kMaxQueuedFrames = 2;
constexpr auto kStallTimeout = std::chrono::milliseconds(1000);
constexpr GstClockTime kFinishTimeout = 2 * GST_SECOND;

bool ensure_gst_initialized() {
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, [] {
        GError *error = nullptr;
        ok = gst_init_check(nullptr, nullptr, &error);
        if (!ok) {
            spdlog::error("[VideoEncoder] GStreamer failed to initialize: {}",
                          error != nullptr ? error->message : "unknown error");
            g_clear_error(&error);
        }
    });
    return ok;
}

GstElement *make_element(const char *factory, const char *name) {
    GstElement *element = gst_element_factory_make(factory, name);
    if (element == nullptr) {
        spdlog::warn("[VideoEncoder] GStreamer element {} is not installed", factory);
    }
    return element;
}

/** The appsrc format. For nvvidconv, BGR is converted to BGRA in encode(): it has no 3-channel
 *  input. videoconvert takes BGR as is. */
const char *gst_format_name(VideoInputFormat format, bool hardware) {
    switch (format) {
        case VideoInputFormat::UYVY:
            return "UYVY";
        case VideoInputFormat::BGRA:
            return "BGRx";
        case VideoInputFormat::BGR:
            return hardware ? "BGRx" : "BGR";
    }
    return "BGRx";
}

int bytes_per_pixel(const char *format) {
    const std::string name = format;
    if (name == "UYVY") return 2;
    if (name == "BGR") return 3;
    return 4;
}

bool gst_element_installed(const char *factory_name) {
    if (!ensure_gst_initialized()) {
        return false;
    }
    GstElementFactory *factory = gst_element_factory_find(factory_name);
    if (factory == nullptr) {
        return false;
    }
    gst_object_unref(factory);
    return true;
}

class GstH264Backend : public H264Backend {
   public:
    /** `factory` is kNvv4l2Factory (NVMM through nvvidconv) or kX264Factory (I420 through
     *  videoconvert). */
    GstH264Backend(const char *factory, PacketSink sink)
        : name_(factory), hardware_(name_ == kNvv4l2Factory), sink_(std::move(sink)) {}
    ~GstH264Backend() override { teardown(); }

    bool open(const VideoEncoderOptions &options) {
        options_ = options;
        pipeline_ = gst_pipeline_new("video_encoder");
        GstElement *source = make_element("appsrc", "source");
        GstElement *convert = make_element(hardware_ ? "nvvidconv" : "videoconvert", "convert");
        GstElement *encoder = make_element(name_.c_str(), "encoder");
        GstElement *parse = make_element("h264parse", "parse");
        GstElement *sink = make_element("appsink", "sink");
        if (pipeline_ == nullptr || source == nullptr || convert == nullptr || encoder == nullptr ||
            parse == nullptr || sink == nullptr) {
            for (GstElement *element : {source, convert, encoder, parse, sink}) {
                if (element != nullptr) gst_object_unref(element);
            }
            return false;
        }
        appsrc_ = GST_APP_SRC(source);
        // The pipeline owns the elements from here; teardown() releases them with it.
        gst_bin_add_many(GST_BIN(pipeline_), source, convert, encoder, parse, sink, nullptr);
        encoder_ = encoder;
        // The keyframe request fires from a probe on the encoder's input as the requested frame
        // arrives, so it lands on that frame however many frames appsrc held ahead of it.
        // nvv4l2h264enc ignores the force-key-unit event and takes its force-IDR action instead;
        // x264enc takes the event.
        GstPad *encoder_input = gst_element_get_static_pad(encoder, "sink");
        gst_pad_add_probe(encoder_input, GST_PAD_PROBE_TYPE_BUFFER,
                          &GstH264Backend::on_encoder_input, this, nullptr);
        gst_object_unref(encoder_input);

        const char *format = gst_format_name(options_.input_format, hardware_);
        GstCaps *raw =
            gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, format, "width", G_TYPE_INT,
                                options_.width, "height", G_TYPE_INT, options_.height, "framerate",
                                GST_TYPE_FRACTION, options_.fps, 1, nullptr);
        // Not blocking: encode() runs on VideoEncoder's thread, and a push that waits on a stalled
        // pipeline would hang stop(). encode() drops instead, and VideoEncoder counts it.
        g_object_set(source, "caps", raw, "is-live", TRUE, "format", GST_FORMAT_TIME, "block",
                     FALSE, "do-timestamp", FALSE, nullptr);
        gst_caps_unref(raw);

        if (hardware_) {
            g_object_set(encoder, "bitrate", static_cast<guint>(options_.bitrate), "iframeinterval",
                         static_cast<guint>(options_.keyframe_interval), "idrinterval",
                         static_cast<guint>(options_.keyframe_interval),
                         // SPS/PPS in band ahead of every IDR: CompressedVideo needs it, and it is
                         // what makes a mid-file seek decode.
                         "insert-sps-pps", TRUE, "num-B-Frames", 0u, "maxperf-enable", TRUE,
                         nullptr);
        } else {
            // The preview shares the CPU with perception: the cheapest preset, on one core.
            gst_util_set_object_arg(G_OBJECT(encoder), "tune", "zerolatency");
            gst_util_set_object_arg(G_OBJECT(encoder), "speed-preset",
                                    options_.software_preview ? "ultrafast" : "veryfast");
            g_object_set(encoder, "bitrate", static_cast<guint>(options_.bitrate / 1000),
                         "key-int-max", static_cast<guint>(options_.keyframe_interval), "bframes",
                         0u, "threads", options_.software_preview ? 1u : 0u, "byte-stream", TRUE,
                         nullptr);
            // x264enc writes SPS/PPS once; h264parse repeats them ahead of every IDR.
            g_object_set(parse, "config-interval", -1, nullptr);
        }

        g_object_set(sink, "sync", FALSE, "max-buffers", 0u, "drop", FALSE, nullptr);
        GstAppSinkCallbacks callbacks{};
        callbacks.new_sample = &GstH264Backend::on_new_sample;
        gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);

        GstCaps *encoder_input_caps = gst_caps_from_string(
            hardware_ ? "video/x-raw(memory:NVMM),format=NV12" : "video/x-raw,format=I420");
        GstCaps *annex_b =
            gst_caps_from_string("video/x-h264,stream-format=byte-stream,alignment=au");
        const bool linked = gst_element_link(source, convert) &&
                            gst_element_link_filtered(convert, encoder, encoder_input_caps) &&
                            gst_element_link(encoder, parse) &&
                            gst_element_link_filtered(parse, sink, annex_b);
        gst_caps_unref(encoder_input_caps);
        gst_caps_unref(annex_b);
        if (!linked) {
            spdlog::warn("[VideoEncoder] Could not link the {} pipeline for {} {}x{}", name_,
                         format, options_.width, options_.height);
            return false;
        }

        // Messages are only read in finish(). Anything else would pile up on the bus for hours.
        GstBus *bus = gst_element_get_bus(pipeline_);
        gst_bus_set_sync_handler(bus, &GstH264Backend::on_bus_message, nullptr, nullptr);
        gst_object_unref(bus);

        // nvv4l2h264enc opens /dev/v4l2-nvenc on the way to READY, so a box without the encoder
        // block fails here, before the first frame. Never wait on get_state: a live appsrc with
        // no frames yet does not preroll, so the wait always ran to its timeout, and that blocked
        // the runner (recording toggle) or the publisher (preview subscribe) for 2 s each time.
        if (gst_element_set_state(pipeline_, GST_STATE_READY) == GST_STATE_CHANGE_FAILURE ||
            gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            spdlog::warn("[VideoEncoder] The {} pipeline did not start", name_);
            return false;
        }
        frame_bytes_ =
            static_cast<guint64>(options_.width) * options_.height * bytes_per_pixel(format);
        return true;
    }

    bool encode(cv::Mat image, int64_t pts, bool force_keyframe) override {
        if (appsrc_ == nullptr || image.empty()) {
            return false;
        }
        const auto deadline = std::chrono::steady_clock::now() + kStallTimeout;
        while (gst_app_src_get_current_level_bytes(appsrc_) >= kMaxQueuedFrames * frame_bytes_) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (hardware_ && options_.input_format == VideoInputFormat::BGR) {
            cv::Mat bgra;
            cv::cvtColor(image, bgra, cv::COLOR_BGR2BGRA);
            image = std::move(bgra);
        }
        if (!image.isContinuous()) {
            image = image.clone();
        }

        const GstClockTime timestamp =
            gst_util_uint64_scale(static_cast<guint64>(pts), GST_SECOND, options_.fps);
        if (force_keyframe) {
            std::lock_guard<std::mutex> lock(force_mutex_);
            force_idr_at_.insert(timestamp);
        }

        // The buffer borrows the Mat's pixels and frees the Mat when the encoder is done with it,
        // so the frame VideoEncoder already copied once is not copied again.
        auto *owner = new cv::Mat(std::move(image));
        const gsize size = owner->total() * owner->elemSize();
        GstBuffer *buffer =
            gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, owner->data, size, 0, size, owner,
                                        [](gpointer mat) { delete static_cast<cv::Mat *>(mat); });
        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, options_.fps);
        return gst_app_src_push_buffer(appsrc_, buffer) == GST_FLOW_OK;
    }

    void finish() override {
        if (pipeline_ == nullptr) {
            return;
        }
        gst_app_src_end_of_stream(appsrc_);
        GstBus *bus = gst_element_get_bus(pipeline_);
        GstMessage *message = gst_bus_timed_pop_filtered(
            bus, kFinishTimeout, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
        if (message == nullptr) {
            spdlog::warn("[VideoEncoder] {} did not drain within {} s; the last frames are lost",
                         name_, kFinishTimeout / GST_SECOND);
        } else {
            gst_message_unref(message);
        }
        gst_object_unref(bus);
        teardown();
    }

    const std::string &name() const override { return name_; }

   private:
    static GstPadProbeReturn on_encoder_input(GstPad *pad, GstPadProbeInfo *info,
                                              gpointer user_data) {
        auto *self = static_cast<GstH264Backend *>(user_data);
        const GstClockTime timestamp = GST_BUFFER_PTS(GST_PAD_PROBE_INFO_BUFFER(info));
        bool force = false;
        {
            std::lock_guard<std::mutex> lock(self->force_mutex_);
            force = self->force_idr_at_.erase(timestamp) > 0;
        }
        if (force && self->hardware_) {
            g_signal_emit_by_name(self->encoder_, "force-IDR");
        } else if (force) {
            // gst_video_event_new_downstream_force_key_unit() without linking gstreamer-video:
            // the structure is the event. No running time means the next frame, which is this one.
            GstStructure *request = gst_structure_new(
                "GstForceKeyUnit", "timestamp", G_TYPE_UINT64, GST_CLOCK_TIME_NONE, "stream-time",
                G_TYPE_UINT64, GST_CLOCK_TIME_NONE, "running-time", G_TYPE_UINT64,
                GST_CLOCK_TIME_NONE, "all-headers", G_TYPE_BOOLEAN, TRUE, "count", G_TYPE_UINT, 0u,
                nullptr);
            gst_pad_send_event(pad, gst_event_new_custom(GST_EVENT_CUSTOM_DOWNSTREAM, request));
        }
        return GST_PAD_PROBE_OK;
    }

    static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data) {
        auto *self = static_cast<GstH264Backend *>(user_data);
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (sample == nullptr) {
            return GST_FLOW_EOS;
        }
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (buffer != nullptr && gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            // Running time, not the raw PTS: x264enc (a GstVideoEncoder) shifts its output PTS
            // and segment by 1000 hours to keep DTS positive, and VideoEncoder drops a packet
            // whose ordinal matches no frame it queued. nvv4l2h264enc does not shift, and there
            // the two are equal.
            const GstClockTime running_time = gst_segment_to_running_time(
                gst_sample_get_segment(sample), GST_FORMAT_TIME, GST_BUFFER_PTS(buffer));
            const auto pts = static_cast<int64_t>(gst_util_uint64_scale_round(
                running_time, static_cast<guint64>(self->options_.fps), GST_SECOND));
            self->sink_(reinterpret_cast<const std::byte *>(map.data), map.size, pts,
                        !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT));
            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    static GstBusSyncReply on_bus_message(GstBus *, GstMessage *message, gpointer) {
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR:
            case GST_MESSAGE_WARNING: {
                GError *error = nullptr;
                gchar *debug = nullptr;
                const bool is_error = GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR;
                if (is_error) {
                    gst_message_parse_error(message, &error, &debug);
                } else {
                    gst_message_parse_warning(message, &error, &debug);
                }
                const std::string source = GST_OBJECT_NAME(GST_MESSAGE_SRC(message));
                if (is_error) {
                    spdlog::error("[VideoEncoder] {}: {} ({})", source, error->message,
                                  debug != nullptr ? debug : "");
                } else {
                    spdlog::warn("[VideoEncoder] {}: {}", source, error->message);
                }
                g_clear_error(&error);
                g_free(debug);
                return is_error ? GST_BUS_PASS : GST_BUS_DROP;
            }
            case GST_MESSAGE_EOS:
                return GST_BUS_PASS;
            default:
                return GST_BUS_DROP;
        }
    }

    void teardown() {
        if (pipeline_ != nullptr) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_ = nullptr;
            encoder_ = nullptr;
        }
    }

    const std::string name_;
    const bool hardware_;
    PacketSink sink_;
    VideoEncoderOptions options_;
    GstElement *pipeline_ = nullptr;
    GstAppSrc *appsrc_ = nullptr;
    /** Owned by pipeline_. */
    GstElement *encoder_ = nullptr;
    std::mutex force_mutex_;
    /** Buffer timestamps whose frame must come out as an IDR. */
    std::set<GstClockTime> force_idr_at_;
    guint64 frame_bytes_ = 0;
};

std::unique_ptr<H264Backend> open_gst(const char *factory, const VideoEncoderOptions &options,
                                      H264Backend::PacketSink sink) {
    if (!gst_element_installed(factory)) {
        return nullptr;
    }
    auto backend = std::make_unique<GstH264Backend>(factory, std::move(sink));
    if (!backend->open(options)) {
        return nullptr;
    }
    return backend;
}
}  // namespace

bool gst_nvv4l2_h264_installed() { return gst_element_installed(kNvv4l2Factory); }

bool gst_x264_installed() { return gst_element_installed(kX264Factory); }

std::unique_ptr<H264Backend> open_gst_nvv4l2_h264(const VideoEncoderOptions &options,
                                                  H264Backend::PacketSink sink) {
    return open_gst(kNvv4l2Factory, options, std::move(sink));
}

std::unique_ptr<H264Backend> open_gst_x264(const VideoEncoderOptions &options,
                                           H264Backend::PacketSink sink) {
    return open_gst(kX264Factory, options, std::move(sink));
}
}  // namespace auto_battlebot

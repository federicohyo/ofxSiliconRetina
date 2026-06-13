#pragma once
/// @file dvs_yolo_pipeline.hpp
/// @brief Self-contained YOLO object-detection pipeline for DVS event cameras.
///
/// Owns its OnnxRunner, builds VTEI input tensors, runs inference, decodes
/// detections, applies NMS and temporal smoothing, and draws overlays.

#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>

#include "ofMain.h"
#include "onnx_run.hpp"
#include "dvs_nn_utils.hpp"

// Forward-declare the polarity struct used in ofxDVS.hpp
struct polarity;

namespace dvs {

/// Detection in sensor coordinates.
struct YoloDet { ofRectangle box; float score; int cls; };

/// Runtime-tunable YOLO configuration.
struct YoloConfig {
    float conf_thresh   = 0.63f;    ///< Display threshold (eval uses 0.01 for mAP)
    float iou_thresh    = 0.7f;    ///< NMS IoU threshold (matches eval protocol)
    int   smooth_frames = 2;      ///< Temporal smoothing history length (1..5)
    bool  draw          = true;   ///< Draw overlay when true
    bool  show_labels   = true;
    float vtei_win_ms   = 50.0f;  ///< VTEI accumulation window in milliseconds
    int   encoding = 1;  ///< 0=VTEI (5ch), 1=temporal_bins (5 ternary bins)
    int   num_classes   = 2;
    bool  normalized_coords = false; ///< Model outputs coords in [0,1] (scale by model dims)
    std::vector<std::string> class_names = {"person"};
};

/// Complete YOLO pipeline: model loading, VTEI tensor building, inference,
/// decoding, temporal smoothing, and drawing.
class YoloPipeline {
public:
    YoloPipeline() = default;

    /// Load the ONNX model.  Call once during setup().
    /// @param path      Absolute path to the .onnx file.
    /// @param threads   Intra-op thread count (0 = use OnnxRunner default).
    void loadModel(const std::string& path, int threads = 0);

    bool isLoaded() const { return nn_ && nn_->isLoaded(); }

    /// Build the 5-channel input tensor from events.
    /// When cfg.encoding == 0 (VTEI): pos_count, neg_count, time_surface, ch3, ch4.
    /// When cfg.encoding == 1 (temporal_bins): 5 ternary bins {-1,0,+1}.
    /// Returns CHW float buffer of size 5 * sensorH * sensorW.
    const std::vector<float>& buildVTEI(
        const std::vector<polarity>& events,
        float** surfaceMapLastTs,
        const ofPixels& intensityPixels,
        int sensorW, int sensorH);

    /// Run inference on a pre-built VTEI tensor.  Performs letterbox, ONNX run,
    /// output decoding, NMS, un-letterbox, and temporal smoothing.
    /// Stores results internally; retrieve with detections().
    void infer(const std::vector<float>& vtei_sensor_chw,
               int sensorW, int sensorH);

    /// Draw bounding-box overlays in sensor coordinates.
    /// Caller should have set up the chip->screen transform.
    void drawDetections(int sensorW, int sensorH) const;

    /// Clear temporal smoothing history.
    void clearHistory();

    /// Access current detections (sensor coordinates).
    const std::vector<YoloDet>& detections() const { return dets_; }
    std::vector<YoloDet>& detections() { return dets_; }

    /// Last built VTEI/temporal-bins sensor-resolution tensor [5 * H * W].
    const std::vector<float>& lastVteiSensor() const { return chw5_sensor_; }
    int lastVteiW() const { return lastVteiW_; }
    int lastVteiH() const { return lastVteiH_; }

    /// Register 3 intermediate layer probes for activation visualization.
    /// Call once after loadModel(). Silently skips on failure.
    void setupProbes();

    /// Channel-averaged activation maps from the last inference.
    static constexpr int NUM_PROBES = 12;
    const std::array<ProbeMap, NUM_PROBES>& probeResults() const { return probeResults_; }
    bool probesEnabled() const { return probesEnabled_; }

    /// Mutable config for GUI binding.
    YoloConfig cfg;

private:
    // Temporal smoothing helper
    std::vector<YoloDet> temporalSmooth_(const std::vector<YoloDet>& cur);

    std::unique_ptr<OnnxRunner> nn_;

    // Model dimensions (filled on load)
    int model_H_ = 0, model_W_ = 0;

    // Cached letterbox params (sensor -> model), recomputed when sensor size changes
    float lb_scale_ = 1.f;
    int   lb_padx_  = 0, lb_pady_ = 0;
    int   cached_sW_ = 0, cached_sH_ = 0;

    void ensureLetterboxParams_(int sensorW, int sensorH);

    /// Build 5 ternary temporal bins from events (encoding=1).
    const std::vector<float>& buildTemporalBins_(
        const std::vector<polarity>& events, int sW, int sH);

    // ConvLSTM stateful inference (auto-detected from ONNX model)
    bool stateful_ = false;
    int  state_size_ = 0;
    std::vector<float> lstm_state_;

    // Pre-allocated buffers (avoid per-frame allocation)
    std::vector<float> pos_buf_, neg_buf_, T_buf_, E_buf_;
    std::vector<float> chw5_sensor_, chw5_model_;

    // Last sensor dimensions (for external VTEI visualization)
    int lastVteiW_ = 0, lastVteiH_ = 0;

    // Reusable output map for runCHW_into
    std::unordered_map<std::string, std::vector<float>> outmap_;

    // Temporal smoothing state
    std::deque<std::vector<YoloDet>> smooth_hist_;

    // Current detections
    std::vector<YoloDet> dets_;

    // VTEI window in microseconds (derived from cfg.vtei_win_ms)
    long vtei_win_us() const { return static_cast<long>(cfg.vtei_win_ms * 1000.f); }

    // Intermediate layer probes (channel-average activation maps)
    static const char* const PROBE_NAMES[NUM_PROBES];
    static const char* const PROBE_LABELS[NUM_PROBES];
    std::array<ProbeMap, NUM_PROBES> probeResults_;
    bool probesEnabled_ = false;
    void extractProbes_();
};

} // namespace dvs

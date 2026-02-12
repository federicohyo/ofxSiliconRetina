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
    float conf_thresh   = 0.8f;
    float iou_thresh    = 0.45f;
    int   smooth_frames = 2;      ///< Temporal smoothing history length (1..5)
    bool  draw          = true;   ///< Draw overlay when true
    bool  show_labels   = true;
    float vtei_win_ms   = 50.0f;  ///< VTEI accumulation window in milliseconds
    int   num_classes   = 1;
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

    /// Build the 5-channel VTEI tensor (pos, neg, time-surface, edge, intensity)
    /// from the current event packet and image generator state.
    /// Single-pass over packetsPolarity to find latest_ts and accumulate counts.
    /// Returns CHW float buffer of size 5 * sensorH * sensorW.
    std::vector<float> buildVTEI(
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

    // Pre-allocated buffers (avoid per-frame allocation)
    std::vector<float> pos_buf_, neg_buf_, T_buf_, E_buf_;
    std::vector<float> chw5_sensor_, chw5_model_;

    // Reusable output map for runCHW_into
    std::unordered_map<std::string, std::vector<float>> outmap_;

    // Temporal smoothing state
    std::deque<std::vector<YoloDet>> smooth_hist_;

    // Current detections
    std::vector<YoloDet> dets_;

    // VTEI window in microseconds (derived from cfg.vtei_win_ms)
    long vtei_win_us() const { return static_cast<long>(cfg.vtei_win_ms * 1000.f); }
};

} // namespace dvs

#pragma once
/// @file dvs_snn_yolo_pipeline.hpp
/// @brief SNN-YOLO pipeline (twl_spike_yolo / yolo8n_gen1) for DVS cameras.
///
/// Processes 2-channel (neg/pos) binary event bins, one 8-ms step per call,
/// using a stateful ONNX model exported from the Norse-based twl_spike_yolo repo.
/// Decodes SSD-style anchor boxes into YoloDet sensor-coordinate results.

#include <vector>
#include <string>
#include <memory>
#include <array>
#include <deque>

#include "ofMain.h"
#include "onnx_run.hpp"
#include "dvs_nn_utils.hpp"
#include "dvs_yolo_pipeline.hpp"  // for YoloDet

struct polarity;

namespace dvs {

struct SnnYoloConfig {
    float conf_thresh   = 0.30f;
    float iou_thresh    = 0.45f;
    float bin_ms        = 8.0f;    ///< Accumulation window per inference step
    int   smooth_frames = 2;
    bool  draw          = true;
    bool  show_labels   = true;
    int   num_classes   = 2;       ///< Background excluded; classes: car, pedestrian
    std::vector<std::string> class_names = {"car", "pedestrian"};
};

class SnnYoloPipeline {
public:
    SnnYoloPipeline() = default;

    /// Load ONNX model + companion anchor .npy file.
    /// anchors_path may be empty → derived by replacing ".onnx" with "_anchors.npy".
    void loadModel(const std::string& onnx_path,
                   const std::string& anchors_path = "");

    bool isLoaded() const { return nn_ && nn_->isLoaded(); }

    /// Feed events into the accumulation buffer.
    void pushEvents(const std::vector<polarity>& events, int sW, int sH);

    /// Run one inference step if the accumulation window is full.
    /// Returns true if inference was actually run.
    bool maybeInfer(int sW, int sH);

    /// Draw bounding-box overlays (call inside viewer draw).
    void drawDetections(int sW, int sH) const;

    /// Clear state and history.
    void clearHistory();

    const std::vector<YoloDet>& detections() const { return dets_; }

    SnnYoloConfig cfg;

private:
    // ── ONNX ───────────────────────────────────────────────────────────────
    std::unique_ptr<OnnxRunner> nn_;

    // ── Model dimensions ───────────────────────────────────────────────────
    static constexpr int MODEL_H = 256, MODEL_W = 320, MODEL_C = 2;

    // ── Anchors [N × 4], normalized (x1,y1,x2,y2) ─────────────────────────
    std::vector<float> anchors_;   // flat [N*4]
    int num_anchors_ = 0;

    // ── SNN membrane state ─────────────────────────────────────────────────
    std::vector<float> state_;
    int state_size_ = 0;

    // ── Letterbox (sensor → model) ─────────────────────────────────────────
    float lb_scale_ = 1.f;
    int   lb_padx_  = 0, lb_pady_ = 0;
    int   cached_sW_ = -1, cached_sH_ = -1;
    void  ensureLetterbox_(int sW, int sH);

    // ── Frame accumulation ─────────────────────────────────────────────────
    std::vector<float> frame_model_;  // CHW: [2, 256, 320]
    long  win_start_ts_ = -1;
    long  win_end_ts_   = -1;

    // ── Temporal smoothing (same as YoloPipeline) ──────────────────────────
    std::deque<std::vector<YoloDet>> smooth_hist_;
    std::vector<YoloDet> temporalSmooth_(const std::vector<YoloDet>& cur);

    // ── Current detections (sensor coords) ────────────────────────────────
    std::vector<YoloDet> dets_;

    // ── Reuse output map ───────────────────────────────────────────────────
    std::map<std::string, std::vector<float>> outmap_;

    // ── Decoding helpers ───────────────────────────────────────────────────
    static void offsetInverse_(float ax1, float ay1, float ax2, float ay2,
                                float dx, float dy, float dw, float dh,
                                float& ox1, float& oy1, float& ox2, float& oy2);

    void decodeAndStore_(const std::vector<float>& cls_raw,
                         const std::vector<float>& bbox_raw,
                         int sW, int sH);

    bool loadAnchors_(const std::string& path);
};

} // namespace dvs

#pragma once
/// @file dvs_tsdt_pipeline.hpp
/// @brief Self-contained TSDT (temporal-spatial-detection-transformer) gesture
///        recognition pipeline for DVS event cameras.
///
/// Owns its OnnxRunner, maintains a rolling event history, builds the
/// T x 2 x H x W input tensor, runs inference with EMA smoothing, and draws
/// the predicted gesture label.

#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <map>

#include "ofMain.h"
#include "onnx_run.hpp"

// Forward-declare the polarity struct
struct polarity;

namespace dvs {

/// Lightweight event struct for the TSDT event history.
struct TsEvent { int x, y; bool p; long ts; };

/// Runtime-tunable TSDT configuration.
struct TsdtConfig {
    int   T          = 8;        ///< Number of temporal bins
    int   inH        = 128;      ///< Model input height
    int   inW        = 128;      ///< Model input width
    int   bin_ms     = 10;       ///< Per-timestep bin width in ms
    int   ev_per_bin = 10000;    ///< Events per temporal bin (matches training)
    float ema_alpha  = 1.0f;     ///< EMA smoothing for logits (1.0 = no smoothing)
    bool  show_label = true;     ///< Draw overlay text

    // Time-based binning (for TPDVSGesture and similar models)
    bool  time_based_binning = false; ///< true = bin by time window, false = by event count
    float bin_window_ms      = 75.0f; ///< Time window per bin in ms (when time_based_binning)
    float conf_threshold     = 0.0f;  ///< Min confidence to display label
    float display_timeout    = 2.0f;  ///< Seconds to keep showing last prediction
    float label_y_offset     = 0.f;   ///< Vertical offset to avoid label overlap with TSDT

    std::string log_tag      = "TSDT"; ///< Log prefix to distinguish pipeline instances

    std::vector<std::string> labels = {
        "hand_clapping", "right_hand_wave", "left_hand_wave",
        "right_hand_clockwise", "right_hand_counter_clockwise",
        "left_hand_clockwise", "left_hand_counter_clockwise",
        "forearm_roll", "drums", "guitar", "random_other_gestures"
    };
};

/// Complete TSDT pipeline: event history, tensor building, inference,
/// EMA smoothing, and label drawing.
class TsdtPipeline {
public:
    TsdtPipeline() = default;

    /// Load the ONNX model.  Call once during setup().
    /// @param path      Absolute path to the .onnx file.
    /// @param threads   Intra-op thread count (0 = use OnnxRunner default).
    void loadModel(const std::string& path, int threads = 0);

    bool isLoaded() const { return tsdt_ && tsdt_->isLoaded(); }

    /// Append valid events from the current polarity packet to the rolling history.
    void pushEvents(const std::vector<polarity>& events, int sensorW, int sensorH);

    /// Build the T x 2 x inH x inW tensor from the event history.
    /// Uses the single consolidated builder (letterbox-fixed variant).
    /// Returns empty vector if not enough events yet.
    std::vector<float> buildTensor(int sensorW, int sensorH);

    /// Run inference.  Returns (class_index, confidence) or (-1, 0) on failure.
    /// Also consumes the used events from the history.
    std::pair<int, float> infer(int sensorW, int sensorH);

    /// Draw the predicted gesture label at bottom-center of the window.
    void drawLabel() const;

    /// Run a self-test with synthetic data (prints results to log).
    void selfTest();

    /// Run inference on a saved tensor file for comparison with Python.
    void debugFromFile(const std::string& binPath);

    /// Clear event history and prediction state.
    void clearHistory();

    /// Access last prediction.
    int   lastIndex()      const { return last_idx_; }
    float lastConfidence() const { return last_conf_; }

    /// Mutable config for GUI binding.
    TsdtConfig cfg;

    /// Direct access to the underlying OnnxRunner (for self-test / debug).
    OnnxRunner* runner() { return tsdt_.get(); }

private:
    std::unique_ptr<OnnxRunner> tsdt_;

    // Rolling event history
    std::deque<TsEvent> hist_;

    // Prediction state
    int   last_idx_  = -1;
    float last_conf_ = 0.f;
    float last_predict_time_ = 0.f;

    // Pre-allocated buffers
    std::vector<float> tsdt_tensor_;
    std::vector<float> ema_logits_;
    std::vector<float> softmax_exps_;

    // Cached letterbox params
    float lb_scale_ = 1.f;
    int   lb_padx_  = 0, lb_pady_ = 0;
    int   cached_sW_ = 0, cached_sH_ = 0;

    void ensureLetterboxParams_(int sensorW, int sensorH);
};

} // namespace dvs

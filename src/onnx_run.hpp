#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <thread>

#include "ofMain.h"

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>


/**
 * Minimal API for running an ONNX model from openFrameworks.
 * - Loads an ONNX model once.
 * - Accepts an ofImage or pre-built CHW float data.
 * - Handles RGB (3ch) or Gray (1ch) inputs, float32 and float16 models.
 * - Returns all model outputs (float) by output name.
 */
class OnnxRunner {
public:
    /// Run with already-prepared CHW float data (allocates result map).
    std::unordered_map<std::string, std::vector<float>>
    runCHW(const std::vector<float>& chw, int C, int H, int W);

    /// Run with CHW float data, reusing the output map to avoid per-call allocation.
    void runCHW_into(const std::vector<float>& chw, int C, int H, int W,
                     std::unordered_map<std::string, std::vector<float>>& results);

    struct IOInfo {
        std::string name;
        ONNXTensorElementDataType type;
        std::vector<int64_t> dims; // e.g., {1, C, H, W} with dynamic dims possibly -1
    };

    struct Config {
        std::string model_path;
        int intra_op_num_threads = std::max(1u, std::thread::hardware_concurrency() / 2);
        bool use_cuda = false;        // requires CUDA build of onnxruntime
        bool verbose = false;
        // Preproc
        bool normalize_01 = true;     // scale to [0,1]
    };

    explicit OnnxRunner(const Config& cfg);
    ~OnnxRunner();

    // Load / unload
    void load();
    bool isLoaded() const { return loaded_; }

    // Introspect model I/O
    const std::vector<IOInfo>& inputs()  const { return inputs_;  }
    const std::vector<IOInfo>& outputs() const { return outputs_; }

    // Run inference on an ofImage. If the model size differs, the image is resized.
    std::unordered_map<std::string, std::vector<float>> run(const ofImage& img);

    std::pair<int,int> getInputHW() const;

    // Run with raw float data and arbitrary shape (e.g. for TSDT [1,T,2,H,W]).
    std::map<std::string, std::vector<float>>
    runRaw(const float* data, const std::vector<int64_t>& shape);

    /// Run with multiple named float inputs and arbitrary shapes.
    /// Each entry: (data pointer, shape).  Inputs matched by order to model input list.
    std::map<std::string, std::vector<float>>
    runRawMulti(const std::vector<std::pair<const float*, std::vector<int64_t>>>& inputs);


private:
    void queryModelIO_();
    void buildNameCaches_();
    void dumpModelIO_() const;

    std::vector<float> pixelsToCHWFloat_(const ofPixels& px, int64_t C, int64_t H, int64_t W) const;
    static std::string dataTypeName_(ONNXTensorElementDataType t);

    /// Shared implementation for runCHW / runCHW_into.
    void runCHW_impl_(const std::vector<float>& chw, int C, int H, int W,
                      std::unordered_map<std::string, std::vector<float>>& results);

    Config cfg_;
    bool loaded_ = false;

    // ORT objects
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    Ort::MemoryInfo mem_info_;  // cached, created once in constructor

    // Cached IO info
    std::vector<IOInfo> inputs_;
    std::vector<IOInfo> outputs_;

    // Cached names (std::string ownership + const char* views)
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> in_names_c_;
    std::vector<const char*> out_names_c_;

    // Pre-allocated FP16 conversion buffer (reused across calls)
    mutable std::vector<uint16_t> chw_f16_buf_;
};

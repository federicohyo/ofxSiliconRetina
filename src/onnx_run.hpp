#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <stdexcept>
#include <sstream>          // <-- add this

#include "ofMain.h"

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>


/**
 * Minimal, header-only API for running an ONNX model from openFrameworks.
 * - Loads an ONNX model once.
 * - Accepts an ofImage and does CHW float32 normalization [0,1].
 * - Handles RGB (3ch) or Gray (1ch) inputs.
 * - Returns all model outputs (float) by output name.
 */
class OnnxRunner {
public:
    // new helper: run with already-prepared CHW float data
    std::unordered_map<std::string, std::vector<float>>
    runCHW(const std::vector<float>& chw, int C, int H, int W);

    struct IOInfo {
        std::string name;
        ONNXTensorElementDataType type;
        std::vector<int64_t> dims; // e.g., {1, C, H, W} with dynamic dims possibly -1
    };

    struct Config {
        std::string model_path;
        int intra_op_num_threads = 1;
        bool use_cuda = false;        // requires CUDA build of onnxruntime
        bool verbose = false;
        // Preproc
        bool normalize_01 = true;     // scale to [0,1]
        // If your model expects mean/std, you can extend with vectors here.
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
    // Returns a map: output_name -> flat float buffer (NCHW flattened, usually N=1).
    std::unordered_map<std::string, std::vector<float>> run(const ofImage& img);

    std::pair<int,int> getInputHW() const;

    //spikevision
    std::map<std::string, std::vector<float>>
    runRaw(const float* data, const std::vector<int64_t>& shape);


private:
    // Helpers
    void queryModelIO_();

    void dumpModelIO_() const;   // <-- declare helper

    std::vector<float> ofImageToCHWFloat_(const ofImage& src, int64_t C, int64_t H, int64_t W) const;
    static std::string dataTypeName_(ONNXTensorElementDataType t);

    Config cfg_;
    bool loaded_ = false;

    // ORT objects (ordered per ORT best practices)
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_; // used for names and metadata

    // Cached IO info
    std::vector<IOInfo> inputs_;
    std::vector<IOInfo> outputs_;

    // Cached input/output names (char* owned by allocator_ → convert to std::string)
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
};

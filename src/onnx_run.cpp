#include "onnx_run.hpp"
#include <algorithm>
#include <cstring>
#include <sstream>
#include <numeric>
#include <cstdint>

// ---- float32 <-> float16 helpers (IEEE-754 binary16) ----
static inline uint16_t f32_to_f16_bits(float f) {
    uint32_t x; std::memcpy(&x, &f, sizeof(x));
    uint32_t sign = (x >> 31) & 0x1;
    int32_t  exp  = int32_t((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = x & 0x7FFFFF;

    if (exp <= 0) {
        if (exp < -10) return uint16_t(sign << 15);
        mant = (mant | 0x800000) >> (1 - exp);
        return uint16_t((sign << 15) | (mant + 0x1000) >> 13);
    } else if (exp >= 31) {
        return uint16_t((sign << 15) | (0x1F << 10) | (mant ? 0x200 : 0));
    } else {
        return uint16_t((sign << 15) | (exp << 10) | ((mant + 0x1000) >> 13));
    }
}

static inline float f16_bits_to_f32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp  = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    uint32_t x;

    if (exp == 0) {
        if (mant == 0) { x = sign << 31; }
        else {
            exp = 127 - 15 + 1;
            while ((mant & 0x400) == 0) { mant <<= 1; --exp; }
            mant &= 0x3FF;
            x = (sign << 31) | (exp << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        x = (sign << 31) | (0xFF << 23) | (mant ? 0x7FFFFF : 0);
    } else {
        x = (sign << 31) | ((exp - 15 + 127) << 23) | (mant << 13);
    }
    float f; std::memcpy(&f, &x, sizeof(f)); return f;
}

// ---- Constructor ----
OnnxRunner::OnnxRunner(const Config& cfg)
    : cfg_(cfg),
      env_(ORT_LOGGING_LEVEL_WARNING, "ofxDVS_onnx"),
      session_options_(),
      mem_info_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault))
{
    session_options_.SetIntraOpNumThreads(cfg_.intra_op_num_threads);
    if (cfg_.verbose) {
        session_options_.SetLogSeverityLevel(0);
    }

#if defined(ORT_API_VERSION) && ORT_API_VERSION >= 8
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
#endif
}

OnnxRunner::~OnnxRunner() {
    session_.reset();
    loaded_ = false;
}

// ---- Load ----
void OnnxRunner::load() {
    if (cfg_.model_path.empty())
        throw std::runtime_error("OnnxRunner: model_path is empty.");

    session_ = std::make_unique<Ort::Session>(env_, cfg_.model_path.c_str(), session_options_);
    loaded_ = true;

    queryModelIO_();
    buildNameCaches_();
    dumpModelIO_();

    if (inputs_.empty())  throw std::runtime_error("OnnxRunner: model has no inputs.");
    if (outputs_.empty()) throw std::runtime_error("OnnxRunner: model has no outputs.");
}

void OnnxRunner::buildNameCaches_() {
    in_names_c_.clear();
    in_names_c_.reserve(input_names_.size());
    for (auto& s : input_names_) in_names_c_.push_back(s.c_str());

    out_names_c_.clear();
    out_names_c_.reserve(output_names_.size());
    for (auto& s : output_names_) out_names_c_.push_back(s.c_str());
}

void OnnxRunner::dumpModelIO_() const {
    ofLogNotice() << "[NN] inputs=" << session_->GetInputCount()
                  << " outputs=" << session_->GetOutputCount();

    for (size_t i = 0; i < inputs_.size(); ++i) {
        std::ostringstream d; d << "[";
        for (size_t k = 0; k < inputs_[i].dims.size(); ++k) {
            if (k) d << ",";
            d << inputs_[i].dims[k];
        }
        d << "]";
        ofLogNotice() << "[NN] in" << i
                      << " name=" << inputs_[i].name
                      << " type=" << dataTypeName_(inputs_[i].type)
                      << " dims=" << d.str();
    }

    for (size_t i = 0; i < outputs_.size(); ++i) {
        std::ostringstream d; d << "[";
        for (size_t k = 0; k < outputs_[i].dims.size(); ++k) {
            if (k) d << ",";
            d << outputs_[i].dims[k];
        }
        d << "]";
        ofLogNotice() << "[NN] out" << i
                      << " name=" << outputs_[i].name
                      << " type=" << dataTypeName_(outputs_[i].type)
                      << " dims=" << d.str();
    }
}

std::pair<int,int> OnnxRunner::getInputHW() const {
    if (inputs_.empty()) return { -1, -1 };
    const auto& in0 = inputs_.front();
    int H = -1, W = -1;
    if (in0.dims.size() >= 4) {
        H = (in0.dims[2] > 0 ? (int)in0.dims[2] : -1);
        W = (in0.dims[3] > 0 ? (int)in0.dims[3] : -1);
    }
    return { H, W };
}

// ---- queryModelIO_ ----
void OnnxRunner::queryModelIO_() {
    input_names_.clear();
    output_names_.clear();
    inputs_.clear();
    outputs_.clear();

    size_t num_inputs  = session_->GetInputCount();
    size_t num_outputs = session_->GetOutputCount();

    for (size_t i = 0; i < num_inputs; ++i) {
        IOInfo info;
    #if defined(ORT_API_VERSION) && ORT_API_VERSION >= 12
        auto name = session_->GetInputNameAllocated(i, allocator_);
        info.name = name.get();
    #else
        char* name_c = session_->GetInputName(i, allocator_);
        info.name = name_c ? name_c : "";
        if (name_c) allocator_.Free(name_c);
    #endif
        Ort::TypeInfo tinfo = session_->GetInputTypeInfo(i);
        auto tensor_info = tinfo.GetTensorTypeAndShapeInfo();
        info.type = tensor_info.GetElementType();
        info.dims = tensor_info.GetShape();
        input_names_.push_back(info.name);
        inputs_.push_back(std::move(info));
    }

    for (size_t i = 0; i < num_outputs; ++i) {
        IOInfo info;
    #if defined(ORT_API_VERSION) && ORT_API_VERSION >= 12
        auto name = session_->GetOutputNameAllocated(i, allocator_);
        info.name = name.get();
    #else
        char* name_c = session_->GetOutputName(i, allocator_);
        info.name = name_c ? name_c : "";
        if (name_c) allocator_.Free(name_c);
    #endif
        Ort::TypeInfo tinfo = session_->GetOutputTypeInfo(i);
        auto tensor_info = tinfo.GetTensorTypeAndShapeInfo();
        info.type = tensor_info.GetElementType();
        info.dims = tensor_info.GetShape();
        output_names_.push_back(info.name);
        outputs_.push_back(std::move(info));
    }
}

// ---- runRaw (for TSDT and other arbitrary-shape models) ----
std::map<std::string, std::vector<float>>
OnnxRunner::runRaw(const float* data, const std::vector<int64_t>& shape)
{
    using namespace Ort;

    size_t numel = std::accumulate(
        shape.begin(), shape.end(), (size_t)1, std::multiplies<size_t>());

    Value input = Value::CreateTensor<float>(
        mem_info_,
        const_cast<float*>(data),
        numel,
        shape.data(),
        (size_t)shape.size()
    );

    RunOptions opts;
    auto outputs = session_->Run(
        opts,
        in_names_c_.data(), &input, 1,
        out_names_c_.data(), out_names_c_.size()
    );

    std::map<std::string, std::vector<float>> outmap;
    for (size_t i = 0; i < outputs.size(); ++i) {
        auto& v = outputs[i];
        auto info = v.GetTensorTypeAndShapeInfo();
        size_t n = info.GetElementCount();
        float* ptr = v.GetTensorMutableData<float>();
        std::vector<float> buf(ptr, ptr + n);

        const std::string& key = (i < output_names_.size()) ? output_names_[i] : ("output"+std::to_string(i));
        outmap[key] = std::move(buf);
    }
    return outmap;
}

// ---- runCHW (allocating) ----
std::unordered_map<std::string, std::vector<float>>
OnnxRunner::runCHW(const std::vector<float>& chw, int C, int H, int W)
{
    std::unordered_map<std::string, std::vector<float>> results;
    runCHW_impl_(chw, C, H, W, results);
    return results;
}

// ---- runCHW_into (reusing output map) ----
void OnnxRunner::runCHW_into(const std::vector<float>& chw, int C, int H, int W,
                              std::unordered_map<std::string, std::vector<float>>& results)
{
    runCHW_impl_(chw, C, H, W, results);
}

// ---- runCHW shared implementation ----
void OnnxRunner::runCHW_impl_(const std::vector<float>& chw, int C, int H, int W,
                               std::unordered_map<std::string, std::vector<float>>& results)
{
    if (!loaded_) throw std::runtime_error("OnnxRunner::runCHW before load().");
    if (inputs_.empty()) throw std::runtime_error("OnnxRunner: no inputs.");

    if (C <= 0 || H <= 0 || W <= 0)
        throw std::runtime_error("OnnxRunner::runCHW: invalid C/H/W.");

    const size_t need = static_cast<size_t>(C) * static_cast<size_t>(H) * static_cast<size_t>(W);
    if (chw.size() != need) {
        std::ostringstream oss;
        oss << "OnnxRunner::runCHW: data size " << chw.size()
            << " != C*H*W " << need << " (C=" << C << ",H=" << H << ",W=" << W << ")";
        throw std::runtime_error(oss.str());
    }

    const auto& in0 = inputs_.front();
    std::vector<int64_t> shape = {1, (int64_t)C, (int64_t)H, (int64_t)W};

    Ort::Value input_tensor{nullptr};

    if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        input_tensor = Ort::Value::CreateTensor<float>(
            mem_info_,
            const_cast<float*>(chw.data()),
            need,
            shape.data(), shape.size());
    } else if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        // Reuse pre-allocated FP16 buffer
        chw_f16_buf_.resize(need);
        for (size_t i = 0; i < need; ++i)
            chw_f16_buf_[i] = f32_to_f16_bits(chw[i]);
        input_tensor = Ort::Value::CreateTensor<Ort::Float16_t>(
            mem_info_,
            reinterpret_cast<Ort::Float16_t*>(chw_f16_buf_.data()),
            need,
            shape.data(), shape.size());
    } else {
        std::ostringstream oss;
        oss << "OnnxRunner: unsupported input type " << dataTypeName_(in0.type);
        throw std::runtime_error(oss.str());
    }

    // Use cached name arrays
    auto outs = session_->Run(Ort::RunOptions{nullptr},
                              in_names_c_.data(), &input_tensor, 1,
                              out_names_c_.data(), out_names_c_.size());

    // Collect outputs (reuse vectors if already in results map)
    for (size_t i = 0; i < outs.size(); ++i) {
        auto& v = outs[i];
        if (!v.IsTensor()) throw std::runtime_error("OnnxRunner: non-tensor output.");

        auto tinf = v.GetTensorTypeAndShapeInfo();
        const auto et = tinf.GetElementType();
        const size_t count = tinf.GetElementCount();
        const std::string& key = output_names_[i];

        // Reuse existing vector in map if possible
        auto& dst = results[key];

        if (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            const float* p = v.GetTensorData<float>();
            dst.assign(p, p + count);
        } else if (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
            const uint16_t* q = reinterpret_cast<const uint16_t*>(v.GetTensorData<Ort::Float16_t>());
            dst.resize(count);
            for (size_t k = 0; k < count; ++k) dst[k] = f16_bits_to_f32(q[k]);
        } else {
            std::ostringstream oss;
            oss << "OnnxRunner: unsupported output type " << dataTypeName_(et);
            throw std::runtime_error(oss.str());
        }
    }
}


// ---- run (ofImage) ----
std::unordered_map<std::string, std::vector<float>>
OnnxRunner::run(const ofImage& img)
{
    if (!loaded_)
        throw std::runtime_error("OnnxRunner::run() called before load().");
    if (inputs_.empty())
        throw std::runtime_error("OnnxRunner: model has no inputs.");

    const auto& in0 = inputs_.front();

    if (in0.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
        in0.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        std::ostringstream oss;
        oss << "OnnxRunner: first input must be float/float16. Got " << dataTypeName_(in0.type);
        throw std::runtime_error(oss.str());
    }

    int64_t N = 1, C = 3, H = img.getHeight(), W = img.getWidth();
    if (in0.dims.size() == 4) {
        N = (in0.dims[0] > 0 ? in0.dims[0] : 1);
        C = (in0.dims[1] > 0 ? in0.dims[1] : (img.getPixels().getNumChannels() == 1 ? 1 : 3));
        H = (in0.dims[2] > 0 ? in0.dims[2] : img.getHeight());
        W = (in0.dims[3] > 0 ? in0.dims[3] : img.getWidth());
    } else if (in0.dims.size() == 3) {
        C = (in0.dims[0] > 0 ? in0.dims[0] : (img.getPixels().getNumChannels() == 1 ? 1 : 3));
        H = (in0.dims[1] > 0 ? in0.dims[1] : img.getHeight());
        W = (in0.dims[2] > 0 ? in0.dims[2] : img.getWidth());
    } else {
        throw std::runtime_error("OnnxRunner: unsupported input rank (need 3 or 4 dims).");
    }

    // Work directly with pixels, no ofImage copy
    std::vector<float> chw = pixelsToCHWFloat_(img.getPixels(), C, H, W);

    std::vector<int64_t> input_shape = {1, C, H, W};

    Ort::Value input_tensor{nullptr};

    if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        input_tensor = Ort::Value::CreateTensor<float>(
            mem_info_, chw.data(), chw.size(), input_shape.data(), input_shape.size());
    } else { // FLOAT16
        chw_f16_buf_.resize(chw.size());
        for (size_t i = 0; i < chw.size(); ++i) chw_f16_buf_[i] = f32_to_f16_bits(chw[i]);
        input_tensor = Ort::Value::CreateTensor<Ort::Float16_t>(
            mem_info_,
            reinterpret_cast<Ort::Float16_t*>(chw_f16_buf_.data()),
            chw_f16_buf_.size(),
            input_shape.data(),
            input_shape.size());
    }

    // Use cached name arrays
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        in_names_c_.data(), &input_tensor, 1,
        out_names_c_.data(), out_names_c_.size()
    );

    std::unordered_map<std::string, std::vector<float>> results;
    for (size_t i = 0; i < output_tensors.size(); ++i) {
        auto& val = output_tensors[i];
        if (!val.IsTensor())
            throw std::runtime_error("OnnxRunner: non-tensor output encountered.");

        auto tinf = val.GetTensorTypeAndShapeInfo();
        auto elem_type = tinf.GetElementType();
        size_t count = tinf.GetElementCount();

        if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            const float* data = val.GetTensorData<float>();
            results[output_names_[i]] = std::vector<float>(data, data + count);
        } else if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
            const uint16_t* d16 = reinterpret_cast<const uint16_t*>(val.GetTensorData<Ort::Float16_t>());
            std::vector<float> out(count);
            for (size_t k = 0; k < count; ++k) out[k] = f16_bits_to_f32(d16[k]);
            results[output_names_[i]] = std::move(out);
        } else {
            std::ostringstream oss;
            oss << "OnnxRunner: unsupported output type " << dataTypeName_(elem_type);
            throw std::runtime_error(oss.str());
        }
    }

    return results;
}


// ---- pixelsToCHWFloat_ (works with const ofPixels& directly, no ofImage copy) ----
std::vector<float> OnnxRunner::pixelsToCHWFloat_(const ofPixels& srcPx, int64_t C, int64_t H, int64_t W) const {
    // We may need to resize or convert channels - use a local ofPixels for that
    ofPixels px = srcPx; // lightweight copy of pixel data only (no texture)
    const int srcC = px.getNumChannels();

    // Channel conversion
    if (C == 1 && srcC != 1) {
        // Convert to grayscale manually
        ofPixels gray;
        gray.allocate(px.getWidth(), px.getHeight(), OF_IMAGE_GRAYSCALE);
        for (size_t y = 0; y < px.getHeight(); ++y) {
            for (size_t x = 0; x < px.getWidth(); ++x) {
                size_t idx = (y * px.getWidth() + x) * srcC;
                unsigned char r = px[idx + 0];
                unsigned char g = px[idx + 1];
                unsigned char b = px[idx + 2];
                gray[y * px.getWidth() + x] = static_cast<unsigned char>(0.299f*r + 0.587f*g + 0.114f*b);
            }
        }
        px = std::move(gray);
    } else if (C == 3 && srcC == 4) {
        // Drop alpha channel
        ofPixels noA;
        noA.allocate(px.getWidth(), px.getHeight(), OF_IMAGE_COLOR);
        for (size_t y = 0; y < px.getHeight(); ++y) {
            for (size_t x = 0; x < px.getWidth(); ++x) {
                size_t idx4 = (y * px.getWidth() + x) * 4;
                size_t idx3 = (y * px.getWidth() + x) * 3;
                noA[idx3 + 0] = px[idx4 + 0];
                noA[idx3 + 1] = px[idx4 + 1];
                noA[idx3 + 2] = px[idx4 + 2];
            }
        }
        px = std::move(noA);
    }

    // Resize if needed (ofPixels resize is lightweight - no texture upload)
    if ((int64_t)px.getWidth() != W || (int64_t)px.getHeight() != H) {
        px.resize(W, H);
    }

    const int chC = px.getNumChannels();
    if ((C != 1 && C != 3) || (chC != C)) {
        std::ostringstream oss;
        oss << "OnnxRunner: channel mismatch after preprocessing. srcC=" << chC << " wanted C=" << C;
        throw std::runtime_error(oss.str());
    }

    // Convert to float32 CHW normalized
    std::vector<float> out(static_cast<size_t>(C * H * W));
    auto toFloat = [&](unsigned char v) { return cfg_.normalize_01 ? (v / 255.0f) : static_cast<float>(v); };

    if (C == 1) {
        for (int64_t y = 0; y < H; ++y) {
            for (int64_t x = 0; x < W; ++x) {
                size_t idx = static_cast<size_t>(y * W + x);
                out[idx] = toFloat(px[idx]);
            }
        }
    } else { // C == 3
        for (int64_t y = 0; y < H; ++y) {
            for (int64_t x = 0; x < W; ++x) {
                size_t hw = static_cast<size_t>(y * W + x);
                size_t srcIdx = hw * 3;
                out[0 * (H * W) + hw] = toFloat(px[srcIdx + 0]);
                out[1 * (H * W) + hw] = toFloat(px[srcIdx + 1]);
                out[2 * (H * W) + hw] = toFloat(px[srcIdx + 2]);
            }
        }
    }

    return out;
}

std::string OnnxRunner::dataTypeName_(ONNXTensorElementDataType t) {
    switch (t) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:   return "float32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:   return "uint8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:    return "int8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:  return "uint16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:   return "int16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:   return "int32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:   return "int64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:  return "double";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:  return "uint32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:  return "uint64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:    return "bool";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16: return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:return "bfloat16";
        default: return "unknown";
    }
}

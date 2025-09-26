#include "onnx_run.hpp"
#include <algorithm>
#include <cstring>
#include <sstream>

#include <cstring>
#include <cstdint>

// ---- float32 <-> float16 helpers (IEEE-754 binary16) ----
static inline uint16_t f32_to_f16_bits(float f){
    uint32_t x; std::memcpy(&x, &f, sizeof(x));
    uint32_t sign = (x >> 31) & 0x1;
    int32_t  exp  = int32_t((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = x & 0x7FFFFF;

    if (exp <= 0) {
        if (exp < -10) return uint16_t(sign << 15); // underflow to zero
        mant = (mant | 0x800000) >> (1 - exp);
        // round to nearest
        return uint16_t((sign << 15) | (mant + 0x1000) >> 13);
    } else if (exp >= 31) {
        // Inf / NaN
        return uint16_t((sign << 15) | (0x1F << 10) | (mant ? 0x200 : 0));
    } else {
        // normal
        return uint16_t((sign << 15) | (exp << 10) | ((mant + 0x1000) >> 13));
    }
}

static inline float f16_bits_to_f32(uint16_t h){
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp  = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    uint32_t x;

    if (exp == 0) {
        if (mant == 0) { x = sign << 31; }
        else {
            // subnormal
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

void OnnxRunner::load() {
    if (cfg_.model_path.empty())
        throw std::runtime_error("OnnxRunner: model_path is empty.");

    session_ = std::make_unique<Ort::Session>(env_, cfg_.model_path.c_str(), session_options_);
    loaded_ = true;

    queryModelIO_();
    dumpModelIO_();   // <-- call here

    if (inputs_.empty())  throw std::runtime_error("OnnxRunner: model has no inputs.");
    if (outputs_.empty()) throw std::runtime_error("OnnxRunner: model has no outputs.");
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


// OnnxRunner.cpp
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


OnnxRunner::OnnxRunner(const Config& cfg)
: cfg_(cfg),
  env_(ORT_LOGGING_LEVEL_WARNING, "ofxDVS_onnx"),
  session_options_()
{
    session_options_.SetIntraOpNumThreads(cfg_.intra_op_num_threads);
    if (cfg_.verbose) {
        session_options_.SetLogSeverityLevel(0);
    }
#if defined(ORT_API_VERSION) && ORT_API_VERSION >= 8
    // Leave graph optimization at default; can be set to all if desired:
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
#endif
    // Note: CUDA EP requires onnxruntime built with CUDA and EP initialization.
    // Keeping CPU EP by default.
}

OnnxRunner::~OnnxRunner() {
    session_.reset();
    loaded_ = false;
}

std::unordered_map<std::string, std::vector<float>>
OnnxRunner::runCHW(const std::vector<float>& chw, int C, int H, int W)
{
    if (!loaded_) throw std::runtime_error("OnnxRunner::runCHW before load().");
    if (inputs_.empty()) throw std::runtime_error("OnnxRunner: no inputs.");

    // enforce sane dims
    if (C <= 0 || H <= 0 || W <= 0) {
        throw std::runtime_error("OnnxRunner::runCHW: invalid C/H/W.");
    }

    const size_t need = static_cast<size_t>(C) * static_cast<size_t>(H) * static_cast<size_t>(W);
    if (chw.size() != need) {
        std::ostringstream oss;
        oss << "OnnxRunner::runCHW: data size " << chw.size()
            << " != C*H*W " << need << " (C="<<C<<",H="<<H<<",W="<<W<<")";
        throw std::runtime_error(oss.str());
    }

    const auto& in0 = inputs_.front(); // we feed only first input
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);

    // Build input tensor with the EXACT shape we want: 1xC×H×W
    std::vector<int64_t> shape = {1, (int64_t)C, (int64_t)H, (int64_t)W};

    Ort::Value input_tensor{nullptr};
    std::vector<uint16_t> chw_f16; // backing store for fp16 if needed

    if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        input_tensor = Ort::Value::CreateTensor<float>(
            mem_info,
            const_cast<float*>(chw.data()), // ORT won’t modify
            need,
            shape.data(), shape.size());
    } else if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        chw_f16.resize(need);
        // pack FP32 -> FP16 bits
        for (size_t i=0;i<need;++i) {
            float f = chw[i];
            // your helper
            chw_f16[i] = f32_to_f16_bits(f);
        }
        input_tensor = Ort::Value::CreateTensor<Ort::Float16_t>(
            mem_info,
            reinterpret_cast<Ort::Float16_t*>(chw_f16.data()),
            need,
            shape.data(), shape.size());
    } else {
        std::ostringstream oss;
        oss << "OnnxRunner: unsupported input type " << dataTypeName_(in0.type);
        throw std::runtime_error(oss.str());
    }

    // name arrays
    std::vector<const char*> in_names_c;  in_names_c.reserve(input_names_.size());
    for (auto &s : input_names_) in_names_c.push_back(s.c_str());
    std::vector<const char*> out_names_c; out_names_c.reserve(output_names_.size());
    for (auto &s : output_names_) out_names_c.push_back(s.c_str());

    // run
    auto outs = session_->Run(Ort::RunOptions{nullptr},
                              in_names_c.data(), &input_tensor, 1,
                              out_names_c.data(), out_names_c.size());

    // collect outputs as float (support float & float16)
    std::unordered_map<std::string, std::vector<float>> results;
    for (size_t i=0;i<outs.size();++i) {
        auto& v = outs[i];
        if (!v.IsTensor()) throw std::runtime_error("OnnxRunner: non-tensor output.");

        auto tinf = v.GetTensorTypeAndShapeInfo();
        const auto et = tinf.GetElementType();
        const size_t count = tinf.GetElementCount();

        if (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            const float* p = v.GetTensorData<float>();
            results[output_names_[i]] = std::vector<float>(p, p + count);
        } else if (et == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
            const Ort::Float16_t* p = v.GetTensorData<Ort::Float16_t>();
            std::vector<float> out(count);
            const uint16_t* q = reinterpret_cast<const uint16_t*>(p);
            for (size_t k=0;k<count;++k) out[k] = f16_bits_to_f32(q[k]);
            results[output_names_[i]] = std::move(out);
        } else {
            std::ostringstream oss;
            oss << "OnnxRunner: unsupported output type " << dataTypeName_(et);
            throw std::runtime_error(oss.str());
        }
    }
    return results;
}


void OnnxRunner::queryModelIO_() {
    input_names_.clear();
    output_names_.clear();
    inputs_.clear();
    outputs_.clear();

    size_t num_inputs  = session_->GetInputCount();
    size_t num_outputs = session_->GetOutputCount();

    // Inputs
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

    // Outputs
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

// runner -> 
std::unordered_map<std::string, std::vector<float>>
OnnxRunner::run(const ofImage& img)
{
    if (!loaded_) {
        throw std::runtime_error("OnnxRunner::run() called before load().");
    }
    if (inputs_.empty()) {
        throw std::runtime_error("OnnxRunner: model has no inputs.");
    }

    const auto& in0 = inputs_.front();

    // Accept float32 or float16 model inputs
    if (in0.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
        in0.type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        std::ostringstream oss;
        oss << "OnnxRunner: first input must be float/float16. Got " << dataTypeName_(in0.type);
        throw std::runtime_error(oss.str());
    }

    // Infer NCHW from model dims (dynamic supported)
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

    // Preprocess to CHW float32 in [0,1] if normalize_01
    std::vector<float> chw = ofImageToCHWFloat_(img, C, H, W);

    // Always feed NCHW with N=1
    std::vector<int64_t> input_shape = {1, C, H, W};

    // Create input tensor (float32 or float16)
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor{nullptr};
    std::vector<uint16_t> chw_f16; // keep alive if we use FP16

    if (in0.type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, chw.data(), chw.size(), input_shape.data(), input_shape.size());
    } else { // FLOAT16
        chw_f16.resize(chw.size());
        for (size_t i = 0; i < chw.size(); ++i) chw_f16[i] = f32_to_f16_bits(chw[i]);
        input_tensor = Ort::Value::CreateTensor<Ort::Float16_t>(
            mem_info,
            reinterpret_cast<Ort::Float16_t*>(chw_f16.data()),
            chw_f16.size(),
            input_shape.data(),
            input_shape.size());
    }

    // Names
    std::vector<const char*> in_names_c;    in_names_c.reserve(input_names_.size());
    for (auto& s : input_names_)  in_names_c.push_back(s.c_str());
    std::vector<const char*> out_names_c;   out_names_c.reserve(output_names_.size());
    for (auto& s : output_names_) out_names_c.push_back(s.c_str());

    // Run
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        in_names_c.data(), &input_tensor, 1,
        out_names_c.data(), out_names_c.size()
    );

    // Extract outputs -> std::vector<float>
    std::unordered_map<std::string, std::vector<float>> results;
    for (size_t i = 0; i < output_tensors.size(); ++i) {
        auto& val = output_tensors[i];
        if (!val.IsTensor()) {
            throw std::runtime_error("OnnxRunner: non-tensor output encountered.");
        }

        auto tinf = val.GetTensorTypeAndShapeInfo();
        auto elem_type = tinf.GetElementType();
        size_t count = tinf.GetElementCount();

        if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            const float* data = val.GetTensorData<float>();
            results[output_names_[i]] = std::vector<float>(data, data + count);
        } else if (elem_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
            const Ort::Float16_t* data = val.GetTensorData<Ort::Float16_t>();
            std::vector<float> out(count);
            const uint16_t* d16 = reinterpret_cast<const uint16_t*>(data);
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


std::vector<float> OnnxRunner::ofImageToCHWFloat_(const ofImage& src, int64_t C, int64_t H, int64_t W) const {
    // Make a copy we can resize without modifying the original
    ofImage img = src;

    // Ensure pixel format matches desired channels
    if (C == 1 && img.getPixels().getNumChannels() != 1) {
        ofPixels gray;
        //ofxCvGrayscaleImage tmp; // optional if you have ofxOpenCv; otherwise do manual grayscale
        if (C == 1 && img.getPixels().getNumChannels() != 1) {
            ofImage grayImg;
            grayImg.allocate(img.getWidth(), img.getHeight(), OF_IMAGE_GRAYSCALE);
            const ofPixels& p = img.getPixels();
            ofPixels& g = grayImg.getPixels();
            for (int y = 0; y < img.getHeight(); ++y) {
                for (int x = 0; x < img.getWidth(); ++x) {
                    int idx = (y * img.getWidth() + x);
                    int idx3 = idx * p.getNumChannels();
                    unsigned char r = p[idx3 + 0];
                    unsigned char gch = p[idx3 + 1];
                    unsigned char b = p[idx3 + 2];
                    g[idx] = static_cast<unsigned char>(0.299f*r + 0.587f*gch + 0.114f*b);
                }
            }
            grayImg.update();
            img = std::move(grayImg);
        }
        // Drop alpha if present
        else if (C == 3 && img.getPixels().getNumChannels() == 4) {
            ofImage noA;
            noA.allocate(img.getWidth(), img.getHeight(), OF_IMAGE_COLOR);
            const ofPixels& p = img.getPixels();
            ofPixels& q = noA.getPixels();
            for (int y = 0; y < img.getHeight(); ++y) {
                for (int x = 0; x < img.getWidth(); ++x) {
                    int idx4 = (y*img.getWidth() + x)*4;
                    int idx3 = (y*img.getWidth() + x)*3;
                    q[idx3+0] = p[idx4+0];
                    q[idx3+1] = p[idx4+1];
                    q[idx3+2] = p[idx4+2];
                }
            }
            noA.update();
            img = std::move(noA);
        }

        // Resize if needed (no clone)
        if (img.getWidth() != W || img.getHeight() != H) {
            img.resize(W, H);
        }


        // To avoid OpenCV dependency, do manual grayscale:
        gray.allocate(img.getWidth(), img.getHeight(), OF_IMAGE_GRAYSCALE);
        const ofPixels& p = img.getPixels();
        for (int y = 0; y < img.getHeight(); ++y) {
            for (int x = 0; x < img.getWidth(); ++x) {
                auto r = p[(y*img.getWidth() + x)*p.getNumChannels() + 0];
                auto g = p[(y*img.getWidth() + x)*p.getNumChannels() + 1];
                auto b = p[(y*img.getWidth() + x)*p.getNumChannels() + 2];
                gray[y*img.getWidth() + x] = static_cast<unsigned char>((0.299f*r + 0.587f*g + 0.114f*b));
            }
        }
        img.setFromPixels(gray);
    } else if (C == 3 && img.getPixels().getNumChannels() == 4) {
        // Drop alpha if present
        ofImage noA;
        noA.allocate(img.getWidth(), img.getHeight(), OF_IMAGE_COLOR);
        const ofPixels& p = img.getPixels();
        ofPixels& q = noA.getPixels();
        q.allocate(img.getWidth(), img.getHeight(), OF_IMAGE_COLOR);
        for (int y = 0; y < img.getHeight(); ++y) {
            for (int x = 0; x < img.getWidth(); ++x) {
                int idx4 = (y*img.getWidth() + x)*4;
                int idx3 = (y*img.getWidth() + x)*3;
                q[idx3+0] = p[idx4+0];
                q[idx3+1] = p[idx4+1];
                q[idx3+2] = p[idx4+2];
            }
        }
        noA.update();
        img = std::move(noA);
    }

    // Resize if needed
    if (img.getWidth() != W || img.getHeight() != H) {
        img.resize(W, H);
    }

    const ofPixels& px = img.getPixels();
    const int srcC = px.getNumChannels();
    if ((C != 1 && C != 3) || (srcC != C)) {
        std::ostringstream oss;
        oss << "OnnxRunner: channel mismatch after preprocessing. srcC=" << srcC << " wanted C=" << C;
        throw std::runtime_error(oss.str());
    }

    // Convert to float32 CHW normalized
    std::vector<float> out(static_cast<size_t>(C * H * W));
    auto toFloat = [&](unsigned char v) { return cfg_.normalize_01 ? (v / 255.0f) : static_cast<float>(v); };

    if (C == 1) {
        // HWC -> CHW (single channel)
        for (int64_t y = 0; y < H; ++y) {
            for (int64_t x = 0; x < W; ++x) {
                size_t srcIdx = static_cast<size_t>(y * W + x);
                size_t dstIdx = static_cast<size_t>(y * W + x); // channel 0 plane
                out[dstIdx] = toFloat(px[srcIdx]);
            }
        }
    } else { // C == 3
        // HWC (RGB) -> CHW
        for (int64_t y = 0; y < H; ++y) {
            for (int64_t x = 0; x < W; ++x) {
                size_t hw = static_cast<size_t>(y * W + x);
                size_t srcIdx = hw * 3;
                out[0 * (H * W) + hw] = toFloat(px[srcIdx + 0]); // R
                out[1 * (H * W) + hw] = toFloat(px[srcIdx + 1]); // G
                out[2 * (H * W) + hw] = toFloat(px[srcIdx + 2]); // B
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

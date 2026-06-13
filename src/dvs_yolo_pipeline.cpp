#include "dvs_yolo_pipeline.hpp"
#include "ofxDVS.hpp" // for struct polarity

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace dvs {

const char* const YoloPipeline::PROBE_NAMES[NUM_PROBES] = {
    // Group 0: LSTM hidden state h_t (gate Concat: [h_{t-1}, x_t] → updated h_t)
    "/model/backbone/stage1_gate/Concat_output_0",  // [1,128,72,88]   stride-4
    "/model/backbone/stage2_gate/Concat_output_0",  // [1,256,36,44]   stride-8
    "/model/backbone/stage3_gate/Concat_output_0",  // [1,512,18,22]   stride-16
    "/model/backbone/stage4_gate/Concat_output_0",  // [1,1024, 9,11]  stride-32
    // Group 1: LSTM cell state c_t = f*c_prev + i*g — accumulated temporal memory
    "/model/backbone/stage1_gate/Add_1_output_0",   // [unk,64,72,88]
    "/model/backbone/stage2_gate/Add_1_output_0",   // [unk,128,36,44]
    "/model/backbone/stage3_gate/Add_1_output_0",   // [unk,256,18,22]
    "/model/backbone/stage4_gate/Add_1_output_0",   // [unk,512, 9,11]
    // Group 2: feedforward conv features (stem, SPPF, neck)
    "/model/backbone/stem2/act/Mul_output_0",              // [0, 64,72,88]
    "/model/backbone/sppf/cv2/act/Mul_output_0",           // [0,512, 9,11]
    "/model/neck/up_c2f_p3/cv1/act/Mul_output_0",          // [0,128,36,44]
    "/model/neck/down_c2f_p5/cv1/act/Mul_output_0",        // [0,512, 9,11]
};
const char* const YoloPipeline::PROBE_LABELS[NUM_PROBES] = {
    "P2",   "P3",   "P4",   "P5",      // LSTM hidden states h_t
    "c2",   "c3",   "c4",   "c5",      // LSTM cell states c_t
    "Stem", "SPPF", "NkP3", "NkP5",    // Conv features
};

// ---- loadModel ----
void YoloPipeline::loadModel(const std::string& path, int threads) {
    OnnxRunner::Config nncfg;
    nncfg.model_path = path;
    if (threads > 0) nncfg.intra_op_num_threads = threads;
    nncfg.normalize_01 = true;
    nncfg.verbose = false;

    nn_ = std::make_unique<OnnxRunner>(nncfg);
    nn_->load();

    auto hw = nn_->getInputHW();
    model_H_ = hw.first  > 0 ? hw.first  : 288;
    model_W_ = hw.second > 0 ? hw.second : 352;

    cached_sW_ = 0;
    cached_sH_ = 0;

    // Auto-detect stateful ConvLSTM model (has "state_in" input)
    stateful_ = false;
    state_size_ = 0;
    lstm_state_.clear();
    for (const auto& inp : nn_->inputs()) {
        if (inp.name == "state_in") {
            stateful_ = true;
            int64_t sz = 1;
            for (auto d : inp.dims) { if (d > 0) sz *= d; }
            state_size_ = (int)sz;
            lstm_state_.assign(state_size_, 0.f);
            ofLogNotice() << "[YoloPipeline] stateful ConvLSTM model detected"
                          << " (state_size=" << state_size_ << ")";
            break;
        }
    }

    ofLogNotice() << "[YoloPipeline] loaded " << path
                  << " model=" << model_W_ << "x" << model_H_
                  << (stateful_ ? " [stateful]" : " [stateless]");
}

// ---- ensureLetterboxParams_ ----
void YoloPipeline::ensureLetterboxParams_(int sW, int sH) {
    if (sW == cached_sW_ && sH == cached_sH_) return;
    nn::letterboxParams(sW, sH, model_W_, model_H_, lb_scale_, lb_padx_, lb_pady_);
    cached_sW_ = sW;
    cached_sH_ = sH;
}

// ---- buildTemporalBins_ ----
const std::vector<float>& YoloPipeline::buildTemporalBins_(
    const std::vector<polarity>& events, int sW, int sH)
{
    const int bins = 5;
    const size_t plane = (size_t)sW * sH;
    chw5_sensor_.assign(bins * plane, 0.f);

    // Find t_min, t_max across valid events
    int64_t t_min = INT64_MAX, t_max = INT64_MIN;
    for (const auto& e : events) {
        if (!e.valid) continue;
        if (e.timestamp < t_min) t_min = e.timestamp;
        if (e.timestamp > t_max) t_max = e.timestamp;
    }
    if (t_max <= t_min) {
        // All same timestamp or no events — put everything in last bin
        for (const auto& e : events) {
            if (!e.valid) continue;
            int x = (int)e.pos.x, y = (int)e.pos.y;
            if ((unsigned)x >= (unsigned)sW || (unsigned)y >= (unsigned)sH) continue;
            chw5_sensor_[(bins - 1) * plane + y * sW + x] = e.pol ? 1.f : -1.f;
        }
        return chw5_sensor_;
    }

    double dt = (double)(t_max - t_min);
    for (const auto& e : events) {
        if (!e.valid) continue;
        int x = (int)e.pos.x, y = (int)e.pos.y;
        if ((unsigned)x >= (unsigned)sW || (unsigned)y >= (unsigned)sH) continue;
        int bin = (int)((double)(e.timestamp - t_min) / dt * bins);
        if (bin >= bins) bin = bins - 1;
        chw5_sensor_[bin * plane + y * sW + x] = e.pol ? 1.f : -1.f;
    }
    lastVteiW_ = sW; lastVteiH_ = sH;
    return chw5_sensor_;
}

// ---- buildVTEI (single-pass) ----
const std::vector<float>& YoloPipeline::buildVTEI(
    const std::vector<polarity>& events,
    float** surfaceMapLastTs,
    const ofPixels& intensity,
    int sW, int sH)
{
    // Dispatch: temporal bins encoding bypasses VTEI entirely
    if (cfg.encoding == 1)
        return buildTemporalBins_(events, sW, sH);
    const size_t plane = (size_t)sW * sH;

    // Resize pre-allocated buffers once
    pos_buf_.assign(plane, 0.f);
    neg_buf_.assign(plane, 0.f);

    // Single pass: find latest_ts AND accumulate counts
    long latest_ts = 0;
    const long win_us = vtei_win_us();
    for (const auto& e : events) {
        if (!e.valid) continue;
        if (e.timestamp > latest_ts) latest_ts = e.timestamp;
    }
    for (const auto& e : events) {
        if (!e.valid) continue;
        if (e.timestamp + win_us >= latest_ts) {
            int x = (int)e.pos.x, y = (int)e.pos.y;
            if ((unsigned)x < (unsigned)sW && (unsigned)y < (unsigned)sH) {
                if (e.pol) pos_buf_[y * sW + x] += 1.f;
                else       neg_buf_[y * sW + x] += 1.f;
            }
        }
    }

    // Normalize counts
    const float count_scale = 5.0f;
    for (size_t i = 0; i < plane; ++i) {
        pos_buf_[i] = std::min(1.f, pos_buf_[i] / count_scale);
        neg_buf_[i] = std::min(1.f, neg_buf_[i] / count_scale);
    }

    // Time surface
    T_buf_.assign(plane, 0.f);
    if (surfaceMapLastTs) {
        const float tau_us = 5e5f;
        for (int y = 0; y < sH; ++y) {
            for (int x = 0; x < sW; ++x) {
                float last = surfaceMapLastTs[y][x];
                float dt   = std::max(0.f, (float)latest_ts - last);
                T_buf_[y * sW + x] = std::clamp(std::exp(-dt / tau_us), 0.f, 1.f);
            }
        }
    }

    // Channels 3-4: ternary temporal bins {-1, 0, +1} (matches training with fill_dead_channels=True)
    E_buf_.assign(plane, 0.f);   // ch3
    std::vector<float> I_buf(plane, 0.f);  // ch4

    {
        // Ternary temporal bins matching Python vtei_encode(fill_dead_channels=True):
        //   ch3 = early half of window (t < t_mid): last polarity per pixel
        //   ch4 = late half of window  (t >= t_mid): last polarity per pixel
        std::vector<long> early_last_ts(plane, -1);
        std::vector<long> late_last_ts(plane, -1);

        const long t_mid = latest_ts - win_us / 2;
        for (const auto& e : events) {
            if (!e.valid) continue;
            if (e.timestamp + win_us < latest_ts) continue;
            int x = (int)e.pos.x, y = (int)e.pos.y;
            if ((unsigned)x >= (unsigned)sW || (unsigned)y >= (unsigned)sH) continue;
            size_t idx = (size_t)y * sW + x;
            if (e.timestamp < t_mid) {
                if (e.timestamp >= early_last_ts[idx]) {
                    early_last_ts[idx] = e.timestamp;
                    E_buf_[idx] = e.pol ? 1.f : -1.f;
                }
            } else {
                if (e.timestamp >= late_last_ts[idx]) {
                    late_last_ts[idx] = e.timestamp;
                    I_buf[idx] = e.pol ? 1.f : -1.f;
                }
            }
        }
    }

    // Pack into CHW (C=5: pos, neg, T, ch3, ch4)
    const size_t C5 = 5;
    chw5_sensor_.resize(C5 * plane);

    for (int y = 0; y < sH; ++y) {
        for (int x = 0; x < sW; ++x) {
            size_t hw = (size_t)y * sW + x;
            chw5_sensor_[0 * plane + hw] = pos_buf_[hw];
            chw5_sensor_[1 * plane + hw] = neg_buf_[hw];
            chw5_sensor_[2 * plane + hw] = T_buf_[hw];
            chw5_sensor_[3 * plane + hw] = E_buf_[hw];
            chw5_sensor_[4 * plane + hw] = I_buf[hw];
        }
    }

    lastVteiW_ = sW; lastVteiH_ = sH;
    return chw5_sensor_;
}

// ---- infer ----
void YoloPipeline::infer(const std::vector<float>& vtei_sensor_chw,
                          int sensorW, int sensorH)
{
    if (!nn_ || !nn_->isLoaded()) { dets_.clear(); return; }

    ensureLetterboxParams_(sensorW, sensorH);

    const int C5 = 5;

    // Letterbox sensor -> model (reuse pre-allocated buffer)
    const size_t model_plane = (size_t)C5 * model_H_ * model_W_;
    chw5_model_.assign(model_plane, 0.0f);
    nn::letterboxCHW_into(
        vtei_sensor_chw.data(), C5, sensorH, sensorW,
        chw5_model_.data(), model_H_, model_W_,
        lb_scale_, lb_padx_, lb_pady_);

    if (chw5_model_.size() != (size_t)C5 * model_H_ * model_W_) {
        ofLogError() << "[YOLO] letterbox produced wrong size";
        dets_.clear();
        return;
    }

    // Run ONNX (stateful or stateless)
    if (stateful_) {
        std::vector<int64_t> frame_shape = {1, (int64_t)C5, (int64_t)model_H_, (int64_t)model_W_};
        std::vector<int64_t> state_shape = {1, (int64_t)state_size_};
        std::vector<std::pair<const float*, std::vector<int64_t>>> inputs = {
            {chw5_model_.data(), frame_shape},
            {lstm_state_.data(), state_shape}
        };
        auto raw_outmap = nn_->runRawMulti(inputs);
        // Copy results into outmap_ for downstream decode
        outmap_.clear();
        for (auto& kv : raw_outmap) outmap_[kv.first] = std::move(kv.second);
        // Update ConvLSTM state
        auto sit = outmap_.find("state_out");
        if (sit != outmap_.end()) lstm_state_ = sit->second;
    } else {
        nn_->runCHW_into(chw5_model_, C5, model_H_, model_W_, outmap_);
    }
    if (outmap_.empty()) { dets_.clear(); return; }

    // Find output
    const std::vector<float>* pv = nullptr;
    auto it0 = outmap_.find("output0");
    if (it0 != outmap_.end()) pv = &it0->second;
    else                      pv = &outmap_.begin()->second;
    const std::vector<float>& v = *pv;

    // Decode: out0 = [1, C, N]
    // ReYOLOv8m format: C=6 [cx_px, cy_px, w_px, h_px, conf(sigmoided), cls_idx(float)]
    // SNN_DET format:   C=4+nc [cx, cy, w, h, logit0, logit1, ...]
    const int C_reyolo = 6;
    const int nc = cfg.num_classes + 1;
    const int C_snn   = 4 + nc;
    // Auto-detect: stateful models are ReYOLOv8m; also check if C=6 divides evenly
    const bool reyolo_fmt = stateful_ || (v.size() % C_reyolo == 0 && C_reyolo != C_snn);
    const int C = reyolo_fmt ? C_reyolo : C_snn;
    if (v.size() % C != 0) {
        ofLogError() << "[YOLO] unexpected output length=" << v.size() << " not divisible by C=" << C;
        dets_.clear(); return;
    }
    const int N = (int)(v.size() / C);
    auto at = [&](int c, int i) -> float { return v[c * N + i]; };

    std::vector<nn::Det> raw_dets;
    raw_dets.reserve(128);
    for (int i = 0; i < N; ++i) {
        float cx = at(0,i), cy = at(1,i), w = at(2,i), h = at(3,i);

        if (cfg.normalized_coords) {
            cx *= model_W_;  cy *= model_H_;
            w  *= model_W_;  h  *= model_H_;
        }

        int   best_cls = -1;
        float best_p   = -1.f;
        if (reyolo_fmt) {
            // ReYOLOv8m: ch4=conf (already sigmoided), ch5=cls_idx (float)
            best_p   = at(4, i);
            best_cls = (int)std::round(at(5, i));
        } else {
            // SNN_DET: ch4..ch(4+nc-1) are raw logits, need sigmoid
            for (int c = 0; c < nc; ++c) {
                float p = nn::sigmoid(at(4 + c, i));
                if (p > best_p) { best_p = p; best_cls = c; }
            }
        }
        if (best_p < cfg.conf_thresh) continue;
        if (w <= 1.f || h <= 1.f) continue;
        // Reject tiny boxes (false positives from noise/far-away objects)
        if (w * h < 400.f) continue;  // min ~20x20 pixels in model coords
        float ar = w / std::max(1.f, h);
        if (ar < 0.15f || ar > 6.7f) continue;

        nn::Det d;
        d.x1 = cx - 0.5f*w; d.y1 = cy - 0.5f*h;
        d.x2 = cx + 0.5f*w; d.y2 = cy + 0.5f*h;
        d.score = best_p;
        d.cls   = best_cls;
        raw_dets.push_back(d);
    }

    // Extract intermediate layer activations (if probes registered)
    if (probesEnabled_) extractProbes_();

    // NMS
    auto kept = nn::nms(std::move(raw_dets), cfg.iou_thresh);

    // Un-letterbox to sensor coords
    std::vector<YoloDet> cur_sensor;
    cur_sensor.reserve(kept.size());
    for (auto& k : kept) {
        auto r = nn::unletterboxToSensor(
            k.x1, k.y1, k.x2, k.y2,
            lb_scale_, lb_padx_, lb_pady_,
            sensorW, sensorH);
        if (r.getWidth() > 0 && r.getHeight() > 0)
            cur_sensor.push_back(YoloDet{r, k.score, k.cls});
    }

    // Temporal smoothing
    dets_ = temporalSmooth_(cur_sensor);

    for (auto& d : dets_) {
        ofLogNotice() << "[YOLO] det cls=" << d.cls << " score=" << d.score
                      << " rect=" << d.box;
    }
}

// ---- temporalSmooth_ ----
std::vector<YoloDet> YoloPipeline::temporalSmooth_(const std::vector<YoloDet>& cur)
{
    const int   max_hist = cfg.smooth_frames;
    const float match_iou = 0.5f;
    const int   min_hits  = 2;
    const float min_w     = 12.f, min_h = 12.f;

    smooth_hist_.push_back(cur);
    if ((int)smooth_hist_.size() > max_hist) smooth_hist_.pop_front();

    std::vector<YoloDet> out;
    out.reserve(cur.size());

    for (const auto& d0 : cur) {
        if (d0.box.getWidth() < min_w || d0.box.getHeight() < min_h) continue;

        float sum_w = d0.score;
        float x1 = d0.box.getX() * d0.score;
        float y1 = d0.box.getY() * d0.score;
        float x2 = (d0.box.getX() + d0.box.getWidth())  * d0.score;
        float y2 = (d0.box.getY() + d0.box.getHeight()) * d0.score;
        int   hits = 1;

        for (int t = (int)smooth_hist_.size() - 2; t >= 0; --t) {
            const auto& prev = smooth_hist_[t];
            int best_j = -1; float best_iou = 0.f; float best_s = 0.f;
            ofRectangle best_r;
            for (int j = 0; j < (int)prev.size(); ++j) {
                if (prev[j].cls != d0.cls) continue;
                float iou = nn::rectIoU(d0.box, prev[j].box);
                if (iou > best_iou) {
                    best_iou = iou; best_j = j;
                    best_s = prev[j].score; best_r = prev[j].box;
                }
            }
            if (best_j >= 0 && best_iou >= match_iou) {
                if (best_r.getWidth() >= min_w && best_r.getHeight() >= min_h) {
                    x1 += best_r.getX() * best_s;
                    y1 += best_r.getY() * best_s;
                    x2 += (best_r.getX() + best_r.getWidth())  * best_s;
                    y2 += (best_r.getY() + best_r.getHeight()) * best_s;
                    sum_w += best_s;
                    ++hits;
                }
            }
        }

        if (hits >= min_hits) {
            float ax1 = x1 / sum_w, ay1 = y1 / sum_w;
            float ax2 = x2 / sum_w, ay2 = y2 / sum_w;
            ofRectangle r(ax1, ay1, ax2 - ax1, ay2 - ay1);
            if (r.getWidth() >= min_w && r.getHeight() >= min_h) {
                out.push_back(YoloDet{r, sum_w / hits, d0.cls});
            }
        }
    }
    return out;
}

// ---- drawDetections ----
void YoloPipeline::drawDetections(int sensorW, int sensorH) const {
    if (!cfg.draw || dets_.empty()) return;

    ofPushStyle();
    ofDisableDepthTest();
    ofNoFill();
    ofSetColor(255, 215, 0);
    ofSetLineWidth(3.0f);

    ofPushMatrix();
    ofScale(ofGetWidth() / (float)sensorW, ofGetHeight() / (float)sensorH);
    ofScale(1.0f, -1.0f);
    ofTranslate(0.0f, -(float)sensorH);

    for (const auto& d : dets_) {
        ofDrawRectangle(d.box);

        std::string name = (d.cls >= 0 && d.cls < (int)cfg.class_names.size())
                           ? cfg.class_names[d.cls]
                           : ("id:" + ofToString(d.cls));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s %.2f", name.c_str(), d.score);

        if (cfg.show_labels) {
            ofPushMatrix();
            ofTranslate(d.box.getX() + 2, d.box.getY() + d.box.getHeight() - 4);
            ofScale(1.0f, -1.0f);
            ofDrawBitmapStringHighlight(buf, 0, 0, ofColor(0,0,0,180), ofColor(255,215,0));
            ofPopMatrix();
        }
    }

    ofPopMatrix();
    ofPopStyle();
}

// ---- clearHistory ----
void YoloPipeline::clearHistory() {
    smooth_hist_.clear();
    dets_.clear();
    if (stateful_) lstm_state_.assign(state_size_, 0.f);
}

// ---- setupProbes ----
// Loads a pre-built "_probed.onnx" model that has the 3 intermediate tensors
// declared as explicit graph outputs, then validates with a dummy run.
// The probe model replaces nn_ so every infer() call also materialises the probes.
void YoloPipeline::setupProbes() {
    if (!nn_ || !nn_->isLoaded()) return;

    // Build probe model path: replace ".onnx" suffix with "_probed.onnx"
    std::string base = nn_->modelPath();
    const std::string suffix = ".onnx";
    auto pos = base.rfind(suffix);
    if (pos == std::string::npos || pos + suffix.size() != base.size()) {
        ofLogWarning() << "[YoloPipeline] cannot derive probe model path from: " << base;
        return;
    }
    std::string probe_path = base.substr(0, pos) + "_probed.onnx";

    {
        std::ifstream f(probe_path);
        if (!f.good()) {
            ofLogWarning() << "[YoloPipeline] probe model not found: " << probe_path;
            return;
        }
    }

    try {
        OnnxRunner::Config nncfg;
        nncfg.model_path        = probe_path;
        nncfg.normalize_01      = true;
        nncfg.verbose           = false;

        auto probe_nn = std::make_unique<OnnxRunner>(nncfg);
        probe_nn->load();

        // Validate: dummy run with correct inputs
        if (stateful_) {
            std::vector<int64_t> frame_shape = {1, 5, (int64_t)model_H_, (int64_t)model_W_};
            std::vector<int64_t> state_shape  = {1, (int64_t)state_size_};
            std::vector<float> dummy_frame((size_t)5 * model_H_ * model_W_, 0.f);
            std::vector<float> dummy_state(state_size_, 0.f);
            probe_nn->runRawMulti({
                {dummy_frame.data(), frame_shape},
                {dummy_state.data(), state_shape}
            });
        } else {
            std::vector<float> dummy((size_t)5 * model_H_ * model_W_, 0.f);
            std::unordered_map<std::string, std::vector<float>> tmp;
            probe_nn->runCHW_into(dummy, 5, model_H_, model_W_, tmp);
        }

        // Swap in the probe runner — nn_ now outputs detections AND probe activations
        nn_ = std::move(probe_nn);
        outmap_.clear();

        for (int i = 0; i < NUM_PROBES; ++i)
            probeResults_[i].label = PROBE_LABELS[i];

        probesEnabled_ = true;
        ofLogNotice() << "[YoloPipeline] probes enabled (" << probe_path << ")";
    } catch (const std::exception& e) {
        probesEnabled_ = false;
        ofLogWarning() << "[YoloPipeline] probes disabled: " << e.what();
    }
}

// ---- extractProbes_ ----
void YoloPipeline::extractProbes_() {
    for (int p = 0; p < NUM_PROBES; ++p) {
        auto it = outmap_.find(PROBE_NAMES[p]);
        if (it == outmap_.end()) { probeResults_[p].W = 0; probeResults_[p].H = 0; continue; }

        const auto* shape = nn_->getLastOutputShape(PROBE_NAMES[p]);
        if (!shape || shape->size() < 4) { probeResults_[p].W = 0; continue; }

        const int64_t C = (*shape)[1], H = (*shape)[2], W = (*shape)[3];
        if (C <= 0 || H <= 0 || W <= 0) { probeResults_[p].W = 0; continue; }

        const std::vector<float>& raw = it->second;
        const size_t plane = (size_t)H * W;
        std::vector<float>& avg = probeResults_[p].avg;
        avg.assign(plane, 0.f);

        // Max absolute value across channels per pixel — avoids sign cancellation
        // that makes deep-layer channel means collapse to near-zero.
        for (int64_t c = 0; c < C; ++c) {
            const float* ch = raw.data() + c * plane;
            for (size_t i = 0; i < plane; ++i) {
                float a = std::abs(ch[i]);
                if (a > avg[i]) avg[i] = a;
            }
        }
        // Normalize to [0,1]; vmin is always 0 since we used abs.
        float vmax = *std::max_element(avg.begin(), avg.end());
        const float scale = vmax > 1e-6f ? (1.f / vmax) : 1.f;
        for (size_t i = 0; i < plane; ++i) avg[i] *= scale;

        probeResults_[p].W = (int)W;
        probeResults_[p].H = (int)H;
    }
}

} // namespace dvs

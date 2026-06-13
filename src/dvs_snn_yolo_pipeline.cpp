#include "dvs_snn_yolo_pipeline.hpp"
#include "ofxDVS.hpp"  // for struct polarity

#include <cmath>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <numeric>

namespace dvs {

// ── loadAnchors_ ─────────────────────────────────────────────────────────────
// Minimal numpy .npy parser for float32 arrays (shape [N, 4]).
bool SnnYoloPipeline::loadAnchors_(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { ofLogWarning() << "[SnnYolo] anchor file not found: " << path; return false; }

    // Read 6-byte magic + version
    char magic[6]; f.read(magic, 6);
    char major, minor; f.read(&major, 1); f.read(&minor, 1);

    // Read header length (uint16 little-endian)
    uint16_t hlen; f.read(reinterpret_cast<char*>(&hlen), 2);

    // Skip header (contains shape/dtype string)
    std::string header(hlen, ' ');
    f.read(&header[0], hlen);

    // Parse shape N from header: look for "N, 4" pattern
    // e.g. 'shape': (13440, 4)
    num_anchors_ = 0;
    auto pos = header.find("'shape'");
    if (pos == std::string::npos) pos = header.find("\"shape\"");
    if (pos != std::string::npos) {
        auto lp = header.find('(', pos);
        if (lp != std::string::npos)
            num_anchors_ = std::stoi(header.substr(lp + 1));
    }
    if (num_anchors_ <= 0) {
        ofLogWarning() << "[SnnYolo] could not parse anchor count from header";
        return false;
    }

    anchors_.resize((size_t)num_anchors_ * 4);
    f.read(reinterpret_cast<char*>(anchors_.data()),
           (std::streamsize)anchors_.size() * sizeof(float));
    if (!f) { ofLogWarning() << "[SnnYolo] anchor file read error"; return false; }

    ofLogNotice() << "[SnnYolo] loaded " << num_anchors_ << " anchors from " << path;
    return true;
}

// ── loadModel ────────────────────────────────────────────────────────────────
void SnnYoloPipeline::loadModel(const std::string& onnx_path,
                                 const std::string& anchors_path) {
    OnnxRunner::Config nncfg;
    nncfg.model_path    = onnx_path;
    nncfg.normalize_01  = false;  // binary 0/1 events — no further normalization
    nncfg.verbose       = false;

    nn_ = std::make_unique<OnnxRunner>(nncfg);
    nn_->load();

    if (!nn_->isLoaded()) { ofLogError() << "[SnnYolo] failed to load " << onnx_path; return; }

    // Detect state size from "state_in" input
    state_size_ = 0;
    for (const auto& inp : nn_->inputs()) {
        if (inp.name == "state_in") {
            int64_t sz = 1;
            for (auto d : inp.dims) if (d > 0) sz *= d;
            state_size_ = (int)sz;
            break;
        }
    }
    if (state_size_ <= 0) {
        ofLogError() << "[SnnYolo] 'state_in' input not found — is this a stateful model?";
        return;
    }
    state_.assign(state_size_, 0.f);
    frame_model_.assign((size_t)MODEL_C * MODEL_H * MODEL_W, 0.f);

    // Load anchors
    std::string anc_path = anchors_path;
    if (anc_path.empty()) {
        auto pos = onnx_path.rfind(".onnx");
        anc_path = (pos != std::string::npos)
                   ? onnx_path.substr(0, pos) + "_anchors.npy"
                   : onnx_path + "_anchors.npy";
    }
    if (!loadAnchors_(anc_path)) {
        ofLogWarning() << "[SnnYolo] anchors unavailable — detections will be disabled";
    }

    ofLogNotice() << "[SnnYolo] loaded " << onnx_path
                  << "  state=" << state_size_
                  << "  anchors=" << num_anchors_;
}

// ── ensureLetterbox_ ─────────────────────────────────────────────────────────
void SnnYoloPipeline::ensureLetterbox_(int sW, int sH) {
    if (sW == cached_sW_ && sH == cached_sH_) return;
    nn::letterboxParams(sW, sH, MODEL_W, MODEL_H, lb_scale_, lb_padx_, lb_pady_);
    cached_sW_ = sW; cached_sH_ = sH;
    frame_model_.assign((size_t)MODEL_C * MODEL_H * MODEL_W, 0.f);
    ofLogNotice() << "[SnnYolo] letterbox scale=" << lb_scale_
                  << " pad=" << lb_padx_ << "," << lb_pady_;
}

// ── pushEvents ───────────────────────────────────────────────────────────────
void SnnYoloPipeline::pushEvents(const std::vector<polarity>& events, int sW, int sH) {
    if (!isLoaded() || events.empty()) return;
    ensureLetterbox_(sW, sH);

    const long bin_us = static_cast<long>(cfg.bin_ms * 1000.f);

    for (const auto& e : events) {
        if (!e.valid) continue;

        if (win_start_ts_ < 0) win_start_ts_ = e.timestamp;
        if (e.timestamp > win_end_ts_) win_end_ts_ = e.timestamp;

        // Map sensor pixel → letterboxed model pixel
        int mx = static_cast<int>(e.pos.x * lb_scale_) + lb_padx_;
        int my = static_cast<int>(e.pos.y * lb_scale_) + lb_pady_;
        if ((unsigned)mx >= (unsigned)MODEL_W || (unsigned)my >= (unsigned)MODEL_H) continue;

        // Channel 0 = negative, channel 1 = positive
        int ch = e.pol ? 1 : 0;
        size_t idx = (size_t)ch * MODEL_H * MODEL_W + (size_t)my * MODEL_W + mx;
        frame_model_[idx] = 1.f;   // binary: mark pixel active
    }
}

// ── maybeInfer ───────────────────────────────────────────────────────────────
bool SnnYoloPipeline::maybeInfer(int sW, int sH) {
    if (!isLoaded() || num_anchors_ == 0) return false;
    if (win_start_ts_ < 0) return false;

    const long bin_us = static_cast<long>(cfg.bin_ms * 1000.f);
    if (win_end_ts_ - win_start_ts_ < bin_us) return false;

    // Run ONNX: inputs are frame [1,2,256,320] and state [state_size]
    std::vector<int64_t> frame_shape = {1, MODEL_C, MODEL_H, MODEL_W};
    std::vector<int64_t> state_shape = {(int64_t)state_size_};

    outmap_ = nn_->runRawMulti({
        {frame_model_.data(), frame_shape},
        {state_.data(),       state_shape},
    });

    // Reset frame buffer for next accumulation window
    std::fill(frame_model_.begin(), frame_model_.end(), 0.f);
    win_start_ts_ = -1;
    win_end_ts_   = -1;

    if (outmap_.empty()) return false;

    // Update state
    auto sit = outmap_.find("state_out");
    if (sit != outmap_.end() && (int)sit->second.size() == state_size_)
        state_ = sit->second;

    // Decode detections
    auto cit = outmap_.find("cls");
    auto bit = outmap_.find("bbox");
    if (cit == outmap_.end() || bit == outmap_.end()) return false;

    decodeAndStore_(cit->second, bit->second, sW, sH);
    return true;
}

// ── offsetInverse_ ───────────────────────────────────────────────────────────
// Matches Python: box_corner_to_center → offset_inverse → box_center_to_corner
void SnnYoloPipeline::offsetInverse_(float ax1, float ay1, float ax2, float ay2,
                                      float dx,  float dy,  float dw,  float dh,
                                      float& ox1, float& oy1, float& ox2, float& oy2) {
    // anchor → center format
    float acx = (ax1 + ax2) * 0.5f;
    float acy = (ay1 + ay2) * 0.5f;
    float aw  = ax2 - ax1;
    float ah  = ay2 - ay1;

    // decode
    float pcx = dx * aw / 10.f + acx;
    float pcy = dy * ah / 10.f + acy;
    float pw  = std::exp(dw / 5.f) * aw;
    float ph  = std::exp(dh / 5.f) * ah;

    ox1 = std::max(0.f, pcx - pw * 0.5f);
    oy1 = std::max(0.f, pcy - ph * 0.5f);
    ox2 = std::min(1.f, pcx + pw * 0.5f);
    oy2 = std::min(1.f, pcy + ph * 0.5f);
}

// ── decodeAndStore_ ───────────────────────────────────────────────────────────
void SnnYoloPipeline::decodeAndStore_(const std::vector<float>& cls_raw,
                                       const std::vector<float>& bbox_raw,
                                       int sW, int sH) {
    // cls_raw : [1, N, num_classes+1]   (batch=1)
    // bbox_raw: [1, N, 4]

    const int N  = num_anchors_;
    const int nc = cfg.num_classes + 1;  // +1 for background (class 0)

    if ((int)cls_raw.size()  != N * nc) return;
    if ((int)bbox_raw.size() != N * 4)  return;

    ensureLetterbox_(sW, sH);

    std::vector<nn::Det> raw;
    raw.reserve(128);

    for (int i = 0; i < N; ++i) {
        // softmax over nc classes
        float maxv = *std::max_element(cls_raw.data() + i * nc,
                                       cls_raw.data() + i * nc + nc);
        float sum = 0.f;
        std::vector<float> prob(nc);
        for (int c = 0; c < nc; ++c) {
            prob[c] = std::exp(cls_raw[i * nc + c] - maxv);
            sum += prob[c];
        }
        for (float& p : prob) p /= sum;

        // best foreground class (skip background = 0)
        int   best_cls = -1;
        float best_p   = cfg.conf_thresh;
        for (int c = 1; c < nc; ++c) {
            if (prob[c] > best_p) { best_p = prob[c]; best_cls = c - 1; }
        }
        if (best_cls < 0) continue;

        // decode anchor box
        float ax1 = anchors_[i * 4 + 0], ay1 = anchors_[i * 4 + 1];
        float ax2 = anchors_[i * 4 + 2], ay2 = anchors_[i * 4 + 3];
        float dx   = bbox_raw[i * 4 + 0], dy  = bbox_raw[i * 4 + 1];
        float dw   = bbox_raw[i * 4 + 2], dh  = bbox_raw[i * 4 + 3];

        float ox1, oy1, ox2, oy2;
        offsetInverse_(ax1, ay1, ax2, ay2, dx, dy, dw, dh, ox1, oy1, ox2, oy2);
        if (ox2 <= ox1 || oy2 <= oy1) continue;

        // anchors are normalised to model dims → convert to sensor coords via un-letterbox
        float mx1 = ox1 * MODEL_W, my1 = oy1 * MODEL_H;
        float mx2 = ox2 * MODEL_W, my2 = oy2 * MODEL_H;

        auto r = nn::unletterboxToSensor(mx1, my1, mx2, my2,
                                          lb_scale_, lb_padx_, lb_pady_, sW, sH);
        if (r.getWidth() <= 1.f || r.getHeight() <= 1.f) continue;
        if (r.getWidth() * r.getHeight() < 100.f) continue;

        nn::Det d;
        d.x1 = r.x; d.y1 = r.y;
        d.x2 = r.x + r.width; d.y2 = r.y + r.height;
        d.score = best_p;
        d.cls   = best_cls;
        raw.push_back(d);
    }

    auto kept = nn::nms(std::move(raw), cfg.iou_thresh);

    std::vector<YoloDet> cur;
    cur.reserve(kept.size());
    for (auto& k : kept)
        cur.push_back(YoloDet{ofRectangle(k.x1, k.y1, k.x2 - k.x1, k.y2 - k.y1),
                              k.score, k.cls});

    dets_ = temporalSmooth_(cur);
}

// ── temporalSmooth_ (same logic as YoloPipeline) ─────────────────────────────
std::vector<YoloDet> SnnYoloPipeline::temporalSmooth_(const std::vector<YoloDet>& cur) {
    const int   max_hist  = cfg.smooth_frames;
    const float match_iou = 0.5f;
    const int   min_hits  = 2;
    const float min_wh    = 8.f;

    smooth_hist_.push_back(cur);
    if ((int)smooth_hist_.size() > max_hist) smooth_hist_.pop_front();

    std::vector<YoloDet> out;
    out.reserve(cur.size());
    for (const auto& d0 : cur) {
        if (d0.box.getWidth() < min_wh || d0.box.getHeight() < min_wh) continue;
        float sw = d0.score;
        float x1 = d0.box.x * sw, y1 = d0.box.y * sw;
        float x2 = (d0.box.x + d0.box.width)  * sw;
        float y2 = (d0.box.y + d0.box.height) * sw;
        int hits = 1;
        for (int t = (int)smooth_hist_.size() - 2; t >= 0; --t) {
            for (const auto& p : smooth_hist_[t]) {
                if (p.cls != d0.cls) continue;
                float iou = nn::rectIoU(d0.box, p.box);
                if (iou >= match_iou) {
                    x1 += p.box.x * p.score; y1 += p.box.y * p.score;
                    x2 += (p.box.x + p.box.width)  * p.score;
                    y2 += (p.box.y + p.box.height) * p.score;
                    sw += p.score; ++hits; break;
                }
            }
        }
        if (hits >= min_hits) {
            out.push_back(YoloDet{
                ofRectangle(x1/sw, y1/sw, (x2-x1)/sw, (y2-y1)/sw),
                sw / hits, d0.cls});
        }
    }
    return out;
}

// ── drawDetections ───────────────────────────────────────────────────────────
void SnnYoloPipeline::drawDetections(int sW, int sH) const {
    if (!cfg.draw || dets_.empty()) return;

    ofPushStyle();
    ofDisableDepthTest();
    ofNoFill();
    ofSetColor(0, 200, 255);   // cyan — distinct from ReYOLO yellow
    ofSetLineWidth(2.5f);

    ofPushMatrix();
    ofScale(ofGetWidth() / (float)sW, ofGetHeight() / (float)sH);
    ofScale(1.f, -1.f);
    ofTranslate(0.f, -(float)sH);

    for (const auto& d : dets_) {
        ofDrawRectangle(d.box);
        if (cfg.show_labels) {
            const std::string& name = (d.cls >= 0 && d.cls < (int)cfg.class_names.size())
                                      ? cfg.class_names[d.cls] : "?";
            char buf[64];
            std::snprintf(buf, sizeof(buf), "[SNN] %s %.2f", name.c_str(), d.score);
            ofPushMatrix();
            ofTranslate(d.box.x + 2, d.box.y + d.box.height - 4);
            ofScale(1.f, -1.f);
            ofDrawBitmapStringHighlight(buf, 0, 0,
                ofColor(0, 0, 0, 180), ofColor(0, 200, 255));
            ofPopMatrix();
        }
    }

    ofPopMatrix();
    ofPopStyle();
}

// ── clearHistory ─────────────────────────────────────────────────────────────
void SnnYoloPipeline::clearHistory() {
    smooth_hist_.clear();
    dets_.clear();
    std::fill(state_.begin(), state_.end(), 0.f);
    std::fill(frame_model_.begin(), frame_model_.end(), 0.f);
    win_start_ts_ = -1;
    win_end_ts_   = -1;
}

} // namespace dvs

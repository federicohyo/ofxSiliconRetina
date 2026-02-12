#include "dvs_yolo_pipeline.hpp"
#include "ofxDVS.hpp" // for struct polarity

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace dvs {

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

    ofLogNotice() << "[YoloPipeline] loaded " << path
                  << " model=" << model_W_ << "x" << model_H_;
}

// ---- ensureLetterboxParams_ ----
void YoloPipeline::ensureLetterboxParams_(int sW, int sH) {
    if (sW == cached_sW_ && sH == cached_sH_) return;
    nn::letterboxParams(sW, sH, model_W_, model_H_, lb_scale_, lb_padx_, lb_pady_);
    cached_sW_ = sW;
    cached_sH_ = sH;
}

// ---- buildVTEI (single-pass) ----
std::vector<float> YoloPipeline::buildVTEI(
    const std::vector<polarity>& events,
    float** surfaceMapLastTs,
    const ofPixels& intensity,
    int sW, int sH)
{
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

    // Convert intensity to grayscale (matches original which called setImageType(OF_IMAGE_GRAYSCALE))
    E_buf_.assign(plane, 0.f);
    ofPixels grayPx;
    bool haveGray = false;
    if (intensity.isAllocated() && (int)intensity.getWidth() == sW && (int)intensity.getHeight() == sH) {
        if (intensity.getNumChannels() == 1) {
            grayPx = intensity;
        } else {
            grayPx.allocate(sW, sH, OF_IMAGE_GRAYSCALE);
            const int nc = intensity.getNumChannels();
            for (int y = 0; y < sH; ++y) {
                for (int x = 0; x < sW; ++x) {
                    size_t src = ((size_t)y * sW + x) * nc;
                    unsigned char r = intensity[src + 0];
                    unsigned char g = intensity[src + 1];
                    unsigned char b = intensity[src + 2];
                    grayPx[y * sW + x] = (unsigned char)(0.299f*r + 0.587f*g + 0.114f*b);
                }
            }
        }
        haveGray = true;
    }

    // Edge magnitude (Sobel) from grayscale intensity
    if (haveGray) {
        auto P = [&](int yy, int xx) -> unsigned char { return grayPx[yy * sW + xx]; };
        for (int y = 1; y < sH - 1; ++y) {
            for (int x = 1; x < sW - 1; ++x) {
                float gx = float(P(y-1,x+1) + 2*P(y,x+1) + P(y+1,x+1)
                               - P(y-1,x-1) - 2*P(y,x-1) - P(y+1,x-1));
                float gy = float(P(y+1,x-1) + 2*P(y+1,x) + P(y+1,x+1)
                               - P(y-1,x-1) - 2*P(y-1,x) - P(y-1,x+1));
                float mag = std::sqrt(gx*gx + gy*gy) / (4.f * 255.f);
                E_buf_[y * sW + x] = std::clamp(mag, 0.f, 1.f);
            }
        }
    }

    // Pack into CHW (C=5: pos, neg, T, E, I)
    const size_t C5 = 5;
    chw5_sensor_.resize(C5 * plane);

    for (int y = 0; y < sH; ++y) {
        for (int x = 0; x < sW; ++x) {
            size_t hw = (size_t)y * sW + x;
            float i01 = (haveGray ? grayPx[hw] / 255.f : 0.f);
            chw5_sensor_[0 * plane + hw] = pos_buf_[hw];
            chw5_sensor_[1 * plane + hw] = neg_buf_[hw];
            chw5_sensor_[2 * plane + hw] = T_buf_[hw];
            chw5_sensor_[3 * plane + hw] = E_buf_[hw];
            chw5_sensor_[4 * plane + hw] = i01;
        }
    }

    return chw5_sensor_;
}

// ---- infer ----
void YoloPipeline::infer(const std::vector<float>& vtei_sensor_chw,
                          int sensorW, int sensorH)
{
    if (!nn_ || !nn_->isLoaded()) { dets_.clear(); return; }

    ensureLetterboxParams_(sensorW, sensorH);

    const int C5 = 5;

    // Letterbox sensor -> model
    chw5_model_ = nn::letterboxCHW(
        vtei_sensor_chw, C5, sensorH, sensorW,
        model_H_, model_W_, lb_scale_, lb_padx_, lb_pady_);

    if (chw5_model_.size() != (size_t)C5 * model_H_ * model_W_) {
        ofLogError() << "[YOLO] letterbox produced wrong size";
        dets_.clear();
        return;
    }

    // Run ONNX
    nn_->runCHW_into(chw5_model_, C5, model_H_, model_W_, outmap_);
    if (outmap_.empty()) { dets_.clear(); return; }

    // Find output
    const std::vector<float>* pv = nullptr;
    auto it0 = outmap_.find("output0");
    if (it0 != outmap_.end()) pv = &it0->second;
    else                      pv = &outmap_.begin()->second;
    const std::vector<float>& v = *pv;

    // Decode: out0 = [1, C, N] where C = 4 + nc
    const int nc = cfg.num_classes + 1; // PEDRo export uses nc=2
    const int C  = 4 + nc;
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
        for (int c = 0; c < nc; ++c) {
            float p = nn::sigmoid(at(4 + c, i));
            if (p > best_p) { best_p = p; best_cls = c; }
        }
        if (best_p < cfg.conf_thresh) continue;
        if (w <= 1.f || h <= 1.f) continue;
        float ar = w / std::max(1.f, h);
        if (ar < 0.15f || ar > 6.7f) continue;

        nn::Det d;
        d.x1 = cx - 0.5f*w; d.y1 = cy - 0.5f*h;
        d.x2 = cx + 0.5f*w; d.y2 = cy + 0.5f*h;
        d.score = best_p;
        d.cls   = best_cls;
        raw_dets.push_back(d);
    }

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
}

} // namespace dvs

#include "dvs_tsdt_pipeline.hpp"
#include "ofxDVS.hpp" // for struct polarity

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cfloat>
#include <fstream>
#include <numeric>
#include <limits>

namespace dvs {

// ---- loadModel ----
void TsdtPipeline::loadModel(const std::string& path, int threads) {
    OnnxRunner::Config scfg;
    scfg.model_path = path;
    if (threads > 0) scfg.intra_op_num_threads = threads;
    scfg.normalize_01 = false;  // we feed already-prepared [0,1] binary maps
    scfg.verbose = false;

    tsdt_ = std::make_unique<OnnxRunner>(scfg);
    tsdt_->load();

    cached_sW_ = 0;
    cached_sH_ = 0;

    // Auto-detect stateful SNN model (has "state_in" input)
    stateful_ = false;
    state_size_ = 0;
    snn_state_.clear();
    for (const auto& inp : tsdt_->inputs()) {
        if (inp.name == "state_in") {
            stateful_ = true;
            // Compute total state size from dims (e.g. [1, 409611])
            int64_t sz = 1;
            for (auto d : inp.dims) {
                if (d > 0) sz *= d;
            }
            state_size_ = (int)sz;
            snn_state_.assign(state_size_, 0.f);
            ofLogNotice() << "[" << cfg.log_tag << "] stateful SNN model (state_size=" << state_size_ << ")";
            break;
        }
    }

    ofLogNotice() << "[TsdtPipeline] loaded " << path;
}

// ---- ensureLetterboxParams_ ----
void TsdtPipeline::ensureLetterboxParams_(int sW, int sH) {
    if (sW == cached_sW_ && sH == cached_sH_) return;
    lb_scale_ = std::min((float)cfg.inW / (float)sW,
                         (float)cfg.inH / (float)sH);
    lb_padx_ = (cfg.inW - (int)std::round(sW * lb_scale_)) / 2;
    lb_pady_ = (cfg.inH - (int)std::round(sH * lb_scale_)) / 2;
    cached_sW_ = sW;
    cached_sH_ = sH;
}

// ---- pushEvents ----
void TsdtPipeline::pushEvents(const std::vector<polarity>& events,
                               int sensorW, int sensorH) {
    for (const auto& p : events) {
        if (!p.valid) continue;
        int x = (int)p.pos.x, y = (int)p.pos.y;
        if ((unsigned)x >= (unsigned)sensorW || (unsigned)y >= (unsigned)sensorH)
            continue;

        // Detect timestamp backward jump (file loop): auto-clear history
        if (cfg.time_based_binning && !hist_.empty() &&
            p.timestamp < hist_.back().ts - 1000000) {
            hist_.clear();
            ema_logits_.clear();
        }

        hist_.push_back({x, y, (bool)p.pol, p.timestamp});
    }

    if (cfg.time_based_binning) {
        // Time-based cap: keep events within 3x the total window span
        if (!hist_.empty()) {
            long latest = hist_.back().ts;
            long horizon = (long)(3.0f * cfg.T * cfg.bin_window_ms * 1000.f); // us
            long cutoff = latest - horizon;
            while (!hist_.empty() && hist_.front().ts < cutoff)
                hist_.pop_front();
        }
    } else {
        // Event-count cap: keep ~2x what we need + slack
        const size_t need = (size_t)cfg.T * (size_t)cfg.ev_per_bin;
        const size_t cap  = need * 2 + 2000;
        if (hist_.size() > cap) {
            hist_.erase(hist_.begin(), hist_.begin() + (std::ptrdiff_t)(hist_.size() - cap));
        }
    }
}

// ---- buildTensor (supports event-count bins and time-based bins) ----
std::vector<float> TsdtPipeline::buildTensor(int sensorW, int sensorH) {
    ensureLetterboxParams_(sensorW, sensorH);

    const int T = cfg.T;
    const int Hd = cfg.inH, Wd = cfg.inW;
    const size_t plane = (size_t)Hd * Wd;

    // --- Stateful SNN path: accumulate at sensor res, then anti-aliased resize ---
    if (stateful_) {
        if (hist_.empty()) return {};

        // Check readiness (same as time-based)
        if (cfg.time_based_binning) {
            float winUs = cfg.bin_window_ms * 1000.f;
            long span = hist_.back().ts - hist_.front().ts;
            if (span < (long)winUs) return {};
        }

        // Accumulate event counts at full sensor resolution (2 channels)
        const size_t sensorPlane = (size_t)sensorW * sensorH;
        sensor_buf_.assign(2 * sensorPlane, 0.f);

        if (cfg.time_based_binning) {
            float winUs = cfg.bin_window_ms * 1000.f;
            long windowStart = hist_.back().ts - (long)(T * winUs);
            for (const auto& e : hist_) {
                if (e.ts < windowStart) continue;
                if ((unsigned)e.x >= (unsigned)sensorW || (unsigned)e.y >= (unsigned)sensorH) continue;
                size_t hw = (size_t)e.y * sensorW + e.x;
                sensor_buf_[0 * sensorPlane + hw] += (e.p ? 0.f : 1.f);  // OFF → ch0
                sensor_buf_[1 * sensorPlane + hw] += (e.p ? 1.f : 0.f);  // ON  → ch1
            }
        } else {
            const size_t need = (size_t)T * (size_t)cfg.ev_per_bin;
            if (hist_.size() < need) return {};
            size_t start = hist_.size() - need;
            for (size_t i = start; i < hist_.size(); ++i) {
                const auto& e = hist_[i];
                if ((unsigned)e.x >= (unsigned)sensorW || (unsigned)e.y >= (unsigned)sensorH) continue;
                size_t hw = (size_t)e.y * sensorW + e.x;
                sensor_buf_[0 * sensorPlane + hw] += (e.p ? 0.f : 1.f);
                sensor_buf_[1 * sensorPlane + hw] += (e.p ? 1.f : 0.f);
            }
        }

        // Anti-aliased resize each channel from sensor res to model input res
        tsdt_tensor_.resize(2 * plane);
        for (int c = 0; c < 2; ++c) {
            cv::Mat src(sensorH, sensorW, CV_32FC1, sensor_buf_.data() + c * sensorPlane);
            cv::Mat dst(Hd, Wd, CV_32FC1, tsdt_tensor_.data() + c * plane);
            cv::resize(src, dst, cv::Size(Wd, Hd), 0, 0, cv::INTER_AREA);
        }

        return tsdt_tensor_;
    }

    if (cfg.time_based_binning) {
        // --- Time-based binning path ---
        if (hist_.empty()) return {};

        long latest = hist_.back().ts;
        float winUs = cfg.bin_window_ms * 1000.f;
        long totalSpan = (long)(T * winUs);

        // Require history to span at least one full window
        long earliest = hist_.front().ts;
        if ((latest - earliest) < (long)winUs) return {};

        tsdt_tensor_.assign((size_t)T * 2 * plane, 0.f);

        long windowStart = latest - totalSpan;

        for (const auto& e : hist_) {
            if (e.ts < windowStart) continue;

            // Determine which temporal bin this event belongs to
            int t = (int)((e.ts - windowStart) / winUs);
            if (t < 0) continue;
            if (t >= T) t = T - 1;

            int dx = (int)std::round(e.x * lb_scale_) + lb_padx_;
            int dy = (int)std::round(e.y * lb_scale_) + lb_pady_;
            if ((unsigned)dx >= (unsigned)Wd || (unsigned)dy >= (unsigned)Hd) continue;

            size_t hw = (size_t)dy * Wd + dx;
            size_t base = ((size_t)t * 2) * plane;
            tsdt_tensor_[base + 0 * plane + hw] += (e.p ? 0.f : 1.f);
            tsdt_tensor_[base + 1 * plane + hw] += (e.p ? 1.f : 0.f);
        }

        return tsdt_tensor_;
    }

    // --- Event-count binning path (original) ---
    const int evPerBin = cfg.ev_per_bin;
    const size_t need = (size_t)T * (size_t)evPerBin;

    if (hist_.size() < need) return {};

    tsdt_tensor_.assign((size_t)T * 2 * plane, 0.f);

    const size_t start = hist_.size() - need;
    size_t idxEv = start;

    for (int t = 0; t < T; ++t) {
        for (int k = 0; k < evPerBin; ++k, ++idxEv) {
            const auto& e = hist_[idxEv];
            int x = e.x, y = e.y;

            int dx = (int)std::round(x * lb_scale_) + lb_padx_;
            int dy = (int)std::round(y * lb_scale_) + lb_pady_;
            if ((unsigned)dx >= (unsigned)Wd || (unsigned)dy >= (unsigned)Hd) continue;

            size_t hw = (size_t)dy * Wd + dx;
            size_t base = ((size_t)t * 2) * plane;
            tsdt_tensor_[base + 0 * plane + hw] += (e.p ? 0.f : 1.f);
            tsdt_tensor_[base + 1 * plane + hw] += (e.p ? 1.f : 0.f);
        }
    }

    return tsdt_tensor_;
}

// ---- infer ----
std::pair<int, float> TsdtPipeline::infer(int sensorW, int sensorH) {
    if (!tsdt_ || !tsdt_->isLoaded()) return {-1, 0.f};

    if (cfg.time_based_binning) {
        // Rate-limit: don't run more often than the window period
        if (last_predict_time_ > 0.f) {
            float elapsed = ofGetElapsedTimef() - last_predict_time_;
            if (elapsed < cfg.bin_window_ms / 1000.f) return {last_idx_, last_conf_};
        }
        // Time-based readiness: need events spanning at least one window
        if (hist_.empty()) return {-1, 0.f};
        float winUs = cfg.bin_window_ms * 1000.f;
        long span = hist_.back().ts - hist_.front().ts;
        if (span < (long)winUs) return {-1, 0.f};
    } else {
        const size_t need = (size_t)cfg.T * (size_t)cfg.ev_per_bin;
        if (hist_.size() < need) return {-1, 0.f};
    }

    auto tensor = buildTensor(sensorW, sensorH);
    if (tensor.empty()) return {-1, 0.f};

    try {
        std::map<std::string, std::vector<float>> outmap;

        if (stateful_) {
            // Stateful SNN: shape [1, 2, H, W], pass state_in alongside
            std::vector<int64_t> frame_shape = {1, 2, (int64_t)cfg.inH, (int64_t)cfg.inW};
            std::vector<int64_t> state_shape = {1, (int64_t)state_size_};
            std::vector<std::pair<const float*, std::vector<int64_t>>> inputs = {
                {tensor.data(), frame_shape},
                {snn_state_.data(), state_shape}
            };
            outmap = tsdt_->runRawMulti(inputs);

            // Update SNN membrane state from output
            auto sit = outmap.find("state_out");
            if (sit != outmap.end()) {
                snn_state_ = sit->second;
            }
        } else {
            // Non-stateful: shape [1, T, 2, H, W]
            std::vector<int64_t> shape = {1, (int64_t)cfg.T, 2, cfg.inH, cfg.inW};
            outmap = tsdt_->runRaw(tensor.data(), shape);
        }

        // Pick logits
        const std::vector<float>* pv = nullptr;
        auto it = outmap.find("logits");
        pv = (it != outmap.end()) ? &it->second : &outmap.begin()->second;
        const auto& logits = *pv;

        // EMA smoothing (skipped when ema_alpha == 1.0; SNN state handles temporal integration)
        if (ema_logits_.size() != logits.size())
            ema_logits_.assign(logits.size(), 0.f);
        for (size_t i = 0; i < logits.size(); ++i)
            ema_logits_[i] = cfg.ema_alpha * logits[i]
                           + (1.f - cfg.ema_alpha) * ema_logits_[i];

        // Softmax
        float maxv = *std::max_element(ema_logits_.begin(), ema_logits_.end());
        softmax_exps_.resize(ema_logits_.size());
        float sum = 0.f;
        for (size_t i = 0; i < softmax_exps_.size(); ++i) {
            softmax_exps_[i] = std::exp(ema_logits_[i] - maxv);
            sum += softmax_exps_[i];
        }

        float bestp = 0.f; int besti = -1;
        for (size_t i = 0; i < softmax_exps_.size(); ++i) {
            float p = softmax_exps_[i] / sum;
            if (p > bestp) { bestp = p; besti = (int)i; }
        }
        last_idx_  = besti;
        last_conf_ = bestp;
        last_predict_time_ = ofGetElapsedTimef();

        // Tensor diagnostics + logits (verbose, rate-limited by inference rate)
        {
            float tsum = 0.f; int tnz = 0;
            for (float v : tensor) { tsum += v; if (v != 0.f) ++tnz; }

            std::ostringstream oss; oss.setf(std::ios::fixed); oss.precision(4);
            for (size_t i = 0; i < logits.size(); ++i) {
                if (i) oss << ", ";
                oss << logits[i];
            }
            ofLogVerbose() << "[" << cfg.log_tag << "] tensor: sum=" << tsum
                           << " nonzero=" << tnz << "/" << tensor.size()
                           << " hist=" << hist_.size();
            ofLogVerbose() << "[" << cfg.log_tag << "] logits: " << oss.str();
            ofLogNotice()  << "[" << cfg.log_tag << "] argmax=" << last_idx_
                           << " conf=" << last_conf_;
        }

        // Consume the events we used
        if (cfg.time_based_binning) {
            // Erase events older than the consumed window
            float winUs = cfg.bin_window_ms * 1000.f;
            long totalSpan = (long)(cfg.T * winUs);
            long cutoff = hist_.back().ts - totalSpan;
            while (!hist_.empty() && hist_.front().ts < cutoff)
                hist_.pop_front();
        } else {
            const size_t need = (size_t)cfg.T * (size_t)cfg.ev_per_bin;
            hist_.erase(hist_.begin(), hist_.begin() + (std::ptrdiff_t)need);
        }

    } catch (const std::exception& e) {
        ofLogError() << "[" << cfg.log_tag << "] inference error: " << e.what();
        return {-1, 0.f};
    }

    return {last_idx_, last_conf_};
}

// ---- drawLabel ----
void TsdtPipeline::drawLabel() const {
    if (!(cfg.show_label && last_idx_ >= 0)) return;
    if (last_conf_ < cfg.conf_threshold) return;
    if (cfg.display_timeout > 0.f) {
        float elapsed = ofGetElapsedTimef() - last_predict_time_;
        if (elapsed > cfg.display_timeout) return;
    }

    std::string name = (last_idx_ < (int)cfg.labels.size())
        ? cfg.labels[last_idx_]
        : ("class" + std::to_string(last_idx_));
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s (%.2f)", name.c_str(), last_conf_);

    // Font (cached via static)
    const int fontPx = 36;
    static ofTrueTypeFont font;
    static int loadedSize = 0;
    static bool fontOk = false;

    auto ensureFont = [&](int px) {
        if (fontOk && loadedSize == px) return;
        fontOk = false;
        std::string path = ofToDataPath("fonts/Label.ttf", true);
        if (!ofFile::doesFileExist(path)) path = ofToDataPath("verdana.ttf", true);
        ofTrueTypeFontSettings s(path, px);
        s.antialiased = true;
        s.dpi = 96;
        s.addRanges(ofAlphabet::Latin);
        fontOk = font.load(s);
        loadedSize = px;
    };
    ensureFont(fontPx);

    const float cx = ofGetWidth() * 0.5f;
    const float marginBottom = 650.0f;
    const float yBottom = ofGetHeight() - marginBottom + cfg.label_y_offset;

    ofPushStyle();
    if (fontOk) {
        ofRectangle bb = font.getStringBoundingBox(buf, 0, 0);
        const float pad = 12.0f;
        float drawX = cx - bb.width * 0.5f;
        float drawY = yBottom - bb.y - bb.height;

        ofSetColor(0, 0, 0, 160);
        ofDrawRectRounded(drawX + bb.x - pad, drawY + bb.y - pad,
                          bb.width + 2*pad, bb.height + 2*pad, 8);
        ofSetColor(255, 215, 0);
        font.drawString(buf, drawX, drawY - 22);
    } else {
        const float s = 2.0f;
        ofPushMatrix();
        ofTranslate(cx, yBottom);
        ofScale(s, s);
        ofSetColor(255, 215, 0);
        ofDrawBitmapStringHighlight(buf, -30.f, -8.f, ofColor(0,0,0,180), ofColor(255,215,0));
        ofPopMatrix();
    }
    ofPopStyle();
}

// ---- selfTest ----
void TsdtPipeline::selfTest() {
    if (!tsdt_ || !tsdt_->isLoaded()) {
        ofLogError() << "[TSDT/SELFTEST] model not loaded";
        return;
    }
    const int T = cfg.T > 0 ? cfg.T : 8;
    const int H = cfg.inH > 0 ? cfg.inH : 128;
    const int W = cfg.inW > 0 ? cfg.inW : 128;

    std::vector<float> x((size_t)T * 2 * H * W);
    for (size_t i = 0; i < x.size(); ++i)
        x[i] = float((i % 97) + 1) / 97.f;

    std::vector<int64_t> shape = {1, (int64_t)T, 2, H, W};

    try {
        auto out = tsdt_->runRaw(x.data(), shape);
        for (auto& kv : out) {
            const auto& name = kv.first;
            const auto& v    = kv.second;
            double sum = 0, mn = std::numeric_limits<double>::infinity(), mx = -mn;
            for (float f : v) { sum += f; mn = std::min(mn, (double)f); mx = std::max(mx, (double)f); }
            ofLogNotice() << "[TSDT/SELFTEST] out=" << name
                          << " size=" << v.size()
                          << " sum=" << sum << " min=" << mn << " max=" << mx;
            if (!v.empty()) {
                std::ostringstream os;
                os.setf(std::ios::fixed); os << std::setprecision(4);
                int n = std::min<int>(v.size(), 8);
                for (int i = 0; i < n; ++i) { if (i) os << ", "; os << v[i]; }
                ofLogNotice() << "[TSDT/SELFTEST] head: " << os.str();
            }
        }
    } catch (const std::exception& e) {
        ofLogError() << "[TSDT/SELFTEST] runRaw failed: " << e.what();
    }
}

// ---- debugFromFile ----
void TsdtPipeline::debugFromFile(const std::string& binPath) {
    if (!tsdt_ || !tsdt_->isLoaded()) {
        ofLogError() << "[TSDT/DEBUG] model not loaded";
        return;
    }

    const std::vector<int64_t> shape = {1, 8, 2, 128, 128};
    const size_t numel = (size_t)(shape[0] * shape[1] * shape[2] * shape[3] * shape[4]);

    std::ifstream f(binPath, std::ios::binary);
    if (!f) {
        ofLogError() << "[TSDT/DEBUG] cannot open " << binPath;
        return;
    }

    std::vector<float> x(numel, 0.f);
    f.read(reinterpret_cast<char*>(x.data()), (std::streamsize)(numel * sizeof(float)));
    if (f.gcount() != (std::streamsize)(numel * sizeof(float))) {
        ofLogError() << "[TSDT/DEBUG] short read";
        return;
    }

    {
        float s = 0.f, mn = FLT_MAX, mx = -FLT_MAX;
        for (float v : x) { s += v; mn = std::min(mn, v); mx = std::max(mx, v); }
        ofLogNotice() << "[TSDT/DEBUG] input numel=" << numel
                      << " sum=" << s << " min=" << mn << " max=" << mx;
    }

    try {
        auto out = tsdt_->runRaw(x.data(), shape);
        if (out.empty()) { ofLogError() << "[TSDT/DEBUG] empty outputs"; return; }

        auto it = out.find("logits");
        if (it == out.end()) it = out.begin();
        const auto& y = it->second;

        float sum = 0.f, minv = FLT_MAX, maxv = -FLT_MAX;
        int argmax = -1; float best = -FLT_MAX;
        for (int i = 0; i < (int)y.size(); ++i) {
            float v = y[i]; sum += v;
            minv = std::min(minv, v); maxv = std::max(maxv, v);
            if (v > best) { best = v; argmax = i; }
        }

        std::ostringstream oss; oss.setf(std::ios::fixed); oss.precision(4);
        for (int i = 0; i < (int)y.size(); ++i) { if (i) oss << ", "; oss << y[i]; }

        ofLogNotice() << "[TSDT/DEBUG] out=" << it->first
                      << " size=" << y.size()
                      << " sum=" << sum << " min=" << minv << " max=" << maxv;
        ofLogNotice() << "[TSDT/DEBUG] logits: " << oss.str();
        ofLogNotice() << "[TSDT/DEBUG] argmax=" << argmax << " val=" << best;

    } catch (const std::exception& e) {
        ofLogError() << "[TSDT/DEBUG] inference error: " << e.what();
    }
}

// ---- clearHistory ----
void TsdtPipeline::clearHistory() {
    hist_.clear();
    last_idx_  = -1;
    last_conf_ = 0.f;
    last_predict_time_ = 0.f;
    ema_logits_.clear();
    std::fill(snn_state_.begin(), snn_state_.end(), 0.f);
}

} // namespace dvs

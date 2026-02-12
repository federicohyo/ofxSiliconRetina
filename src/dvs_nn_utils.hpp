#pragma once
/// @file dvs_nn_utils.hpp
/// @brief Shared neural-network utility functions for the ofxDVS addon.
///
/// Header-only. All functions live in `namespace dvs::nn`.

#include <vector>
#include <algorithm>
#include <cmath>

#include "ofMain.h"

namespace dvs { namespace nn {

// ---------------------------------------------------------------------------
// Activation
// ---------------------------------------------------------------------------

/// Standard sigmoid activation.
inline float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

// ---------------------------------------------------------------------------
// Detection helpers
// ---------------------------------------------------------------------------

/// Axis-aligned bounding box with class label, used for NMS.
struct Det { float x1, y1, x2, y2, score; int cls; };

/// Intersection-over-Union for two Det boxes.
inline float IoU(const Det& a, const Det& b) {
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;
    float areaA = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float areaB = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    return inter / std::max(1e-6f, areaA + areaB - inter);
}

/// Greedy non-maximum suppression, sorted by score descending.
inline std::vector<Det> nms(std::vector<Det> dets, float iou_thresh) {
    std::vector<Det> keep;
    std::sort(dets.begin(), dets.end(),
              [](const Det& a, const Det& b) { return a.score > b.score; });
    std::vector<char> removed(dets.size(), 0);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!removed[j] && IoU(dets[i], dets[j]) > iou_thresh)
                removed[j] = 1;
        }
    }
    return keep;
}

// ---------------------------------------------------------------------------
// Letterbox transforms
// ---------------------------------------------------------------------------

/// Nearest-neighbour letterbox for CHW float buffers.
/// Resizes (C, Hs, Ws) -> (C, Hd, Wd) preserving aspect ratio,
/// padding with zeros.  Returns the resulting buffer and writes
/// back the computed `scale`, `padx`, `pady`.
inline std::vector<float> letterboxCHW(
    const std::vector<float>& src,
    int C, int Hs, int Ws,
    int Hd, int Wd,
    float& scale, int& padx, int& pady)
{
    std::vector<float> dst((size_t)C * Hd * Wd, 0.0f);
    scale = std::min((float)Wd / (float)Ws, (float)Hd / (float)Hs);
    const int newW = (int)std::round(Ws * scale);
    const int newH = (int)std::round(Hs * scale);
    padx = (Wd - newW) / 2;
    pady = (Hd - newH) / 2;

    for (int y = 0; y < newH; ++y) {
        int sy = std::min(Hs - 1, (int)std::floor(y / scale));
        for (int x = 0; x < newW; ++x) {
            int sx = std::min(Ws - 1, (int)std::floor(x / scale));
            for (int c = 0; c < C; ++c) {
                dst[(size_t)c * Hd * Wd + (size_t)(y + pady) * Wd + (x + padx)] =
                    src[(size_t)c * Hs * Ws + (size_t)sy * Ws + sx];
            }
        }
    }
    return dst;
}

/// Letterbox into a pre-allocated destination buffer (avoids allocation).
/// `dst` must already be sized to C * Hd * Wd and zero-filled by the caller.
inline void letterboxCHW_into(
    const float* src, int C, int Hs, int Ws,
    float* dst, int Hd, int Wd,
    float scale, int padx, int pady)
{
    for (int y = 0; y < (int)std::round(Hs * scale); ++y) {
        int sy = std::min(Hs - 1, (int)std::floor(y / scale));
        for (int x = 0; x < (int)std::round(Ws * scale); ++x) {
            int sx = std::min(Ws - 1, (int)std::floor(x / scale));
            for (int c = 0; c < C; ++c) {
                dst[(size_t)c * Hd * Wd + (size_t)(y + pady) * Wd + (x + padx)] =
                    src[(size_t)c * Hs * Ws + (size_t)sy * Ws + sx];
            }
        }
    }
}

/// Compute letterbox scale and padding without actually transforming data.
inline void letterboxParams(
    int Ws, int Hs, int Wd, int Hd,
    float& scale, int& padx, int& pady)
{
    scale = std::min((float)Wd / (float)Ws, (float)Hd / (float)Hs);
    const int newW = (int)std::round(Ws * scale);
    const int newH = (int)std::round(Hs * scale);
    padx = (Wd - newW) / 2;
    pady = (Hd - newH) / 2;
}

/// Unletterbox coordinates from model space back to sensor space.
/// Returns a clamped ofRectangle in sensor pixels.
inline ofRectangle unletterboxToSensor(
    float x1, float y1, float x2, float y2,
    float scale, int padx, int pady,
    int sensorW, int sensorH)
{
    float sx1 = (x1 - padx) / scale;
    float sy1 = (y1 - pady) / scale;
    float sx2 = (x2 - padx) / scale;
    float sy2 = (y2 - pady) / scale;

    sx1 = std::max(0.f, std::min((float)sensorW, sx1));
    sy1 = std::max(0.f, std::min((float)sensorH, sy1));
    sx2 = std::max(0.f, std::min((float)sensorW, sx2));
    sy2 = std::max(0.f, std::min((float)sensorH, sy2));

    return ofRectangle(sx1, sy1, sx2 - sx1, sy2 - sy1);
}

/// IoU for two ofRectangle objects (screen-space or sensor-space).
inline float rectIoU(const ofRectangle& A, const ofRectangle& B) {
    float ax1 = A.getX(), ay1 = A.getY();
    float ax2 = ax1 + A.getWidth(), ay2 = ay1 + A.getHeight();
    float bx1 = B.getX(), by1 = B.getY();
    float bx2 = bx1 + B.getWidth(), by2 = by1 + B.getHeight();

    float x1 = std::max(ax1, bx1), y1 = std::max(ay1, by1);
    float x2 = std::min(ax2, bx2), y2 = std::min(ay2, by2);
    float w = std::max(0.f, x2 - x1), h = std::max(0.f, y2 - y1);
    float inter = w * h;
    float aA = std::max(0.f, A.getWidth()) * std::max(0.f, A.getHeight());
    float aB = std::max(0.f, B.getWidth()) * std::max(0.f, B.getHeight());
    float denom = aA + aB - inter;
    return denom > 0.f ? inter / denom : 0.f;
}

}} // namespace dvs::nn

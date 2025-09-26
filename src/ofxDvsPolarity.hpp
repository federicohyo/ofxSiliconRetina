#pragma once

#include <cstdint>

// NOTE: this is also part of the binary file layout,
//  changing this may break backwards compatibility with previous recordings.
struct ofxDvsPolarity {
    int64_t             timestamp;  // us resolution
    uint16_t            x;
    uint16_t            y;
    bool                polarity;
};
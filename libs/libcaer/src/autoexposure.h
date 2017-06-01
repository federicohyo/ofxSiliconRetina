#ifndef LIBCAER_SRC_AUTOEXPOSURE_H_
#define LIBCAER_SRC_AUTOEXPOSURE_H_

#include "libcaer.h"
#include "events/frame.h"

#define AUTOEXPOSURE_ENABLE_DEBUG_LOGGING 0

#define AUTOEXPOSURE_HISTOGRAM_PIXELS 256
#define AUTOEXPOSURE_HISTOGRAM_MSV 5
#define AUTOEXPOSURE_LOW_BOUNDARY 0.10f
#define AUTOEXPOSURE_HIGH_BOUNDARY 0.90f
#define AUTOEXPOSURE_UNDEROVER_FRAC 0.33f
#define AUTOEXPOSURE_UNDEROVER_CORRECTION 14000.0f
#define AUTOEXPOSURE_MSV_CORRECTION 100.0f

struct auto_exposure_state {
	size_t pixelHistogram[AUTOEXPOSURE_HISTOGRAM_PIXELS];
	size_t msvHistogram[AUTOEXPOSURE_HISTOGRAM_MSV];
	uint32_t lastFrameExposureValue;
};

typedef struct auto_exposure_state *autoExposureState;

// Returns next exposure value in µs, or -1 if currently set is optimal.
int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureFrameValue, uint32_t exposureLastSetValue);

#endif /* LIBCAER_SRC_AUTOEXPOSURE_H_ */

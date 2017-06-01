#include "autoexposure.h"
#include <math.h>

static inline int32_t upAndClip(int32_t newExposure, int32_t lastExposure) {
	// Ensure increase.
	if (newExposure == lastExposure) {
		newExposure++;
	}

	// Clip exposure at maximum (1s = 1000000µs).
	if (newExposure > 1000000) {
		newExposure = 1000000;
	}

	return (newExposure);
}

static inline int32_t downAndClip(int32_t newExposure, int32_t lastExposure) {
	// Ensure decrease.
	if (newExposure == lastExposure) {
		newExposure--;
	}

	// Clip exposure at minimum (1µs).
	if (newExposure < 1) {
		newExposure = 1;
	}

	return (newExposure);
}

int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureFrameValue,
	uint32_t exposureLastSetValue) {
#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
	caerLog(CAER_LOG_ERROR, "AutoExposure", "Last set exposure value was: %d.", exposureLastSetValue);
	caerLog(CAER_LOG_ERROR, "AutoExposure", "Frame exposure value was: %d.", exposureFrameValue);
	caerLog(CAER_LOG_ERROR, "AutoExposure", "Real frame exposure value was: %d.",
		caerFrameEventGetExposureLength(frame));
#endif

	// Only run if the frame corresponds to the last set value.
	if (exposureFrameValue != exposureLastSetValue) {
		return (-1);
	}

	int32_t frameSizeX = caerFrameEventGetLengthX(frame);
	int32_t frameSizeY = caerFrameEventGetLengthY(frame);
	const uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafeConst(frame);

	// Reset histograms.
	memset(state->pixelHistogram, 0, AUTOEXPOSURE_HISTOGRAM_PIXELS * sizeof(size_t));
	memset(state->msvHistogram, 0, AUTOEXPOSURE_HISTOGRAM_MSV * sizeof(size_t));

	// Fill histograms: 256 regions for pixel values; 5 regions for MSV.
	for (int32_t y = 0; y < frameSizeY; y++) {
		for (int32_t x = 0; x < frameSizeX; x++) {
			uint16_t pixelValue = framePixels[(y * frameSizeX) + x];

			// Update histograms.
			size_t pixelIndex = pixelValue / ((UINT16_MAX + 1) / AUTOEXPOSURE_HISTOGRAM_PIXELS);
			state->pixelHistogram[pixelIndex]++;

			size_t msvIndex = pixelValue / ((UINT16_MAX + 1) / AUTOEXPOSURE_HISTOGRAM_MSV);
			state->msvHistogram[msvIndex] += pixelValue;
		}
	}

	// Calculate statistics on pixel histogram. Sum of histogram is always equal
	// to the number of pixels in the camera.
	size_t pixelsSum = (size_t) (frameSizeX * frameSizeY);

	size_t pixelsBinLow = (size_t) (AUTOEXPOSURE_LOW_BOUNDARY * (float) AUTOEXPOSURE_HISTOGRAM_PIXELS);
	size_t pixelsBinHigh = (size_t) (AUTOEXPOSURE_HIGH_BOUNDARY * (float) AUTOEXPOSURE_HISTOGRAM_PIXELS);

	size_t pixelsSumLow = 0, pixelsSumHigh = 0;

	for (size_t i = 0; i < pixelsBinLow; i++) {
		pixelsSumLow += state->pixelHistogram[i];
	}

	for (size_t i = pixelsBinHigh; i < AUTOEXPOSURE_HISTOGRAM_PIXELS; i++) {
		pixelsSumHigh += state->pixelHistogram[i];
	}

	float pixelsFracLow = (float) pixelsSumLow / (float) pixelsSum;
	float pixelsFracHigh = (float) pixelsSumHigh / (float) pixelsSum;

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
	caerLog(CAER_LOG_ERROR, "AutoExposure",
		"BinLow: %zu, BinHigh: %zu, Sum: %zu, SumLow: %zu, SumHigh: %zu, FracLow: %f, FracHigh: %f.", pixelsBinLow,
		pixelsBinHigh, pixelsSum, pixelsSumLow, pixelsSumHigh, (double) pixelsFracLow, (double) pixelsFracHigh);
#endif

	float fracLowError = pixelsFracLow - AUTOEXPOSURE_UNDEROVER_FRAC;
	float fracHighError = pixelsFracHigh - AUTOEXPOSURE_UNDEROVER_FRAC;

	// Exposure okay by default.
	int32_t newExposure = -1;

	if ((pixelsFracLow >= AUTOEXPOSURE_UNDEROVER_FRAC) && (pixelsFracHigh < AUTOEXPOSURE_UNDEROVER_FRAC)) {
		// Underexposed but not overexposed.
		newExposure = I32T(
			exposureLastSetValue) + I32T(AUTOEXPOSURE_UNDEROVER_CORRECTION * powf(fracLowError, 1.65f));

		newExposure = upAndClip(newExposure, I32T(exposureLastSetValue));
	}
	else if ((pixelsFracHigh >= AUTOEXPOSURE_UNDEROVER_FRAC) && (pixelsFracLow < AUTOEXPOSURE_UNDEROVER_FRAC)) {
		// Overexposed but not underexposed.
		newExposure = I32T(
			exposureLastSetValue) - I32T(AUTOEXPOSURE_UNDEROVER_CORRECTION * powf(fracHighError, 1.65f));

		newExposure = downAndClip(newExposure, I32T(exposureLastSetValue));
	}
	else {
		// Calculate mean sample value from histogram.
		float meanSampleValueNum = 0, meanSampleValueDenom = 0;

		for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_MSV; i++) {
			meanSampleValueNum += ((float) i + 1.0f) * (float) state->msvHistogram[i];
			meanSampleValueDenom += (float) state->msvHistogram[i];
		}

		// Prevent division by zero.
		if (meanSampleValueDenom == 0) {
			meanSampleValueDenom = 1.0f;
		}

		float meanSampleValue = meanSampleValueNum / meanSampleValueDenom;
		float meanSampleValueError = (AUTOEXPOSURE_HISTOGRAM_MSV / 2.0f) - meanSampleValue;

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
		caerLog(CAER_LOG_ERROR, "AutoExposure", "Mean sample value error is: %f.", (double) meanSampleValueError);
#endif

		// If we're close to the under/over limits, we make the magnitude of changes smaller
		// to avoid back&forth oscillations.
		int32_t divisor = 1;
		if (fabsf(fracLowError) < 0.1f || fabsf(fracHighError) < 0.1f) {
			divisor = 5;
		}
		if (fabsf(fracLowError) < 0.05f || fabsf(fracHighError) < 0.05f) {
			divisor = 10;
		}

		// If we're not too underexposed or overexposed, use MSV to optimize.
		if (meanSampleValueError > 0.1f) {
			// Underexposed.
			newExposure = I32T(exposureLastSetValue)
				+ (I32T(AUTOEXPOSURE_MSV_CORRECTION * powf(meanSampleValueError, 2.0f)) / divisor);

			newExposure = upAndClip(newExposure, I32T(exposureLastSetValue));
		}
		else if (meanSampleValueError < -0.1f) {
			// Overexposed.
			newExposure = I32T(exposureLastSetValue)
				- (I32T(AUTOEXPOSURE_MSV_CORRECTION * powf(meanSampleValueError, 2.0f)) / divisor);

			newExposure = downAndClip(newExposure, I32T(exposureLastSetValue));
		}
	}

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
	caerLog(CAER_LOG_ERROR, "AutoExposure", "New exposure value is: %d.", newExposure);
#endif

	return ((newExposure == I32T(exposureLastSetValue)) ? (-1) : (newExposure));
}

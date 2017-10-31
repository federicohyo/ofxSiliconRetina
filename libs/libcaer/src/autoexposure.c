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

int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frames[DAVIS_APS_ROI_REGIONS_MAX],
	uint32_t exposureFrameValue, uint32_t exposureLastSetValue) {
#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
	caerLog(CAER_LOG_INFO, "AutoExposure", "Last set exposure value was: %d.", exposureLastSetValue);
	caerLog(CAER_LOG_INFO, "AutoExposure", "Frame exposure value was: %d.", exposureFrameValue);
#endif

	// Only run if the frame corresponds to the last set value.
	if (exposureFrameValue != exposureLastSetValue) {
		return (-1);
	}

	int64_t newExposureTotal = 0;
	int32_t activeRoiRegions = 0;

	for (size_t idx = 0; idx < DAVIS_APS_ROI_REGIONS_MAX; idx++) {
		// Skip disabled APS ROI regions.
		if (frames[idx] == NULL) {
			continue;
		}

		// Count enabled APS ROI regions.
		activeRoiRegions++;

		int32_t frameSizeX = caerFrameEventGetLengthX(frames[idx]);
		int32_t frameSizeY = caerFrameEventGetLengthY(frames[idx]);
		const uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafeConst(frames[idx]);

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

		size_t pixelsSumLow = 0;
		size_t pixelsSumHigh = 0;

		for (size_t i = 0; i < pixelsBinLow; i++) {
			pixelsSumLow += state->pixelHistogram[i];
		}

		for (size_t i = pixelsBinHigh; i < AUTOEXPOSURE_HISTOGRAM_PIXELS; i++) {
			pixelsSumHigh += state->pixelHistogram[i];
		}

		float pixelsFracLow = (float) pixelsSumLow / (float) pixelsSum;
		float pixelsFracHigh = (float) pixelsSumHigh / (float) pixelsSum;

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
		caerLog(CAER_LOG_INFO, "AutoExposure",
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
				exposureLastSetValue) + I32T(AUTOEXPOSURE_UNDEROVER_CORRECTION * powf(fracLowError, 1.65F));

			newExposure = upAndClip(newExposure, I32T(exposureLastSetValue));
		}
		else if ((pixelsFracHigh >= AUTOEXPOSURE_UNDEROVER_FRAC) && (pixelsFracLow < AUTOEXPOSURE_UNDEROVER_FRAC)) {
			// Overexposed but not underexposed.
			newExposure = I32T(
				exposureLastSetValue) - I32T(AUTOEXPOSURE_UNDEROVER_CORRECTION * powf(fracHighError, 1.65F));

			newExposure = downAndClip(newExposure, I32T(exposureLastSetValue));
		}
		else {
			// Calculate mean sample value from histogram.
			float meanSampleValueNum = 0;
			float meanSampleValueDenom = 0;

			for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_MSV; i++) {
				meanSampleValueNum += ((float) i + 1.0F) * (float) state->msvHistogram[i];
				meanSampleValueDenom += (float) state->msvHistogram[i];
			}

			// Prevent division by zero.
			if (meanSampleValueDenom == 0) {
				meanSampleValueDenom = 1.0F;
			}

			float meanSampleValue = meanSampleValueNum / meanSampleValueDenom;
			float meanSampleValueError = (AUTOEXPOSURE_HISTOGRAM_MSV / 2.0F) - meanSampleValue;

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
			caerLog(CAER_LOG_INFO, "AutoExposure", "Mean sample value error is: %f.", (double) meanSampleValueError);
#endif

			// If we're close to the under/over limits, we make the magnitude of changes smaller
			// to avoid back&forth oscillations.
			int32_t divisor = 1;
			if ((fabsf(fracLowError) < 0.1F) || (fabsf(fracHighError) < 0.1F)) {
				divisor = 5;
			}
			if ((fabsf(fracLowError) < 0.05F) || (fabsf(fracHighError) < 0.05F)) {
				divisor = 10;
			}

			// If we're not too underexposed or overexposed, use MSV to optimize.
			if (meanSampleValueError > 0.1F) {
				// Underexposed.
				newExposure = I32T(exposureLastSetValue)
					+ (I32T(AUTOEXPOSURE_MSV_CORRECTION * powf(meanSampleValueError, 2.0F)) / divisor);

				newExposure = upAndClip(newExposure, I32T(exposureLastSetValue));
			}
			else if (meanSampleValueError < -0.1F) {
				// Overexposed.
				newExposure = I32T(exposureLastSetValue)
					- (I32T(AUTOEXPOSURE_MSV_CORRECTION * powf(meanSampleValueError, 2.0F)) / divisor);

				newExposure = downAndClip(newExposure, I32T(exposureLastSetValue));
			}
		}

		// Add new exposure value calculated for this ROI region to total.
		newExposureTotal += newExposure;
	}

	// No active APS ROI regions, nothing to analyze exposure on, so no change.
	if (activeRoiRegions == 0) {
		return (-1);
	}

	// Divide total by active ROI regions to get a mean.
	newExposureTotal /= activeRoiRegions;

#if AUTOEXPOSURE_ENABLE_DEBUG_LOGGING == 1
		caerLog(CAER_LOG_INFO, "AutoExposure", "New exposure value is: %" PRIi64 ".", newExposureTotal);
#endif

	return ((I32T(newExposureTotal) == I32T(exposureLastSetValue)) ? (-1) : I32T(newExposureTotal));
}

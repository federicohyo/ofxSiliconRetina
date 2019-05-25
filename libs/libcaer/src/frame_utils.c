#include "frame_utils.h"

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
// Use C++ OpenCV demosaic and contrast functions, defined
// separately in 'frame_utils_opencv.cpp'.
extern void caerFrameUtilsOpenCVDemosaic(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_demosaic_types demosaicType);
extern void caerFrameUtilsOpenCVContrast(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_contrast_types contrastType);
#endif

enum pixelColorEnum { PXR, PXB, PXG1, PXG2, PXW };

static const enum pixelColorEnum colorKeys[9][4] = {
	[MONO] = {PXR, PXR, PXR, PXR}, // This is impossible (MONO), so just use red pixel value, as good as any.
	[RGBG] = {PXR, PXG2, PXG1, PXB},
	[GRGB] = {PXG1, PXB, PXR, PXG2},
	[GBGR] = {PXG2, PXR, PXB, PXG1},
	[BGRG] = {PXB, PXG1, PXG2, PXR},
	[RGBW] = {PXR, PXW, PXG1, PXB},
	[GRWB] = {PXG1, PXB, PXR, PXW},
	[WBGR] = {PXW, PXR, PXB, PXG1},
	[BWRG] = {PXB, PXG1, PXW, PXR},
};

void caerFrameUtilsDemosaic(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_demosaic_types demosaicType) {
	if ((inputFrame == NULL) || (outputFrame == NULL)) {
		return;
	}

	if (caerFrameEventGetChannelNumber(inputFrame) != GRAYSCALE) {
		caerLog(CAER_LOG_ERROR, __func__,
			"Demosaic is only possible on input frames with only one channel (intensity -> color).");
		return;
	}

	if (caerFrameEventGetColorFilter(inputFrame) == MONO) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic is only possible on input frames with a color filter present.");
		return;
	}

	const enum caer_frame_event_color_channels outputColorChannels = caerFrameEventGetChannelNumber(outputFrame);

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
	if ((demosaicType == DEMOSAIC_STANDARD || demosaicType == DEMOSAIC_OPENCV_STANDARD
			|| demosaicType == DEMOSAIC_OPENCV_EDGE_AWARE)
		&& outputColorChannels != RGB) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic to color requires output frame to be RGB.");
		return;
	}
	else if ((demosaicType == DEMOSAIC_TO_GRAY || demosaicType == DEMOSAIC_OPENCV_TO_GRAY)
			 && outputColorChannels != GRAYSCALE) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic to grayscale requires output frame to be GRAYSCALE.");
		return;
	}
#else
	if (demosaicType == DEMOSAIC_STANDARD && outputColorChannels != RGB) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic to color requires output frame to be RGB.");
		return;
	}
	else if (demosaicType == DEMOSAIC_TO_GRAY && outputColorChannels != GRAYSCALE) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic to grayscale requires output frame to be GRAYSCALE.");
		return;
	}
#endif

	if ((caerFrameEventGetLengthX(inputFrame) != caerFrameEventGetLengthX(outputFrame))
		|| (caerFrameEventGetLengthY(inputFrame) != caerFrameEventGetLengthY(outputFrame))) {
		caerLog(CAER_LOG_ERROR, __func__, "Demosaic only possible on compatible frames (equal X/Y lengths).");
		return;
	}

	if ((demosaicType != DEMOSAIC_STANDARD) && (demosaicType != DEMOSAIC_TO_GRAY)) {
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		caerFrameUtilsOpenCVDemosaic(inputFrame, outputFrame, demosaicType);
#else
		caerLog(CAER_LOG_ERROR, __func__,
			"Selected OpenCV demosaic type, but OpenCV support is disabled. Either "
			"enable it or change to use 'DEMOSAIC_STANDARD' or 'DEMOSAIC_TO_GRAY'.");
#endif

		return;
	}

	// Then the actual pixels.
	const uint16_t *inPixels = caerFrameEventGetPixelArrayUnsafeConst(inputFrame);
	uint16_t *outPixels      = caerFrameEventGetPixelArrayUnsafe(outputFrame);

	enum caer_frame_event_color_filter colorFilter = caerFrameEventGetColorFilter(inputFrame);
	int32_t lengthX                                = caerFrameEventGetLengthX(inputFrame);
	int32_t lengthY                                = caerFrameEventGetLengthY(inputFrame);
	int32_t idxCENTER                              = 0;
	int32_t idxOUTPUT                              = 0;

	for (int32_t y = 0; y < lengthY; y++) {
		for (int32_t x = 0; x < lengthX; x++) {
			// Calculate all neighbor indexes.
			int32_t idxLEFT  = idxCENTER - 1;
			int32_t idxRIGHT = idxCENTER + 1;

			int32_t idxCENTERUP = idxCENTER - lengthX;
			int32_t idxLEFTUP   = idxCENTERUP - 1;
			int32_t idxRIGHTUP  = idxCENTERUP + 1;

			int32_t idxCENTERDOWN = idxCENTER + lengthX;
			int32_t idxLEFTDOWN   = idxCENTERDOWN - 1;
			int32_t idxRIGHTDOWN  = idxCENTERDOWN + 1;

			enum pixelColorEnum pixelColor = colorKeys[colorFilter][((x & 0x01) << 1) | (y & 0x01)];
			int32_t RComp;
			int32_t GComp;
			int32_t BComp;

			switch (pixelColor) {
				case PXR: {
					// This is a R pixel. It is always surrounded by G and B only.
					RComp = inPixels[idxCENTER];

					if (y == 0) {
						// First row.
						if (x == 0) {
							// First column.
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxRIGHT]) / 2;
							BComp = inPixels[idxRIGHTDOWN];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxLEFT]) / 2;
							BComp = inPixels[idxLEFTDOWN];
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxLEFT] + inPixels[idxRIGHT]) / 3;
							BComp = (inPixels[idxRIGHTDOWN] + inPixels[idxLEFTDOWN]) / 2;
						}
					}
					else if (y == (lengthY - 1)) {
						// Last row.
						if (x == 0) {
							// First column.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxRIGHT]) / 2;
							BComp = inPixels[idxRIGHTUP];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxLEFT]) / 2;
							BComp = inPixels[idxLEFTUP];
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxLEFT] + inPixels[idxRIGHT]) / 3;
							BComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP]) / 2;
						}
					}
					else {
						// In-between rows.
						if (x == 0) {
							// First column.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxRIGHT]) / 3;
							BComp = (inPixels[idxRIGHTUP] + inPixels[idxRIGHTDOWN]) / 2;
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxLEFT]) / 3;
							BComp = (inPixels[idxLEFTUP] + inPixels[idxLEFTDOWN]) / 2;
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxLEFT]
										+ inPixels[idxRIGHT])
									/ 4;
							BComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP] + inPixels[idxRIGHTDOWN]
										+ inPixels[idxLEFTDOWN])
									/ 4;
						}
					}

					break;
				}

				case PXB: {
					// This is a B pixel. It is always surrounded by G and R only.
					BComp = inPixels[idxCENTER];

					if (y == 0) {
						// First row.
						if (x == 0) {
							// First column.
							RComp = inPixels[idxRIGHTDOWN];
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxRIGHT]) / 2;
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = inPixels[idxLEFTDOWN];
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxLEFT]) / 2;
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxRIGHTDOWN] + inPixels[idxLEFTDOWN]) / 2;
							GComp = (inPixels[idxCENTERDOWN] + inPixels[idxLEFT] + inPixels[idxRIGHT]) / 3;
						}
					}
					else if (y == (lengthY - 1)) {
						// Last row.
						if (x == 0) {
							// First column.
							RComp = inPixels[idxRIGHTUP];
							GComp = (inPixels[idxCENTERUP] + inPixels[idxRIGHT]) / 2;
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = inPixels[idxLEFTUP];
							GComp = (inPixels[idxCENTERUP] + inPixels[idxLEFT]) / 2;
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP]) / 2;
							GComp = (inPixels[idxCENTERUP] + inPixels[idxLEFT] + inPixels[idxRIGHT]) / 3;
						}
					}
					else {
						// In-between rows.
						if (x == 0) {
							// First column.
							RComp = (inPixels[idxRIGHTUP] + inPixels[idxRIGHTDOWN]) / 2;
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxRIGHT]) / 3;
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = (inPixels[idxLEFTUP] + inPixels[idxLEFTDOWN]) / 2;
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxLEFT]) / 3;
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP] + inPixels[idxRIGHTDOWN]
										+ inPixels[idxLEFTDOWN])
									/ 4;
							GComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN] + inPixels[idxLEFT]
										+ inPixels[idxRIGHT])
									/ 4;
						}
					}

					break;
				}

				case PXG1: {
					// This is a G1 (first green) pixel. It is always surrounded by all of R, G, B.
					GComp = inPixels[idxCENTER];

					if (y == 0) {
						// First row.
						BComp = inPixels[idxCENTERDOWN];

						if (x == 0) {
							// First column.
							RComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else if (y == (lengthY - 1)) {
						// Last row.
						BComp = inPixels[idxCENTERUP];

						if (x == 0) {
							// First column.
							RComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else {
						// In-between rows.
						BComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN]) / 2;

						if (x == 0) {
							// First column.
							RComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							RComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							RComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}

					break;
				}

				case PXG2: {
					// This is a G2 (second green) pixel. It is always surrounded by all of R, G, B.
					GComp = inPixels[idxCENTER];

					if (y == 0) {
						// First row.
						RComp = inPixels[idxCENTERDOWN];

						if (x == 0) {
							// First column.
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							BComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else if (y == (lengthY - 1)) {
						// Last row.
						RComp = inPixels[idxCENTERUP];

						if (x == 0) {
							// First column.
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							BComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else {
						// In-between rows.
						RComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN]) / 2;

						if (x == 0) {
							// First column.
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							BComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}

					break;
				}

				case PXW: {
					// This is a W pixel, modified Bayer pattern instead of G2.
					// It is always surrounded by all of R, G, B.
					// TODO: how can W itself contribute to the three colors?
					if (y == 0) {
						// First row.
						RComp = inPixels[idxCENTERDOWN];

						if (x == 0) {
							// First column.
							GComp = inPixels[idxRIGHTDOWN];
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = inPixels[idxLEFTDOWN];
							BComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxRIGHTDOWN] + inPixels[idxLEFTDOWN]) / 2;
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else if (y == (lengthY - 1)) {
						// Last row.
						RComp = inPixels[idxCENTERUP];

						if (x == 0) {
							// First column.
							GComp = inPixels[idxRIGHTUP];
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = inPixels[idxLEFTUP];
							BComp = inPixels[idxRIGHT];
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP]) / 2;
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}
					else {
						// In-between rows.
						RComp = (inPixels[idxCENTERUP] + inPixels[idxCENTERDOWN]) / 2;

						if (x == 0) {
							// First column.
							GComp = (inPixels[idxRIGHTUP] + inPixels[idxRIGHTDOWN]) / 2;
							BComp = inPixels[idxRIGHT];
						}
						else if (x == (lengthX - 1)) {
							// Last column.
							GComp = (inPixels[idxLEFTUP] + inPixels[idxLEFTDOWN]) / 2;
							BComp = inPixels[idxLEFT];
						}
						else {
							// In-between columns.
							GComp = (inPixels[idxRIGHTUP] + inPixels[idxLEFTUP] + inPixels[idxRIGHTDOWN]
										+ inPixels[idxLEFTDOWN])
									/ 4;
							BComp = (inPixels[idxLEFT] + inPixels[idxRIGHT]) / 2;
						}
					}

					break;
				}

				default:
					// Do nothing, all colors are examined above.
					break;
			}

			if (outputColorChannels == GRAYSCALE) {
				// Set output frame pixel value for grayscale channel.
				outPixels[idxOUTPUT] = U16T((RComp + GComp + BComp) / 3);

				// Go to next pixel.
				idxCENTER++;
				idxOUTPUT += GRAYSCALE;
			}
			else {
				// Set output frame pixel values for all color channels.
				outPixels[idxOUTPUT]     = U16T(RComp);
				outPixels[idxOUTPUT + 1] = U16T(GComp);
				outPixels[idxOUTPUT + 2] = U16T(BComp);

				// Go to next pixel.
				idxCENTER++;
				idxOUTPUT += RGB;
			}
		}
	}
}

void caerFrameUtilsContrast(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_contrast_types contrastType) {
	if ((inputFrame == NULL) || (outputFrame == NULL)) {
		return;
	}

	if ((caerFrameEventGetChannelNumber(inputFrame) != caerFrameEventGetChannelNumber(outputFrame))
		|| (caerFrameEventGetLengthX(inputFrame) != caerFrameEventGetLengthX(outputFrame))
		|| (caerFrameEventGetLengthY(inputFrame) != caerFrameEventGetLengthY(outputFrame))) {
		caerLog(CAER_LOG_ERROR, __func__,
			"Contrast enhancement only possible on compatible frames (same number of "
			"color channels and equal X/Y lengths).");
		return;
	}

	if (contrastType != CONTRAST_STANDARD) {
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		caerFrameUtilsOpenCVContrast(inputFrame, outputFrame, contrastType);
#else
		caerLog(CAER_LOG_ERROR, __func__,
			"Selected OpenCV contrast enhancement type, but OpenCV support is "
			"disabled. Either enable it or change to use 'CONTRAST_STANDARD'.");
#endif

		return;
	}

	if (caerFrameEventGetChannelNumber(inputFrame) != GRAYSCALE) {
		caerLog(CAER_LOG_ERROR, __func__,
			"Standard contrast enhancement only works with grayscale images. For color "
			"images support, please use one of the OpenCV contrast enhancement types.");
		return;
	}

	// O(x, y) = alpha * I(x, y) + beta, where alpha maximizes the range
	// (contrast) and beta shifts it so lowest is zero (brightness).
	// Only works with grayscale images currently. Doing so for color (RGB/RGBA) images would require
	// conversion into another color space that has an intensity channel separate from the color
	// channels, such as Lab or YCrCb. The same algorithm would then be applied on the intensity only.
	const uint16_t *inPixels = caerFrameEventGetPixelArrayUnsafeConst(inputFrame);
	uint16_t *outPixels      = caerFrameEventGetPixelArrayUnsafe(outputFrame);

	size_t pixelsSize = caerFrameEventGetPixelsMaxIndex(inputFrame);

	// On first pass, determine minimum and maximum values.
	int32_t minValue = INT32_MAX;
	int32_t maxValue = INT32_MIN;

	for (size_t idx = 0; idx < pixelsSize; idx++) {
		if (inPixels[idx] < minValue) {
			minValue = inPixels[idx];
		}

		if (inPixels[idx] > maxValue) {
			maxValue = inPixels[idx];
		}
	}

	// Use min/max to calculate input range.
	int32_t range = maxValue - minValue;

	// Calculate alpha (contrast).
	float alpha = ((float) UINT16_MAX) / ((float) range);

	// Calculate beta (brightness).
	float beta = ((float) -minValue) * alpha;

	// Apply alpha and beta to pixels array.
	for (size_t idx = 0; idx < pixelsSize; idx++) {
		outPixels[idx] = U16T(alpha * ((float) inPixels[idx]) + beta);
	}
}

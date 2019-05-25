#include "frame_utils.h"

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
void caerFrameUtilsOpenCVDemosaic(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_demosaic_types demosaicType);
void caerFrameUtilsOpenCVContrast(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_contrast_types contrastType);
}

static void frameUtilsOpenCVContrastNormalize(const cv::Mat &input, cv::Mat &output, float clipHistPercent);
static void frameUtilsOpenCVContrastEqualize(const cv::Mat &input, cv::Mat &output);
static void frameUtilsOpenCVContrastCLAHE(const cv::Mat &input, cv::Mat &output, float clipLimit, int tilesGridSize);

void caerFrameUtilsOpenCVDemosaic(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_demosaic_types demosaicType) {
	const enum caer_frame_event_color_filter colorFilter = caerFrameEventGetColorFilter(inputFrame);

	if ((colorFilter == RGBW) || (colorFilter == GRWB) || (colorFilter == BWRG) || (colorFilter == WBGR)) {
		caerLog(CAER_LOG_WARNING, __func__,
			"OpenCV demosaic types don't support the RGBW color filter variants, only RGBG. "
			"Please use the 'DEMOSAIC_STANDARD' or 'DEMOSAIC_TO_GRAY' types for RGBW sensors.");
		return;
	}

	// Demosaic the actual pixels. Only supports RGBG!
	// Initialize OpenCV cv::Mat based on caerFrameEvent data directly (no image copy).
	const cv::Size frameSize(caerFrameEventGetLengthX(inputFrame), caerFrameEventGetLengthY(inputFrame));
	const cv::Mat inputMat(frameSize, CV_16UC(caerFrameEventGetChannelNumber(inputFrame)),
		const_cast<uint16_t *>(caerFrameEventGetPixelArrayUnsafeConst(inputFrame)));
	cv::Mat outputMat(frameSize, CV_16UC(caerFrameEventGetChannelNumber(outputFrame)),
		caerFrameEventGetPixelArrayUnsafe(outputFrame));

	CV_Assert((inputMat.type() == CV_16UC1) && ((outputMat.type() == CV_16UC1) || (outputMat.type() == CV_16UC3)));

	// Select correct type code for OpenCV demosaic algorithm.
	int code = 0;

	// NOTE: DEMOSAIC_OPENCV_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16 bit images.
	switch (demosaicType) {
		case DEMOSAIC_OPENCV_STANDARD:
			switch (colorFilter) {
				case RGBG:
					code = cv::COLOR_BayerBG2RGB;
					break;

				case GRGB:
					code = cv::COLOR_BayerGB2RGB;
					break;

				case GBGR:
					code = cv::COLOR_BayerGR2RGB;
					break;

				case BGRG:
					code = cv::COLOR_BayerRG2RGB;
					break;

				case MONO:
				case RGBW:
				case GRWB:
				case WBGR:
				case BWRG:
				default:
					// Impossible, other color filters get filtered out above.
					break;
			}
			break;

		case DEMOSAIC_OPENCV_EDGE_AWARE:
			switch (colorFilter) {
				case RGBG:
					code = cv::COLOR_BayerBG2RGB_EA;
					break;

				case GRGB:
					code = cv::COLOR_BayerGB2RGB_EA;
					break;

				case GBGR:
					code = cv::COLOR_BayerGR2RGB_EA;
					break;

				case BGRG:
					code = cv::COLOR_BayerRG2RGB_EA;
					break;

				case MONO:
				case RGBW:
				case GRWB:
				case WBGR:
				case BWRG:
				default:
					// Impossible, other color filters get filtered out above.
					break;
			}
			break;

		case DEMOSAIC_OPENCV_TO_GRAY:
			switch (colorFilter) {
				case RGBG:
					code = cv::COLOR_BayerBG2GRAY;
					break;

				case GRGB:
					code = cv::COLOR_BayerGB2GRAY;
					break;

				case GBGR:
					code = cv::COLOR_BayerGR2GRAY;
					break;

				case BGRG:
					code = cv::COLOR_BayerRG2GRAY;
					break;

				case MONO:
				case RGBW:
				case GRWB:
				case WBGR:
				case BWRG:
				default:
					// Impossible, other color filters get filtered out above.
					break;
			}
			break;

		case DEMOSAIC_STANDARD:
		case DEMOSAIC_TO_GRAY:
		default:
			// Impossible, other demosaic types are not available in OpenCV.
			break;
	}

	// Convert Bayer pattern to RGB or GRAYSCALE image.
	cvtColor(inputMat, outputMat, code);
}

static void frameUtilsOpenCVContrastNormalize(const cv::Mat &input, cv::Mat &output, float clipHistPercent) {
	CV_Assert((input.type() == CV_16UC1) && (output.type() == CV_16UC1));
	CV_Assert((clipHistPercent >= 0) && (clipHistPercent < 100));

	// O(x, y) = alpha * I(x, y) + beta, where alpha maximizes the range
	// (contrast) and beta shifts it so lowest is zero (brightness).
	double minValue;
	double maxValue;

	if (clipHistPercent == 0) {
		// Determine minimum and maximum values.
		minMaxLoc(input, &minValue, &maxValue);
	}
	else {
		// Calculate histogram.
		int histSize           = UINT16_MAX + 1;
		float hRange[]         = {0, (float) histSize};
		const float *histRange = {hRange};
		bool uniform           = true;
		bool accumulate        = false;

		cv::Mat hist;
		calcHist(&input, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

		// Calculate cumulative distribution from the histogram.
		for (int i = 1; i < histSize; i++) {
			hist.at<float>(i) += hist.at<float>(i - 1);
		}

		// Locate points that cut at required value.
		float total = hist.at<float>(histSize - 1);
		clipHistPercent *= (total / 100.0F); // Calculate absolute value from percent.
		clipHistPercent /= 2.0F;             // Left and right wings, so divide by two.

		// Locate left cut.
		minValue = 0;
		while (hist.at<float>((int) minValue) < clipHistPercent) {
			minValue++;
		}

		// Locate right cut.
		maxValue = UINT16_MAX;
		while (hist.at<float>((int) maxValue) >= (total - clipHistPercent)) {
			maxValue--;
		}
	}

	// Use min/max to calculate input range.
	double range = maxValue - minValue;

	// Calculate alpha (contrast).
	double alpha = ((double) UINT16_MAX) / range;

	// Calculate beta (brightness).
	double beta = -minValue * alpha;

	// Apply alpha and beta to pixels array.
	input.convertTo(output, -1, alpha, beta);
}

static void frameUtilsOpenCVContrastEqualize(const cv::Mat &input, cv::Mat &output) {
	CV_Assert((input.type() == CV_16UC1) && (output.type() == CV_16UC1));

	// Calculate histogram.
	int histSize           = UINT16_MAX + 1;
	float hRange[]         = {0, (float) histSize};
	const float *histRange = {hRange};
	bool uniform           = true;
	bool accumulate        = false;

	cv::Mat hist;
	calcHist(&input, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

	// Calculate cumulative distribution from the histogram.
	for (int i = 1; i < histSize; i++) {
		hist.at<float>(i) += hist.at<float>(i - 1);
	}

	// Total number of pixels. Must be the last value!
	float total = hist.at<float>(histSize - 1);

	// Smallest non-zero cumulative distribution value. Must be the first non-zero value!
	float min = 0;
	for (int i = 0; i < histSize; i++) {
		if (hist.at<float>(i) > 0) {
			min = hist.at<float>(i);
			break;
		}
	}

	// Calculate lookup table for histogram equalization.
	hist -= (double) min;
	hist /= (double) (total - min);
	hist *= (double) UINT16_MAX;

	// Apply lookup table to input image.
	int idx = 0;
	std::for_each(input.begin<uint16_t>(), input.end<uint16_t>(),
		[&hist, &output, &idx](const uint16_t &elem) { output.at<uint16_t>(idx++) = (uint16_t) hist.at<float>(elem); });
}

static void frameUtilsOpenCVContrastCLAHE(const cv::Mat &input, cv::Mat &output, float clipLimit, int tilesGridSize) {
	CV_Assert((input.type() == CV_16UC1) && (output.type() == CV_16UC1));
	CV_Assert((clipLimit >= 0) && (clipLimit < 100));
	CV_Assert((tilesGridSize >= 1) && (tilesGridSize <= 64));

	// Apply the CLAHE algorithm to the intensity channel (luminance).
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit((double) clipLimit);
	clahe->setTilesGridSize(cv::Size(tilesGridSize, tilesGridSize));
	clahe->apply(input, output);
}

void caerFrameUtilsOpenCVContrast(
	caerFrameEventConst inputFrame, caerFrameEvent outputFrame, enum caer_frame_utils_contrast_types contrastType) {
	const cv::Size frameSize(caerFrameEventGetLengthX(inputFrame), caerFrameEventGetLengthY(inputFrame));
	const cv::Mat input(frameSize, CV_16UC(caerFrameEventGetChannelNumber(inputFrame)),
		const_cast<uint16_t *>(caerFrameEventGetPixelArrayUnsafeConst(inputFrame)));
	cv::Mat output(frameSize, CV_16UC(caerFrameEventGetChannelNumber(outputFrame)),
		caerFrameEventGetPixelArrayUnsafe(outputFrame));

	CV_Assert((input.type() == CV_16UC1) || (input.type() == CV_16UC3) || (input.type() == CV_16UC4));
	CV_Assert((output.type() == CV_16UC1) || (output.type() == CV_16UC3) || (output.type() == CV_16UC4));

	CV_Assert(input.type() == output.type());
	CV_Assert(input.channels() == output.channels());

	// This generally only works well on grayscale intensity images.
	// So, if this is a grayscale image, good, else if its a color
	// image we convert it to YCrCb and operate on the Y channel only.
	cv::Mat intensity;
	cv::Mat yCrCbPlanes[3];
	cv::Mat rgbaAlpha;

	// Grayscale, no intensity extraction needed.
	if (input.channels() == GRAYSCALE) {
		intensity = input;
	}
	else {
		// Color image, extract RGB and intensity/luminance.
		cv::Mat rgb;

		if (input.channels() == RGBA) {
			// We separate Alpha from RGB first.
			// We will restore alpha at the end.
			rgb       = cv::Mat(input.rows, input.cols, CV_16UC3);
			rgbaAlpha = cv::Mat(input.rows, input.cols, CV_16UC1);

			cv::Mat out[] = {rgb, rgbaAlpha};
			// rgba[0] -> rgb[0], rgba[1] -> rgb[1],
			// rgba[2] -> rgb[2], rgba[3] -> rgbaAlpha[0]
			int channelTransform[] = {0, 0, 1, 1, 2, 2, 3, 3};
			mixChannels(&input, 1, out, 2, channelTransform, 4);
		}
		else {
			// Already an RGB image.
			rgb = input;
			CV_Assert(rgb.type() == CV_16UC3);
		}

		// First we convert from RGB to a color space with
		// separate luminance channel.
		cv::Mat rgbYCrCb;
		cvtColor(rgb, rgbYCrCb, cv::COLOR_RGB2YCrCb);

		CV_Assert(rgbYCrCb.type() == CV_16UC3);

		// Then we split it up so that we can access the luminance
		// channel on its own separately.
		split(rgbYCrCb, yCrCbPlanes);

		// Now we have the luminance image in yCrCbPlanes[0].
		intensity = yCrCbPlanes[0];
	}

	CV_Assert(intensity.type() == CV_16UC1);

	// Apply contrast enhancement algorithm.
	switch (contrastType) {
		case CONTRAST_OPENCV_NORMALIZATION:
			frameUtilsOpenCVContrastNormalize(intensity, output, 1.0);
			break;

		case CONTRAST_OPENCV_HISTOGRAM_EQUALIZATION:
			frameUtilsOpenCVContrastEqualize(intensity, output);
			break;

		case CONTRAST_OPENCV_CLAHE:
			frameUtilsOpenCVContrastCLAHE(intensity, output, 4.0, 8);
			break;

		case CONTRAST_STANDARD:
		default:
			// Other contrast enhancement types are not OpenCV.
			break;
	}

	// If original was a color frame, we have to mix the various
	// components back together into an RGB(A) image.
	if (output.channels() != GRAYSCALE) {
		cv::Mat YCrCbrgb;
		merge(yCrCbPlanes, 3, YCrCbrgb);

		CV_Assert(YCrCbrgb.type() == CV_16UC3);

		if (output.channels() == RGBA) {
			cv::Mat rgb;
			cvtColor(YCrCbrgb, rgb, cv::COLOR_YCrCb2RGB);

			CV_Assert(rgb.type() == CV_16UC3);

			// Restore original alpha.
			cv::Mat in[] = {rgb, rgbaAlpha};
			// rgb[0] -> rgba[0], rgb[1] -> rgba[1],
			// rgb[2] -> rgba[2], rgbaAlpha[0] -> rgba[3]
			int channelTransform[] = {0, 0, 1, 1, 2, 2, 3, 3};
			mixChannels(in, 2, &output, 1, channelTransform, 4);
		}
		else {
			cvtColor(YCrCbrgb, output, cv::COLOR_YCrCb2RGB);
		}
	}
}

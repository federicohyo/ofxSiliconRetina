/**
 * @file frame_utils.h
 *
 * Functions for frame enhancement and demosaicing. Basic variants
 * that don't require any external dependencies, such as OpenCV.
 * Use of the OpenCV variants is recommended for quality and performance,
 * and can optionally be enabled at build-time.
 */

#ifndef LIBCAER_FRAME_UTILS_H_
#define LIBCAER_FRAME_UTILS_H_

#include "events/frame.h"

#ifdef __cplusplus
extern "C" {
#endif

caerFrameEventPacket caerFrameUtilsDemosaic(caerFrameEventPacketConst framePacket);
void caerFrameUtilsContrast(caerFrameEventPacket framePacket);

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1

// DEMOSAIC_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
enum caer_frame_utils_opencv_demosaic {
	DEMOSAIC_NORMAL = 0,
	DEMOSAIC_EDGE_AWARE = 1,
};

enum caer_frame_utils_opencv_contrast {
	CONTRAST_NORMALIZATION = 0,
	CONTRAST_HISTOGRAM_EQUALIZATION = 1,
	CONTRAST_CLAHE = 2,
};

caerFrameEventPacket caerFrameUtilsOpenCVDemosaic(caerFrameEventPacketConst framePacket,
	enum caer_frame_utils_opencv_demosaic demosaicType);
void caerFrameUtilsOpenCVContrast(caerFrameEventPacket framePacket, enum caer_frame_utils_opencv_contrast contrastType);

#endif

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FRAME_UTILS_H_ */

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

enum caer_frame_utils_demosaic_types {
	DEMOSAIC_STANDARD = 0,
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
	DEMOSAIC_OPENCV_NORMAL = 1,
	DEMOSAIC_OPENCV_EDGE_AWARE = 2,
	// DEMOSAIC_OPENCV_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
#endif
};

enum caer_frame_utils_contrast_types {
	CONTRAST_STANDARD = 0,
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
	CONTRAST_OPENCV_NORMALIZATION = 1,
	CONTRAST_OPENCV_HISTOGRAM_EQUALIZATION = 2,
	CONTRAST_OPENCV_CLAHE = 3,
#endif
};

caerFrameEventPacket caerFrameUtilsDemosaic(caerFrameEventPacketConst framePacket,
	enum caer_frame_utils_demosaic_types demosaicType);
void caerFrameUtilsContrast(caerFrameEventPacket framePacket, enum caer_frame_utils_contrast_types contrastType);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FRAME_UTILS_H_ */

/**
 * @file dvs_noise.h
 *
 * The DVS noise filter combines a HotPixel filter (high activity pixels),
 * a Background-Activity filter (uncorrelated events), and a
 * Refractory Period filter (limit event rate of a pixel).
 * The HotPixel and Background-Activity filters reduce noise due
 * to transistor mismatch, the Refractory Period filter can reduce
 * the event rate and is efficient to implement together with the
 * Background-Activity filter, requiring only one pixel memory
 * map for both.
 * Please note that the filter is not thread-safe, all function calls
 * should happen on the same thread, unless you take care that they
 * never overlap.
 */

#ifndef LIBCAER_FILTERS_DVS_NOISE_H_
#define LIBCAER_FILTERS_DVS_NOISE_H_

#include "../events/polarity.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure representing a single DVS pixel address,
 * with X and Y components.
 * Used in DVS filtering support.
 */
struct caer_filter_dvs_pixel {
	uint16_t x;
	uint16_t y;
};

/**
 * Pointer to DVS pixel address structure.
 */
typedef struct caer_filter_dvs_pixel *caerFilterDVSPixel;

/**
 * Pointer to DVS noise filter structure (private).
 */
typedef struct caer_filter_dvs_noise *caerFilterDVSNoise;

/**
 * Allocate memory and initialize the DVS noise filter.
 * This filter combines a HotPixel filter (high activity pixels),
 * a Background-Activity filter (uncorrelated events), and a
 * Refractory Period filter (limit event rate of a pixel).
 * The HotPixel and Background-Activity filters reduce noise due
 * to transistor mismatch, the Refractory Period filter can reduce
 * the event rate and is efficient to implement together with the
 * Background-Activity filter, requiring only one pixel memory
 * map for both.
 * At initialization, all filters are disabled. You must configure
 * and enable them using caerFilterDVSNoiseConfigSet().
 * You must specify the maximum resolution at initialization,
 * as it is used to set up efficient lookup tables.
 *
 * @param sizeX maximum X axis resolution.
 * @param sizeY maximum Y axis resolution.
 *
 * @return DVS noise filter instance, NULL on error.
 */
LIBRARY_PUBLIC_VISIBILITY caerFilterDVSNoise caerFilterDVSNoiseInitialize(uint16_t sizeX, uint16_t sizeY);

/**
 * Destroy a DVS noise filter instance and free its memory.
 *
 * @param noiseFilter a valid DVS noise filter instance.
 */
LIBRARY_PUBLIC_VISIBILITY void caerFilterDVSNoiseDestroy(caerFilterDVSNoise noiseFilter);

/**
 * Apply the DVS noise filter to the given polarity events packet.
 * This will filter out events by marking them as invalid, depending
 * on the given filter configuration.
 *
 * @param noiseFilter a valid DVS noise filter instance.
 * @param polarity a valid polarity event packet. If NULL, no operation
 *                 is performed.
 */
LIBRARY_PUBLIC_VISIBILITY void caerFilterDVSNoiseApply(
	caerFilterDVSNoise noiseFilter, caerPolarityEventPacket polarity);

/**
 * Apply the DVS noise filter to the given polarity events packet.
 * This will only gather statistics on the noise, without changing the
 * event packet at all!
 *
 * @param noiseFilter a valid DVS noise filter instance.
 * @param polarity a valid polarity event packet. If NULL, no operation
 *                 is performed.
 */
LIBRARY_PUBLIC_VISIBILITY void caerFilterDVSNoiseStatsApply(
	caerFilterDVSNoise noiseFilter, caerPolarityEventPacketConst polarity);

/**
 * Set DVS noise filter configuration parameters.
 *
 * @param noiseFilter a valid DVS noise filter instance.
 * @param paramAddr a configuration parameter address, see defines CAER_FILTER_DVS_*.
 * @param param a configuration parameter value integer.
 *
 * @return true if operation successful, false otherwise.
 */
LIBRARY_PUBLIC_VISIBILITY bool caerFilterDVSNoiseConfigSet(
	caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t param);

/**
 * Get DVS noise filter configuration parameters.
 *
 * @param noiseFilter a valid DVS noise filter instance.
 * @param paramAddr a configuration parameter address, see defines CAER_FILTER_DVS_*.
 * @param param a pointer to a configuration parameter value integer,
 *              in which to store the current value.
 *
 * @return true if operation successful, false otherwise.
 */
LIBRARY_PUBLIC_VISIBILITY bool caerFilterDVSNoiseConfigGet(
	caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t *param);

/**
 * Get an array of currently learned hot pixels, in order of activity
 * (most active first, least active last).
 * Useful for working with hardware-based pixel filtering (FPGA/CPLD).
 *
 * @param noiseFilter a valid DVS noise filter instance.
 * @param hotPixels array of DVS pixel addresses, sorted by activity (most active first).
 *                  Memory will be allocated for it automatically. On error, the pointer
 *                  is set to NULL. Remember to free() the memory once done!
 *
 * @return number of hot pixels in array, 0 if no hot pixels were found;
 *         or -1 if an error occurred.
 */
LIBRARY_PUBLIC_VISIBILITY ssize_t caerFilterDVSNoiseGetHotPixels(
	caerFilterDVSNoise noiseFilter, caerFilterDVSPixel *hotPixels);

/**
 * DVS HotPixel Filter:
 * Turn on learning to determine which pixels are hot, meaning abnormally
 * active within a certain time period. In the absence of external stimuli,
 * the only pixels behaving as such must be noise.
 * Once learning is enabled, do not disable it until completed. To verify
 * completion, query this parameter and wait for it to switch from 'true'
 * back to 'false'.
 */
#define CAER_FILTER_DVS_HOTPIXEL_LEARN 0
/**
 * DVS HotPixel Filter:
 * Minimum time (in µs) to accumulate events for during learning.
 */
#define CAER_FILTER_DVS_HOTPIXEL_TIME 1
/**
 * DVS HotPixel Filter:
 * Minimum number of events, during the given learning time, for a
 * pixel to be considered hot.
 */
#define CAER_FILTER_DVS_HOTPIXEL_COUNT 2
/**
 * DVS HotPixel Filter:
 * Enable the hot pixel filter, filtering out the last learned hot pixels.
 */
#define CAER_FILTER_DVS_HOTPIXEL_ENABLE 3
/**
 * DVS HotPixel Filter:
 * Number of events filtered out by the hot pixel filter.
 */
#define CAER_FILTER_DVS_HOTPIXEL_STATISTICS 4
/**
 * DVS HotPixel Filter:
 * Number of ON events filtered out by the hot pixel filter.
 */
#define CAER_FILTER_DVS_HOTPIXEL_STATISTICS_ON 17
/**
 * DVS HotPixel Filter:
 * Number of OFF events filtered out by the hot pixel filter.
 */
#define CAER_FILTER_DVS_HOTPIXEL_STATISTICS_OFF 18

/**
 * DVS Background-Activity Filter:
 * enable the background-activity filter, which tries to remove events
 * caused by transistor leakage, by rejecting uncorrelated events.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE 5
/**
 * DVS Background-Activity Filter:
 * specify the time difference constant for the background-activity
 * filter in microseconds. Events that do correlated within this
 * time-frame are let through, while others are filtered out.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME 6
/**
 * DVS Background-Activity Filter:
 * number of events filtered out by the background-activity filter.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_STATISTICS 7
/**
 * DVS Background-Activity Filter:
 * number of ON events filtered out by the background-activity filter.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_STATISTICS_ON 19
/**
 * DVS Background-Activity Filter:
 * number of OFF events filtered out by the background-activity filter.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_STATISTICS_OFF 20

/**
 * DVS Refractory Period Filter:
 * enable the refractory period filter, which limits the firing rate
 * of pixels.
 */
#define CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE 8
/**
 * DVS Refractory Period Filter:
 * specify the time constant for the refractory period filter.
 * Pixels will be inhibited from generating new events during this
 * time after the last even has fired.
 */
#define CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME 9
/**
 * DVS Refractory Period Filter:
 * number of events filtered out by the refractory period filter.
 */
#define CAER_FILTER_DVS_REFRACTORY_PERIOD_STATISTICS 10
/**
 * DVS Refractory Period Filter:
 * number of ON events filtered out by the refractory period filter.
 */
#define CAER_FILTER_DVS_REFRACTORY_PERIOD_STATISTICS_ON 21
/**
 * DVS Refractory Period Filter:
 * number of OFF events filtered out by the refractory period filter.
 */
#define CAER_FILTER_DVS_REFRACTORY_PERIOD_STATISTICS_OFF 22

/**
 * DVS Noise Filter:
 * set a custom log-level for an instance of the DVS Noise filter.
 */
#define CAER_FILTER_DVS_LOG_LEVEL 11

/**
 * DVS Noise Filter:
 * reset this instance of the filter to its initial state, forgetting
 * any learned hot pixels and clearing the timestamp map and the
 * statistics. This does not change or reset the configuration.
 */
#define CAER_FILTER_DVS_RESET 12

/**
 * DVS Background-Activity Filter:
 * repeat the background-activity check, that at least one neighbor pixel
 * supports this pixel, on each pixel that supported the current pixel in
 * turn, basically repeating the check for a second level of pixels.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TWO_LEVELS 13

/**
 * DVS Background-Activity Filter:
 * minimum number of pixels in the immediate neighborhood that must support
 * the current pixel for it to be considered valid.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MIN 14

/**
 * DVS Background-Activity Filter:
 * maximum number of pixels in the immediate neighborhood that can support
 * the current pixel for it to be considered valid.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MAX 15

/**
 * DVS Background-Activity Filter:
 * whether polarity is considered when searching the neighbors for
 * supporting activity.
 */
#define CAER_FILTER_DVS_BACKGROUND_ACTIVITY_CHECK_POLARITY 16

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FILTERS_DVS_NOISE_H_ */

/**
 * @file edvs.h
 *
 * EDVS-4337 specific configuration defines and information structures.
 */

#ifndef LIBCAER_DEVICES_EDVS_H_
#define LIBCAER_DEVICES_EDVS_H_

#include "../events/polarity.h"
#include "../events/special.h"
#include "serial.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device type definition for iniVation EDVS-4337.
 */
#define CAER_DEVICE_EDVS 5

/**
 * Module address: device-side DVS configuration.
 */
#define EDVS_CONFIG_DVS 0
/**
 * Module address: device-side chip bias generator configuration.
 */
#define EDVS_CONFIG_BIAS 1

/**
 * Parameter address for module EDVS_CONFIG_DVS:
 * run the DVS chip and generate polarity event data.
 */
#define EDVS_CONFIG_DVS_RUN 0
/**
 * Parameter address for module EDVS_CONFIG_DVS:
 * reset the time-stamp counter of the device. This is a temporary
 * configuration switch and will reset itself right away.
 */
#define EDVS_CONFIG_DVS_TIMESTAMP_RESET 1

/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * First stage amplifier cascode bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_CAS 0
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Injected ground bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_INJGND 1
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Pull down on chip request (AER).
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_REQPD 2
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Pull up on request from X arbiter (AER).
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_PUX 3
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Off events threshold bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_DIFFOFF 4
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Pull down for passive load inverters in digital AER pixel circuitry.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_REQ 5
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Refractory period bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_REFR 6
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Pull up on request from Y arbiter (AER).
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_PUY 7
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * On events threshold bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_DIFFON 8
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Differential (second stage amplifier) bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_DIFF 9
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Source follower bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_FOLL 10
/**
 * Parameter address for module EDVS_CONFIG_BIAS:
 * Photoreceptor bias.
 * See 'https://inivation.com/support/hardware/biasing/' for more details.
 */
#define EDVS_CONFIG_BIAS_PR 11

/**
 * EDVS device-related information.
 */
struct caer_edvs_info {
	/// Unique device identifier. Also 'source' for events.
	int16_t deviceID;
	/// Device information string, for logging purposes.
	/// If not NULL, pointed-to memory is *only* valid while the corresponding
	/// device is open! After calling deviceClose() this is invalid memory!
	char *deviceString;
	/// Whether the device is a time-stamp master or slave.
	bool deviceIsMaster;
	/// DVS X axis resolution.
	int16_t dvsSizeX;
	/// DVS Y axis resolution.
	int16_t dvsSizeY;
	/// Connected serial port name (OS-specific).
	char serialPortName[64];
	/// Serial connection baud-rate.
	uint32_t serialBaudRate;
};

/**
 * Return basic information on the device, such as its ID, its
 * resolution, the logic version, and so on. See the 'struct
 * caer_edvs_info' documentation for more details.
 *
 * @param handle a valid device handle.
 *
 * @return a copy of the device information structure if successful,
 *         an empty structure (all zeros) on failure.
 */
struct caer_edvs_info caerEDVSInfoGet(caerDeviceHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_EDVS_H_ */

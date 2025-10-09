#ifndef LIBCAER_DEVICES_IMU_SUPPORT_H_
#define LIBCAER_DEVICES_IMU_SUPPORT_H_

/**
 * List of supported IMU models.
 */
enum caer_imu_types {
	IMU_NONE                 = 0,
	IMU_INVENSENSE_6050_6150 = 1,
	IMU_INVENSENSE_9250      = 2,
	IMU_BOSCH_BMI_160        = 3,
};

/**
 * List of accelerometer scale settings for InvenSense IMUs.
 */
enum caer_imu_invensense_accel_scale {
	ACCEL_2G  = 0,
	ACCEL_4G  = 1,
	ACCEL_8G  = 2,
	ACCEL_16G = 3,
};

/**
 * List of gyroscope scale settings for InvenSense IMUs.
 */
enum caer_imu_invensense_gyro_scale {
	GYRO_250DPS  = 0,
	GYRO_500DPS  = 1,
	GYRO_1000DPS = 2,
	GYRO_2000DPS = 3,
};

/**
 * List of accelerometer scale settings for Bosch IMU.
 */
enum caer_imu_bosch_accel_scale {
	BOSCH_ACCEL_2G  = 0,
	BOSCH_ACCEL_4G  = 1,
	BOSCH_ACCEL_8G  = 2,
	BOSCH_ACCEL_16G = 3,
};

/**
 * List of accelerometer data rate settings for Bosch IMU.
 */
enum caer_imu_bosch_accel_data_rate {
	BOSCH_ACCEL_12_5HZ = 0,
	BOSCH_ACCEL_25HZ   = 1,
	BOSCH_ACCEL_50HZ   = 2,
	BOSCH_ACCEL_100HZ  = 3,
	BOSCH_ACCEL_200HZ  = 4,
	BOSCH_ACCEL_400HZ  = 5,
	BOSCH_ACCEL_800HZ  = 6,
	BOSCH_ACCEL_1600HZ = 7,
};

/**
 * List of accelerometer filter settings for Bosch IMU.
 */
enum caer_imu_bosch_accel_filter {
	BOSCH_ACCEL_OSR4   = 0,
	BOSCH_ACCEL_OSR2   = 1,
	BOSCH_ACCEL_NORMAL = 2,
};

/**
 * List of gyroscope scale settings for Bosch IMU.
 */
enum caer_imu_bosch_gyro_scale {
	BOSCH_GYRO_2000DPS = 0,
	BOSCH_GYRO_1000DPS = 1,
	BOSCH_GYRO_500DPS  = 2,
	BOSCH_GYRO_250DPS  = 3,
	BOSCH_GYRO_125DPS  = 4,
};

/**
 * List of gyroscope data rate settings for Bosch IMU.
 */
enum caer_imu_bosch_gyro_data_rate {
	BOSCH_GYRO_25HZ   = 0,
	BOSCH_GYRO_50HZ   = 1,
	BOSCH_GYRO_100HZ  = 2,
	BOSCH_GYRO_200HZ  = 3,
	BOSCH_GYRO_400HZ  = 4,
	BOSCH_GYRO_800HZ  = 5,
	BOSCH_GYRO_1600HZ = 6,
	BOSCH_GYRO_3200HZ = 7,
};

/**
 * List of gyroscope filter settings for Bosch IMU.
 */
enum caer_imu_bosch_gyro_filter {
	BOSCH_GYRO_OSR4   = 0,
	BOSCH_GYRO_OSR2   = 1,
	BOSCH_GYRO_NORMAL = 2,
};

#endif // LIBCAER_DEVICES_IMU_SUPPORT_H_

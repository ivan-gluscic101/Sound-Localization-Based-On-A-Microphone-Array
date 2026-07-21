#ifndef MPU9250_H
#define MPU9250_H

/* Driver for MPU-9250 / MPU-9255 / MPU-6500 (GY-521) over I2C1 (0x68 or 0x69).
 * Detects chip via WHO_AM_I; enables AK8963 magnetometer on 9250/9255.
 * Mahony filter fuses accel+gyro(+mag) into roll/pitch/yaw. */

#include <stdint.h>

/* WHO_AM_I values for supported chips */
#define MPU_WHOAMI_6500   0x70
#define MPU_WHOAMI_9250   0x71
#define MPU_WHOAMI_9255   0x73

typedef enum {
    MPU_CHIP_UNKNOWN = 0,
    MPU_CHIP_6500,           /* 6-axis: accel + gyro, no compass */
    MPU_CHIP_9250,           /* 9-axis: + AK8963 magnetometer */
    MPU_CHIP_9255            /* 9-axis: + AK8963 magnetometer */
} mpu_chip_t;

/* Board orientation in degrees (Tait-Bryan, ZYX). */
typedef struct {
    float roll_deg;   /* about X (-180..+180) */
    float pitch_deg;  /* about Y (-90..+90) */
    float yaw_deg;    /* about Z (0..360), from compass if present */
} mpu_orientation_t;

/* Init MPU (wake, +-2000 deg/s gyro, +-4g accel, DLPF, magnetometer if present).
 * Call after Custom_I2C1_Init(). Returns detected chip or MPU_CHIP_UNKNOWN. */
mpu_chip_t MPU_Init(void);

/* Return 1 if a working magnetometer (9250/9255) was detected, else 0. */
uint8_t MPU_HasMag(void);

/* Read raw WHO_AM_I byte (0x75) for diagnostics; returns 0xFF on I2C error. */
uint8_t MPU_WhoAmI(void);

/* I2C address the MPU answered on (0x68/0x69), valid after MPU_Init. */
uint8_t MPU_Address(void);

/* Read sensors, run Mahony fusion, output orientation. dt_s = time since last
 * call (s). Call periodically from IMU_Task. Returns 0 ok, !=0 I2C error. */
int MPU_Update(float dt_s, mpu_orientation_t *out);

/* Gyro bias calibration; keep the module still. Blocking (~0.5 s). */
void MPU_CalibrateGyro(void);

#endif /* MPU9250_H */

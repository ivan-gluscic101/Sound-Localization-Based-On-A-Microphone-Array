#include "app_init.h"

#include "i2c_driver.h"
#include "mpu9250.h"
#include "uart_driver.h"

#include <stdio.h>

/* Init IMU (I2C1 + MPU-9250), print WHO_AM_I diagnostics, calibrate gyro.
 * Board must stay still during startup for gyro calibration. */
void app_init(void)
{
    Custom_I2C1_Init();
    mpu_chip_t imu_chip = MPU_Init();

    /* WHO_AM_I: 0x70=6500, 0x71=9250, 0x73=9255, 0x68=6050 */
    {
        char line[64];
        const char *name = (imu_chip == MPU_CHIP_9250) ? "MPU-9250" :
                           (imu_chip == MPU_CHIP_9255) ? "MPU-9255" :
                           (imu_chip == MPU_CHIP_6500) ? "MPU-6500" : "UNKNOWN";
        snprintf(line, sizeof(line),
                 "IMU: %s  WHOAMI=0x%02X  ADDR=0x%02X  MAG=%d\r\n",
                 name, MPU_WhoAmI(), MPU_Address(), MPU_HasMag());
        UART_SendString(line);
    }

    if (imu_chip != MPU_CHIP_UNKNOWN) {
        MPU_CalibrateGyro();
    }
}

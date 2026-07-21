#ifndef APP_INIT_H
#define APP_INIT_H

/* Init I2C1 and MPU IMU, print diagnostics, calibrate gyro if chip present.
 * Call from main() before app_tasks_init(); board must be still during it. */
void app_init(void);

#endif /* APP_INIT_H */

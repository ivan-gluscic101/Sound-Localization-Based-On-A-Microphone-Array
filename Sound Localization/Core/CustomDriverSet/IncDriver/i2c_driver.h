#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

/* Minimal blocking I2C1 master (bare-metal registers). PB8=SCL, PB9=SDA (AF4),
 * 400 kHz Fast-mode, 7-bit addressing. Call from a task, never from an ISR. */

#include <stdint.h>

/* Init GPIO (PB8/PB9 AF open-drain + pull-up) and I2C1 @ 400 kHz. */
void Custom_I2C1_Init(void);

/* Write len bytes to register reg on device dev_addr. Returns 0 ok, !=0 error. */
int I2C1_WriteReg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

/* Write a single byte to a register. Returns 0 on success. */
int I2C1_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t val);

/* Read len bytes starting at register reg (auto-increment). Returns 0 ok, !=0 error. */
int I2C1_ReadReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);

/* Probe a device address. Returns 0 if present, !=0 if no response. */
int I2C1_Ping(uint8_t dev_addr);

#endif /* I2C_DRIVER_H */

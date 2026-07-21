#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "main.h"
#include <stdint.h>

/* Init UART4 (PC10/PC11, 115200 8N1). */
void Custom_UART4_Init(void);

/* Send 2D angle packet (type 0x02): azimuth (0.1 deg) + strength. */
void UART_SendAnglePacket(int16_t phi_tenth_deg, uint8_t strength);

/* Send 3D angle packet (type 0x03): azimuth + elevation (0.1 deg) + strength. */
void UART_SendAngle3DPacket(int16_t az_tenth, int16_t el_tenth, uint8_t strength);

/* Send board orientation (type 0x05): roll/pitch/yaw (0.1 deg) + flags.
 * flags bit0 = magnetometer valid (yaw absolute), else yaw from gyro only. */
void UART_SendOrientationPacket(int16_t roll_tenth, int16_t pitch_tenth,
                                int16_t yaw_tenth, uint8_t flags);

/* Send raw capture (type 0x04): 4 channels as a channel-major big-endian frame. */
void UART_SendRawCapture(const uint16_t *ch0, const uint16_t *ch1,
                         const uint16_t *ch2, const uint16_t *ch3);

/* Send a null-terminated string over UART (blocking). */
void UART_SendString(const char *str);

/* Send raw capture as human-readable CSV (blocking, ~2 s at 115200). */
void UART_SendRawCaptureCSV(const uint16_t *ch0, const uint16_t *ch1,
                            const uint16_t *ch2, const uint16_t *ch3);

#endif /* UART_DRIVER_H */

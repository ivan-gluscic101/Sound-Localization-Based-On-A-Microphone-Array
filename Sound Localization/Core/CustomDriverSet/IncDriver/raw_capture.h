#ifndef RAW_CAPTURE_H
#define RAW_CAPTURE_H

/* Snapshot of raw ADC samples for UART debug / MATLAB export.
 * Interleaved buffer: buf[s*4+0..3] = M1(PB14) M2(PC0) M3(PC1) M4(PC2). */

#include "audio_common.h"
#include <stdint.h>

/* Copy one window into dbg_raw_chX[]. Call only on detection; frame_offset
 * selects the window within a larger buffer. */
void RawCapture_Snapshot(const uint16_t *buf, uint32_t frame_offset);

/* Debug snapshot (last detected window per channel). */
extern uint16_t dbg_raw_ch0[SAMPLES_PER_CHANNEL];
extern uint16_t dbg_raw_ch1[SAMPLES_PER_CHANNEL];
extern uint16_t dbg_raw_ch2[SAMPLES_PER_CHANNEL];
extern uint16_t dbg_raw_ch3[SAMPLES_PER_CHANNEL];

#endif /* RAW_CAPTURE_H */

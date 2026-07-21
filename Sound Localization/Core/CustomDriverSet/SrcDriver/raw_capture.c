#include "raw_capture.h"

uint16_t dbg_raw_ch0[SAMPLES_PER_CHANNEL];
uint16_t dbg_raw_ch1[SAMPLES_PER_CHANNEL];
uint16_t dbg_raw_ch2[SAMPLES_PER_CHANNEL];
uint16_t dbg_raw_ch3[SAMPLES_PER_CHANNEL];

/* Deinterleave one window into per-channel debug buffers (read via debugger). */
void RawCapture_Snapshot(const uint16_t *buf, uint32_t frame_offset)
{
    const uint16_t *p = &buf[frame_offset * NUM_CH];
    for (int s = 0; s < SAMPLES_PER_CHANNEL; s++) {
        dbg_raw_ch0[s] = p[s * NUM_CH + 0];
        dbg_raw_ch1[s] = p[s * NUM_CH + 1];
        dbg_raw_ch2[s] = p[s * NUM_CH + 2];
        dbg_raw_ch3[s] = p[s * NUM_CH + 3];
    }
}

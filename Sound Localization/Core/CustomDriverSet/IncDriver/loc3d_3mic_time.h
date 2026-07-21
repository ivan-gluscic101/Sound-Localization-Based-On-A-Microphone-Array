#ifndef LOC3D_3MIC_TIME_H
#define LOC3D_3MIC_TIME_H

/* Time-domain 3-mic (M1..M3) sound localization via onset threshold.
 * Solves the 2D system; elevation assumes z >= 0. M4 is sampled but unused. */

#include "audio_common.h"
#include <stdint.h>

/* Compute M_geom2 (2x2) from mic geometry. Call once before Process. */
void LOC3D_3MIC_TIME_Init(void);

typedef struct {
    int16_t  az_tenth;   /* azimuth in 0.1 deg, 0..3600 */
    int16_t  el_tenth;   /* elevation in 0.1 deg, 0..+900 */
    uint8_t  strength;   /* signal strength 0-100 */
    uint8_t  _pad;
} loc3d_3mic_time_result_t;

/* Time-domain localization from 3 mics. buf = interleaved [s*NUM_CH+ch] read-only
 * sliding buffer. Returns 1 if a valid direction was computed, 0 otherwise. */
uint8_t LOC3D_3MIC_TIME_Process(const uint16_t *buf, loc3d_3mic_time_result_t *out);

#endif /* LOC3D_3MIC_TIME_H */

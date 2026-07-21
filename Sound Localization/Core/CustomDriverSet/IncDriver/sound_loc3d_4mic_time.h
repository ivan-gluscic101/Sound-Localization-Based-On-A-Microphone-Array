#ifndef SOUND_LOC3D_4MIC_TIME_H
#define SOUND_LOC3D_4MIC_TIME_H

/* Time-domain 4-mic (M1..M4) 3D sound localization via onset threshold.
 * M4 solves the full 3x3 system, giving a true signed elevation (z may be < 0). */

#include "audio_common.h"
#include <stdint.h>

/* Compute M_geom (3x3) from mic geometry. Call once before Process. */
void LOC3D_4MIC_TIME_Init(void);

typedef struct {
    int16_t  az_tenth;   /* azimuth in 0.1 deg, 0..3600 */
    int16_t  el_tenth;   /* elevation in 0.1 deg, -900..+900 */
    uint8_t  strength;   /* signal strength 0-100 */
    uint8_t  _pad;
} loc3d_4mic_time_result_t;

/* Time-domain 3D localization from 4 mics. buf = interleaved [s*NUM_CH+ch] read-only
 * sliding buffer. Returns 1 if a valid direction was computed, 0 otherwise. */
uint8_t LOC3D_4MIC_TIME_Process(const uint16_t *buf, loc3d_4mic_time_result_t *out);

#endif /* SOUND_LOC3D_4MIC_TIME_H */

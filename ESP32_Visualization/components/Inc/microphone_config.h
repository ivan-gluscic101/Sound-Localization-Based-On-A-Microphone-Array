#ifndef MICROPHONE_CONFIG_H
#define MICROPHONE_CONFIG_H

#include <stdint.h>
#include <stddef.h>

/* Microphone positions (cm) for 3D web visualization only; STM32 does the
 * angle math. Must match audio_common.h (cm = m*100). +X fwd, +Y left, +Z up. */

#define NUM_MICROPHONES 4

typedef struct {
    float x;          /* X position [cm] */
    float y;          /* Y position [cm] */
    float z;          /* Z position [cm] */
    const char *name; /* label, e.g. "M1" */
} microphone_t;

/* Regular tetrahedron, edge a=13cm: base x = a*sqrt(3)/2 = 11.2583, y = a/2 = 6.5;
 * M4 above base centroid (cx = a/sqrt(3) = 7.5055) at h = a*sqrt(2/3) = 10.6145. */
static const microphone_t microphones[NUM_MICROPHONES] = {
    { .x =  0.0000f, .y =  0.00f, .z =  0.0000f, .name = "M1" },  /* reference */
    { .x = 11.2583f, .y =  6.50f, .z =  0.0000f, .name = "M2" },  /* left (+Y) */
    { .x = 11.2583f, .y = -6.50f, .z =  0.0000f, .name = "M3" },  /* right (-Y) */
    { .x =  7.5055f, .y =  0.00f, .z = 10.6145f, .name = "M4" }   /* apex */
};

/* Return the microphone array. */
static inline const microphone_t* microphone_config_get_all(void) {
    return microphones;
}

/* Return the microphone count. */
static inline int microphone_config_get_count(void) {
    return NUM_MICROPHONES;
}

/* Return microphone by index, or NULL if out of range. */
static inline const microphone_t* microphone_config_get(int index) {
    if (index < 0 || index >= NUM_MICROPHONES) {
        return NULL;
    }
    return &microphones[index];
}

#endif /* MICROPHONE_CONFIG_H */

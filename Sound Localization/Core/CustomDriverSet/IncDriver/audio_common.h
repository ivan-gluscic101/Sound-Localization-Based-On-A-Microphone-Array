#ifndef AUDIO_COMMON_H
#define AUDIO_COMMON_H

/* Buffer layout */
#define NUM_CH                4
#define SAMPLES_PER_CHANNEL   1024
#define HALF_SIZE             SAMPLES_PER_CHANNEL
#define HALF_BUFFER           (NUM_CH * SAMPLES_PER_CHANNEL)   /* 4096 samples */
#define FULL_BUFFER           (HALF_BUFFER * 2)                /* 8192 samples */

/* 1 = ACQ_Task uses synthetic claps from mock_adc (~68 KB BSS); 0 = real ADC data. */
#define USE_MOCK_ADC          0

/* Localization mode: 1 = full 4-mic time-domain (true elevation sign);
 * 0 = 3-mic (M1..M3, assumes z >= 0), use when M4 (PC2) is faulty. */
#define USE_4MIC_TIME_LOC     1

/* Kept for config compatibility; only 1 is supported (task_manager.c #errors on 0). */
#define USE_TIME_DOMAIN_LOC   1

/* Timing: TIM8 ARR=884 @170 MHz triggers ADC every ~5.21 us = 192.09 kHz/channel.
 * Higher fs = finer angular resolution (TDOA scales linearly with fs). */
#define SAMPLE_RATE_HZ        192000
#define SAMPLE_PERIOD_S       (1.0f / SAMPLE_RATE_HZ)          /* 5.208 us */

/* ADC sequential offset: each channel = 37 cycles @ 42.5 MHz = 870.6 ns.
 * Depends on ADC clock, not fs. */
#define CH_DELAY_S            870.6e-9f

/* Acoustics */
#define SPEED_OF_SOUND        343.0f    /* m/s at ~20C */

/* Microphone geometry: regular tetrahedron, edge a=13 cm, M1 at origin.
 * Azimuth = atan2(y,x) wrapped [0,360); +X fwd, +Y left, +Z up. */
#define MIC1_X   0.000000f
#define MIC1_Y   0.000000f
#define MIC1_Z   0.000000f

#define MIC2_X   0.112583f  /* a*sqrt(3)/2 (forward) */
#define MIC2_Y   0.065000f  /* +a/2 (left) */
#define MIC2_Z   0.000000f

#define MIC3_X   0.112583f  /* a*sqrt(3)/2 (forward) */
#define MIC3_Y  (-0.065000f) /* -a/2 (right) */
#define MIC3_Z   0.000000f

#define MIC4_X   0.075055f  /* base centroid X (a/sqrt(3)) */
#define MIC4_Y   0.000000f  /* base centroid Y */
#define MIC4_Z   0.106145f  /* h = a*sqrt(2/3) = 10.61 cm (apex) */

/* Max TDOA: largest baseline from M1 = edge 13 cm.
 * 0.13/343 * 192000 ~ 73 samples -> 74. Scales with fs. */
#define TDOA_MAX_SAMPLES      74

#endif /* AUDIO_COMMON_H */

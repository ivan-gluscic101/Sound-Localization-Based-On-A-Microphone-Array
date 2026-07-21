#include "sound_loc3d_4mic_time.h"
#include "raw_capture.h"   /* RawCapture_Snapshot (raw samples for UART debug) */
#include <math.h>

/* Time-domain TDOA localization for 4 mics (M1..M4) via onset threshold.
 * Solves the full 3x3 system, giving a true signed 3D direction. */

volatile int32_t  dbg4_onset[4];                            /* onset indices (samples), -1 = no trigger */
volatile int32_t  dbg4_ref_mic;                             /* 0..3 = M1..M4 triggered first */
volatile float    dbg4_tau12_meas, dbg4_tau13_meas, dbg4_tau14_meas;  /* before CH_DELAY correction */
volatile float    dbg4_tau12_corr, dbg4_tau13_corr, dbg4_tau14_corr;  /* after correction */
volatile int32_t  dbg4_peak_abs[4];                         /* max |signal-DC| per channel */
volatile float    dbg4_sx, dbg4_sy, dbg4_sz;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* Total frames in the sliding buffer. */
#define WIN_FRAMES   (2u * SAMPLES_PER_CHANNEL)

/* Fixed DC level and trigger threshold; sample triggers when |sample-DC| > THR.
 * DC_LEVEL = 2048 = mid-range of the 12-bit ADC. */
#define DC_LEVEL         2048
#define THRESHOLD_LEVEL  250

/* Max onset spread between channels (samples); larger = reject as inconsistent. */
#define ONSET_MAX_SPREAD  (TDOA_MAX_SAMPLES + 6)

/* Cooldown after detection (Process calls) to skip claps' tail and echoes;
 * scales with fs (~240 ms at 192 kHz). */
#define DETECTION_COOLDOWN_FRAMES  45

static uint16_t s_cooldown = 0;

/* M_geom = c*inv(D), rows of D = baselines (Mj - M1).
 * Propagation dir u = M_geom*tau; direction to source s = -u. */
static float M_geom[3][3];

/* Analytic 3x3 inverse (adjugate / determinant). */
static void invert3x3(const float m[3][3], float inv[3][3])
{
    float det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    float idet = (fabsf(det) > 1e-12f) ? (1.0f / det) : 0.0f;

    inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * idet;
    inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * idet;
    inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * idet;
    inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * idet;
    inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * idet;
    inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * idet;
    inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * idet;
    inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * idet;
    inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * idet;
}

/* Precompute M_geom = c*inv(D) from the mic geometry. */
void LOC3D_4MIC_TIME_Init(void)
{
    const float D[3][3] = {
        { MIC2_X - MIC1_X, MIC2_Y - MIC1_Y, MIC2_Z - MIC1_Z },
        { MIC3_X - MIC1_X, MIC3_Y - MIC1_Y, MIC3_Z - MIC1_Z },
        { MIC4_X - MIC1_X, MIC4_Y - MIC1_Y, MIC4_Z - MIC1_Z }
    };
    float Dinv[3][3];
    invert3x3(D, Dinv);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            M_geom[i][j] = SPEED_OF_SOUND * Dinv[i][j];
        }
    }
}

/* Find a channel's onset: first sample with |sample-DC| > threshold.
 * Returns sample index or -1; also returns max abs deviation. */
static int32_t find_onset(const uint16_t *buf, uint32_t ch, int32_t *max_abs_out)
{
    int32_t onset   = -1;
    int32_t max_abs = 0;

    for (uint32_t s = 0u; s < WIN_FRAMES; s++) {
        int32_t dev = (int32_t)buf[s * NUM_CH + ch] - DC_LEVEL;
        if (dev < 0) dev = -dev;                 /* |sample-DC| */
        if (dev > max_abs) max_abs = dev;
        if (onset < 0 && dev > THRESHOLD_LEVEL) {
            onset = (int32_t)s;                  /* first threshold crossing */
        }
    }
    *max_abs_out = max_abs;
    return onset;
}

/* Detect onset TDOAs and solve for source azimuth/elevation/strength.
 * Returns 1 on a valid detection, 0 otherwise. */
uint8_t LOC3D_4MIC_TIME_Process(const uint16_t *buf, loc3d_4mic_time_result_t *out)
{
    if (s_cooldown > 0) { s_cooldown--; return 0; }

    /* 1. Onset and max deviation per channel M1..M4. */
    int32_t max_abs[4];
    int32_t onset[4];
    for (uint32_t ch = 0u; ch < 4u; ch++) {
        onset[ch] = find_onset(buf, ch, &max_abs[ch]);
        dbg4_onset[ch]    = onset[ch];
        dbg4_peak_abs[ch] = max_abs[ch];
    }

    /* 2. All channels must trigger, else no valid detection. */
    if (onset[0] < 0 || onset[1] < 0 || onset[2] < 0 || onset[3] < 0) return 0;

    /* ref_mic = channel that triggered first (info/debug) */
    int32_t ref = 0;
    for (int32_t k = 1; k < 4; k++) {
        if (onset[k] < onset[ref]) ref = k;
    }
    dbg4_ref_mic = ref;

    /* 3. Raw snapshot around the first onset (for UART CSV / debugger). */
    {
        uint32_t first = (uint32_t)onset[ref];
        uint32_t half  = SAMPLES_PER_CHANNEL / 2u;
        uint32_t snap_off = (first > half) ? (first - half) : 0u;
        if (snap_off + SAMPLES_PER_CHANNEL > WIN_FRAMES) {
            snap_off = WIN_FRAMES - SAMPLES_PER_CHANNEL;
        }
        RawCapture_Snapshot(buf, snap_off);
    }

    /* 4. Geometry TDOA relative to M1 (samples->seconds): tau1j = onset_Mj - onset_M1. */
    int32_t d12 = onset[1] - onset[0];
    int32_t d13 = onset[2] - onset[0];
    int32_t d14 = onset[3] - onset[0];

    /* Consistency: reject if onsets are spread too far (physically impossible). */
    if (d12 > ONSET_MAX_SPREAD || d12 < -ONSET_MAX_SPREAD ||
        d13 > ONSET_MAX_SPREAD || d13 < -ONSET_MAX_SPREAD ||
        d14 > ONSET_MAX_SPREAD || d14 < -ONSET_MAX_SPREAD) return 0;

    float tau12_meas = (float)d12 * SAMPLE_PERIOD_S;
    float tau13_meas = (float)d13 * SAMPLE_PERIOD_S;
    float tau14_meas = (float)d14 * SAMPLE_PERIOD_S;
    dbg4_tau12_meas = tau12_meas;
    dbg4_tau13_meas = tau13_meas;
    dbg4_tau14_meas = tau14_meas;

    /* 5. Correct sequential ADC sampling skew (M2=+1, M3=+2, M4=+3 ranks). */
    float tau12 = tau12_meas + 1.0f * CH_DELAY_S;
    float tau13 = tau13_meas + 2.0f * CH_DELAY_S;
    float tau14 = tau14_meas + 3.0f * CH_DELAY_S;
    dbg4_tau12_corr = tau12;
    dbg4_tau13_corr = tau13;
    dbg4_tau14_corr = tau14;

    /* 6. Full 3D direction: u = M_geom*tau; direction to source s = -u. */
    float ux = M_geom[0][0] * tau12 + M_geom[0][1] * tau13 + M_geom[0][2] * tau14;
    float uy = M_geom[1][0] * tau12 + M_geom[1][1] * tau13 + M_geom[1][2] * tau14;
    float uz = M_geom[2][0] * tau12 + M_geom[2][1] * tau13 + M_geom[2][2] * tau14;

    float sx = -ux, sy = -uy, sz = -uz;
    float norm = sqrtf(sx * sx + sy * sy + sz * sz);
    if (norm < 1e-9f) return 0;
    sx /= norm; sy /= norm; sz /= norm;

    /* 7. M4 gives the true elevation sign, so sz may be < 0 (source below array). */
    dbg4_sx = sx; dbg4_sy = sy; dbg4_sz = sz;

    /* 8. Angles: azimuth wrapped [0,360), elevation signed [-90,+90]. */
    float az_deg = atan2f(sy, sx) * (180.0f / PI);
    if (az_deg < 0.0f) az_deg += 360.0f;

    float el_deg = asinf(sz) * (180.0f / PI);

    out->az_tenth = (int16_t)(az_deg * 10.0f);
    out->el_tenth = (int16_t)(el_deg * 10.0f);

    /* 9. Strength: log map of the largest deviation across channels. */
    int32_t peak_max = max_abs[0];
    for (int32_t k = 1; k < 4; k++) {
        if (max_abs[k] > peak_max) peak_max = max_abs[k];
    }
    float str_f = 20.0f * log10f((float)peak_max + 1.0f);
    if (str_f < 1.0f)    str_f = 1.0f;
    if (str_f > 100.0f)  str_f = 100.0f;
    out->strength = (uint8_t)str_f;

    s_cooldown = DETECTION_COOLDOWN_FRAMES;
    return 1;
}

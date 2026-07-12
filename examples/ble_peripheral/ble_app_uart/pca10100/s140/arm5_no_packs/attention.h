#ifndef __ATTENTION_H__
#define __ATTENTION_H__

#include <stdint.h>
#include <stdbool.h>

#include "arm_math.h"


/* ============================================================
 * Sampling configuration
 * ============================================================ */

/*
 * Current ADS1292R sampling rate:
 *
 *     Fs = 250 SPS
 */
#define ATTENTION_SAMPLE_RATE_HZ          250U


/* ============================================================
 * FFT configuration
 * ============================================================ */

/*
 * 512-point FFT.
 *
 * Frequency resolution:
 *
 *     250 / 512
 *     ˜ 0.48828125 Hz/bin
 */
#define ATTENTION_FFT_SIZE                512U


/*
 * 50% overlap:
 *
 *     window = 512 samples
 *     hop    = 256 samples
 */
#define ATTENTION_HOP_SIZE                256U


/* ============================================================
 * Filter warm-up
 * ============================================================ */

/*
 * First 1024 raw EEG samples:
 *
 *     raw EEG
 *       ->
 *     1 Hz HP
 *       ->
 *     50 Hz notch
 *       ->
 *     40 Hz LP
 *
 * Filters keep running, but output samples are discarded.
 *
 * At 250 SPS:
 *
 *     1024 / 250
 *     ˜ 4.096 s
 */
#define ATTENTION_FILTER_WARMUP_SAMPLES   1024U


/* ============================================================
 * Baseline configuration
 * ============================================================ */

/*
 * Six VALID windows establish the session baseline.
 *
 * Artifact windows do not advance calibration.
 */
#define ATTENTION_BASELINE_WINDOWS        6U


/* ============================================================
 * Focus hysteresis
 * ============================================================ */

#define ATTENTION_FOCUS_ON_SCORE          51U //The correct one is 60, change it to 51 during the demo
#define ATTENTION_FOCUS_OFF_SCORE         49U //The correct one is 45, change it to 49 during the demo


/* ============================================================
 * Streaming high-pass filter
 * ============================================================ */

/*
 * First-order approximately 1 Hz high-pass:
 *
 * y[n] =
 *
 *     alpha *
 *     (
 *         y[n-1]
 *         +
 *         x[n]
 *         -
 *         x[n-1]
 *     )
 *
 * Fs = 250 Hz
 */
#define ATTENTION_HP_ALPHA                0.9755f


/* ============================================================
 * Artifact / quality thresholds
 *
 * IMPORTANT:
 *
 * These thresholds are applied AFTER:
 *
 *     1 Hz HP
 *       ->
 *     50 Hz notch
 *       ->
 *     40 Hz 4th-order LP
 *
 * We intentionally do NOT increase them again yet.
 * ============================================================ */

#define ATTENTION_MAX_P2P_UV              2500.0f
#define ATTENTION_MAX_RMS_UV              450.0f
#define ATTENTION_MIN_RMS_UV              0.02f


/* ============================================================
 * Debug configuration
 * ============================================================ */

/*
 * 1 = enable ATTQ / ATTCAL / ATTS
 * 0 = disable
 */
#define ATTENTION_DEBUG_QUALITY           1U


/* ============================================================
 * Attention result
 * ============================================================ */

typedef struct
{
    /* --------------------------------------------------------
     * Public state
     * -------------------------------------------------------- */

    uint8_t score;

    bool focused;
    bool valid;
    bool calibrating;
    bool artifact;

    /* --------------------------------------------------------
     * Counters
     * -------------------------------------------------------- */

    uint32_t processed_windows;
    uint32_t artifact_windows;
    uint32_t dropped_windows;

    /* --------------------------------------------------------
     * Signal quality
     * -------------------------------------------------------- */

    float32_t rms_uv;
    float32_t p2p_uv;

    /* --------------------------------------------------------
     * EEG spectral power
     * -------------------------------------------------------- */

    float32_t theta_power;
    float32_t alpha_power;
    float32_t beta_power;

    /* --------------------------------------------------------
     * Attention metric
     *
     *     beta / (theta + alpha)
     * -------------------------------------------------------- */

    float32_t engagement_ratio;

    /* --------------------------------------------------------
     * Session baseline
     * -------------------------------------------------------- */

    float32_t baseline_ratio;

} attention_result_t;


/* ============================================================
 * Public simple state
 * ============================================================ */

extern volatile uint8_t g_attention_score;
extern volatile uint8_t g_attention_focused;
extern volatile uint8_t g_attention_valid;


/* ============================================================
 * Public API
 * ============================================================ */

bool attention_init(void);


void attention_push_sample(
    float32_t eeg_uv
);


bool attention_get_result(
    attention_result_t *result
);


void attention_reset_calibration(void);


bool attention_is_initialized(void);


#endif /* __ATTENTION_H__ */
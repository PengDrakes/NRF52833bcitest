#include "attention.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* ============================================================
 * Internal configuration
 * ============================================================ */

#define ATTENTION_TASK_STACK_WORDS      384U
#define ATTENTION_TASK_PRIORITY         3U
#define ATTENTION_WINDOW_QUEUE_LENGTH   1U

#define ATTENTION_SCORE_SMOOTH_ALPHA    0.25f
#define ATTENTION_BASELINE_ADAPT_ALPHA  0.002f
#define ATTENTION_POWER_EPSILON         1.0e-12f

#define ATTENTION_PI                    3.14159265358979323846f


/* ============================================================
 * Filtered-sequence diagnostic
 *
 * FILTSEQ prints consecutive samples AFTER:
 *
 *     1 Hz HP
 *       ->
 *     50 Hz notch
 *       ->
 *     40 Hz LP
 *       ->
 *     warm-up complete
 *
 * Only 40 consecutive points are printed once.
 *
 * At 250 SPS:
 *
 *     40 / 250 = 0.16 s
 *
 * This is sufficient to expose residual 50 Hz periodicity while
 * avoiding a very long UART burst.
 * ============================================================ */

#define ATTENTION_FILTSEQ_ENABLE        1U
#define ATTENTION_FILTSEQ_COUNT         40U


/* ============================================================
 * Filtered-spectrum diagnostic
 *
 * 1 = print ATTSP
 * 0 = disable
 * ============================================================ */

#define ATTENTION_DEBUG_SPECTRUM        1U


/*
 * Print spectrum diagnostics every N processed windows.
 *
 * 1 = every window
 * 2 = every two windows
 */
#define ATTENTION_SPECTRUM_EVERY_N_WINDOWS  1U


/* ============================================================
 * Coefficient/sample-rate guard
 *
 * All IIR coefficients in this file are designed for:
 *
 *     Fs = 250 Hz
 * ============================================================ */

#if ATTENTION_SAMPLE_RATE_HZ != 250U
#error "Attention IIR coefficients require ATTENTION_SAMPLE_RATE_HZ == 250"
#endif


/* ============================================================
 * Signal-processing chain
 *
 * raw ADS1292R CH2
 *      ->
 * 1 Hz HP
 *      ->
 * 50 Hz notch
 *      ->
 * 40 Hz 4th-order Butterworth LP
 *      ->
 * 1024-point warm-up
 *      ->
 * ring buffer
 *      ->
 * ATTQ + ATTSP + Attention FFT
 * ============================================================ */


/* ============================================================
 * 50 Hz notch
 *
 * Fs = 250 Hz
 * F0 = 50 Hz
 * Q  = 10
 *
 * Transfer function:
 *
 *        b0 + b1*z^-1 + b2*z^-2
 * H(z) = -----------------------
 *        1  + a1*z^-1 + a2*z^-2
 * ============================================================ */

#define ATTENTION_NOTCH_B0        0.940809296182f
#define ATTENTION_NOTCH_B1       -0.581452121972f
#define ATTENTION_NOTCH_B2        0.940809296182f

#define ATTENTION_NOTCH_A1       -0.581452121972f
#define ATTENTION_NOTCH_A2        0.881618592363f


/* ============================================================
 * 40 Hz 4th-order Butterworth low-pass
 *
 * Fs = 250 Hz
 * Fc = 40 Hz
 *
 * Implemented as two cascaded second-order sections.
 * ============================================================ */


/* ------------------------------------------------------------
 * Section 1
 * ------------------------------------------------------------ */

#define ATTENTION_LP1_B0          0.022870207716f
#define ATTENTION_LP1_B1          0.045740415433f
#define ATTENTION_LP1_B2          0.022870207716f

#define ATTENTION_LP1_A1         -0.602033202257f
#define ATTENTION_LP1_A2          0.123559343987f


/* ------------------------------------------------------------
 * Section 2
 * ------------------------------------------------------------ */

#define ATTENTION_LP2_B0          1.000000000000f
#define ATTENTION_LP2_B1          2.000000000000f
#define ATTENTION_LP2_B2          1.000000000000f

#define ATTENTION_LP2_A1         -0.809950298939f
#define ATTENTION_LP2_A2          0.511589764694f


/* ============================================================
 * Queue message
 * ============================================================ */

typedef struct
{
    float32_t samples[ATTENTION_FFT_SIZE];

} attention_window_t;


/* ============================================================
 * Generic biquad
 *
 * Transposed Direct Form II:
 *
 * y  = b0*x + z1
 *
 * z1 = b1*x
 *      - a1*y
 *      + z2
 *
 * z2 = b2*x
 *      - a2*y
 *
 * Denominator convention:
 *
 *     1 + a1*z^-1 + a2*z^-2
 * ============================================================ */

typedef struct
{
    float32_t b0;
    float32_t b1;
    float32_t b2;

    float32_t a1;
    float32_t a2;

    float32_t z1;
    float32_t z2;

} attention_biquad_t;


/* ============================================================
 * Public simple state
 * ============================================================ */

volatile uint8_t g_attention_score = 50U;
volatile uint8_t g_attention_focused = 0U;
volatile uint8_t g_attention_valid = 0U;


/* ============================================================
 * Internal state
 * ============================================================ */

static bool s_initialized = false;
static bool s_fft_ready = false;

static QueueHandle_t s_window_queue = NULL;
static TaskHandle_t s_attention_task_handle = NULL;


/* ============================================================
 * Producer-side circular sample history
 *
 * Ring stores FULLY FILTERED samples:
 *
 *     HP
 *      ->
 *     notch
 *      ->
 *     low-pass
 *
 * Warm-up samples are NOT stored.
 * ============================================================ */

static float32_t s_sample_ring[ATTENTION_FFT_SIZE];

static uint32_t s_ring_write_index = 0U;
static uint32_t s_total_samples = 0U;
static uint32_t s_samples_since_window = 0U;


/* ============================================================
 * Total input sample counter
 *
 * Counts every raw sample entering attention_push_sample().
 * ============================================================ */

static uint32_t s_input_sample_count = 0U;


/* ============================================================
 * Filter warm-up state
 * ============================================================ */

static uint32_t s_filter_warmup_count = 0U;
static bool s_filter_warmup_complete = false;


/* ============================================================
 * FILTSEQ diagnostic state
 * ============================================================ */

static uint32_t s_filtseq_printed = 0U;


/* ============================================================
 * Static queue TX/RX buffers
 *
 * Avoid large local stack objects.
 * ============================================================ */

static attention_window_t s_tx_window;
static attention_window_t s_rx_window;


/* ============================================================
 * FFT working memory
 *
 * Shared only by Attention worker operations.
 * ============================================================ */

static arm_rfft_fast_instance_f32 s_rfft;

static float32_t s_hann[ATTENTION_FFT_SIZE];
static float32_t s_fft_input[ATTENTION_FFT_SIZE];
static float32_t s_fft_output[ATTENTION_FFT_SIZE];


/* ============================================================
 * Streaming 1 Hz high-pass state
 * ============================================================ */

static float32_t s_hp_prev_x = 0.0f;
static float32_t s_hp_prev_y = 0.0f;

static bool s_hp_initialized = false;


/* ============================================================
 * 50 Hz notch state
 * ============================================================ */

static attention_biquad_t s_notch_50hz;


/* ============================================================
 * 40 Hz low-pass states
 * ============================================================ */

static attention_biquad_t s_lowpass_40hz_1;
static attention_biquad_t s_lowpass_40hz_2;


/* ============================================================
 * Baseline and score state
 * ============================================================ */

static float32_t s_baseline_sum = 0.0f;
static float32_t s_baseline_ratio = 1.0f;

static uint32_t s_baseline_count = 0U;

static float32_t s_smoothed_score = 50.0f;

static bool s_focused_state = false;

static attention_result_t s_result;


/* ============================================================
 * Helpers
 * ============================================================ */

static float32_t attention_clampf(
    float32_t value,
    float32_t min_value,
    float32_t max_value
)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}


/* ============================================================
 * Convert spectral band power to permille of total power
 *
 * Return:
 *
 *     0..1000 approximately
 *
 * Example:
 *
 *     250 = 25.0%
 * ============================================================ */

static long attention_power_permille(
    float32_t band_power,
    float32_t total_power
)
{
    float32_t ratio;

    if (total_power <= ATTENTION_POWER_EPSILON)
    {
        return 0L;
    }

    ratio =
        1000.0f
        *
        band_power
        /
        total_power;

    ratio =
        attention_clampf(
            ratio,
            0.0f,
            1000.0f
        );

    return (long)ratio;
}


/* ============================================================
 * Generic biquad initialization
 * ============================================================ */

static void attention_biquad_init(
    attention_biquad_t *filter,
    float32_t b0,
    float32_t b1,
    float32_t b2,
    float32_t a1,
    float32_t a2
)
{
    if (filter == NULL)
    {
        return;
    }

    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;

    filter->a1 = a1;
    filter->a2 = a2;

    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}


/* ============================================================
 * Generic biquad reset
 * ============================================================ */

static void attention_biquad_reset(
    attention_biquad_t *filter
)
{
    if (filter == NULL)
    {
        return;
    }

    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}


/* ============================================================
 * Generic biquad processing
 *
 * Transposed Direct Form II
 * ============================================================ */

static float32_t attention_biquad_process(
    attention_biquad_t *filter,
    float32_t input
)
{
    float32_t output;

    float32_t new_z1;
    float32_t new_z2;

    if (filter == NULL)
    {
        return input;
    }

    output =
        filter->b0
        *
        input
        +
        filter->z1;

    new_z1 =
        filter->b1
        *
        input
        -
        filter->a1
        *
        output
        +
        filter->z2;

    new_z2 =
        filter->b2
        *
        input
        -
        filter->a2
        *
        output;

    filter->z1 = new_z1;
    filter->z2 = new_z2;

    return output;
}


/* ============================================================
 * Initialize streaming filters
 * ============================================================ */

static void attention_filters_init(void)
{
    /* --------------------------------------------------------
     * 50 Hz notch
     * -------------------------------------------------------- */

    attention_biquad_init(
        &s_notch_50hz,

        ATTENTION_NOTCH_B0,
        ATTENTION_NOTCH_B1,
        ATTENTION_NOTCH_B2,

        ATTENTION_NOTCH_A1,
        ATTENTION_NOTCH_A2
    );

    /* --------------------------------------------------------
     * 40 Hz LP section 1
     * -------------------------------------------------------- */

    attention_biquad_init(
        &s_lowpass_40hz_1,

        ATTENTION_LP1_B0,
        ATTENTION_LP1_B1,
        ATTENTION_LP1_B2,

        ATTENTION_LP1_A1,
        ATTENTION_LP1_A2
    );

    /* --------------------------------------------------------
     * 40 Hz LP section 2
     * -------------------------------------------------------- */

    attention_biquad_init(
        &s_lowpass_40hz_2,

        ATTENTION_LP2_B0,
        ATTENTION_LP2_B1,
        ATTENTION_LP2_B2,

        ATTENTION_LP2_A1,
        ATTENTION_LP2_A2
    );
}


/* ============================================================
 * Reset streaming filter states
 * ============================================================ */

static void attention_filters_reset(void)
{
    s_hp_prev_x = 0.0f;
    s_hp_prev_y = 0.0f;

    s_hp_initialized = false;

    attention_biquad_reset(
        &s_notch_50hz
    );

    attention_biquad_reset(
        &s_lowpass_40hz_1
    );

    attention_biquad_reset(
        &s_lowpass_40hz_2
    );
}


/* ============================================================
 * Streaming approximately 1 Hz high-pass
 * ============================================================ */

static float32_t attention_highpass_filter(
    float32_t input_uv
)
{
    float32_t output_uv;

    /*
     * Initialize from current electrode level.
     *
     * This avoids a huge artificial startup impulse from the
     * absolute electrode DC offset.
     */
    if (!s_hp_initialized)
    {
        s_hp_prev_x = input_uv;
        s_hp_prev_y = 0.0f;

        s_hp_initialized = true;

        return 0.0f;
    }

    output_uv =
        ATTENTION_HP_ALPHA
        *
        (
            s_hp_prev_y
            +
            input_uv
            -
            s_hp_prev_x
        );

    s_hp_prev_x = input_uv;
    s_hp_prev_y = output_uv;

    return output_uv;
}


/* ============================================================
 * Complete streaming filter
 *
 * raw
 *  ->
 * HP
 *  ->
 * 50 Hz notch
 *  ->
 * LP section 1
 *  ->
 * LP section 2
 * ============================================================ */

static float32_t attention_filter_sample(
    float32_t raw_uv
)
{
    float32_t value;

    /* --------------------------------------------------------
     * 1. High-pass
     * -------------------------------------------------------- */

    value =
        attention_highpass_filter(
            raw_uv
        );

    /* --------------------------------------------------------
     * 2. 50 Hz notch
     * -------------------------------------------------------- */

    value =
        attention_biquad_process(
            &s_notch_50hz,
            value
        );

    /* --------------------------------------------------------
     * 3. LP section 1
     * -------------------------------------------------------- */

    value =
        attention_biquad_process(
            &s_lowpass_40hz_1,
            value
        );

    /* --------------------------------------------------------
     * 4. LP section 2
     * -------------------------------------------------------- */

    value =
        attention_biquad_process(
            &s_lowpass_40hz_2,
            value
        );

    return value;
}


/* ============================================================
 * Build Hann window
 * ============================================================ */

static void attention_build_hann_window(void)
{
    uint32_t i;

    float32_t phase;

    for (
        i = 0U;
        i < ATTENTION_FFT_SIZE;
        i++
    )
    {
        phase =
            (
                2.0f
                *
                ATTENTION_PI
                *
                (float32_t)i
            )
            /
            (
                (float32_t)
                (
                    ATTENTION_FFT_SIZE
                    -
                    1U
                )
            );

        s_hann[i] =
            0.5f
            -
            0.5f
            *
            arm_cos_f32(
                phase
            );
    }
}


/* ============================================================
 * Publish result
 * ============================================================ */

static void attention_publish_result(
    const attention_result_t *result
)
{
    if (result == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    s_result = *result;

    g_attention_score =
        result->score;

    g_attention_focused =
        result->focused
        ?
        1U
        :
        0U;

    g_attention_valid =
        result->valid
        ?
        1U
        :
        0U;

    taskEXIT_CRITICAL();

    printf(
        "ATT win=%lu "
        "score=%u "
        "valid=%u "
        "focus=%u "
        "cal=%u "
        "art=%u "
        "stack=%lu "
        "heap=%lu\r\n",

        (unsigned long)
        result->processed_windows,

        result->score,

        result->valid
        ?
        1U
        :
        0U,

        result->focused
        ?
        1U
        :
        0U,

        result->calibrating
        ?
        1U
        :
        0U,

        result->artifact
        ?
        1U
        :
        0U,

        (unsigned long)
        uxTaskGetStackHighWaterMark(
            NULL
        ),

        (unsigned long)
        xPortGetFreeHeapSize()
    );
}


/* ============================================================
 * Extract latest chronological window
 * ============================================================ */

static void attention_extract_latest_window(void)
{
    uint32_t i;
    uint32_t index;

    for (
        i = 0U;
        i < ATTENTION_FFT_SIZE;
        i++
    )
    {
        index =
            (
                s_ring_write_index
                +
                i
            )
            %
            ATTENTION_FFT_SIZE;

        s_tx_window.samples[i] =
            s_sample_ring[index];
    }
}


/* ============================================================
 * Artifact / quality check
 *
 * Input is already fully filtered.
 * ============================================================ */

static bool attention_window_has_artifact(
    const float32_t *samples,
    float32_t *mean_uv,
    float32_t *rms_uv,
    float32_t *p2p_uv
)
{
    float32_t max_value;
    float32_t min_value;

    uint32_t max_index;
    uint32_t min_index;

    float32_t mean_value;
    float32_t rms_value;

    bool bad_p2p;
    bool bad_rms_high;
    bool bad_rms_low;

    uint32_t i;

    if (
        (samples == NULL)
        ||
        (mean_uv == NULL)
        ||
        (rms_uv == NULL)
        ||
        (p2p_uv == NULL)
    )
    {
        return true;
    }

    /* --------------------------------------------------------
     * Mean
     * -------------------------------------------------------- */

    arm_mean_f32(
        samples,
        ATTENTION_FFT_SIZE,
        &mean_value
    );

    /* --------------------------------------------------------
     * Maximum
     * -------------------------------------------------------- */

    arm_max_f32(
        samples,
        ATTENTION_FFT_SIZE,
        &max_value,
        &max_index
    );

    /* --------------------------------------------------------
     * Minimum
     * -------------------------------------------------------- */

    arm_min_f32(
        samples,
        ATTENTION_FFT_SIZE,
        &min_value,
        &min_index
    );

    /* --------------------------------------------------------
     * Mean removal for RMS
     * -------------------------------------------------------- */

    for (
        i = 0U;
        i < ATTENTION_FFT_SIZE;
        i++
    )
    {
        s_fft_input[i] =
            samples[i]
            -
            mean_value;
    }

    /* --------------------------------------------------------
     * RMS
     * -------------------------------------------------------- */

    arm_rms_f32(
        s_fft_input,
        ATTENTION_FFT_SIZE,
        &rms_value
    );

    *mean_uv =
        mean_value;

    *rms_uv =
        rms_value;

    *p2p_uv =
        max_value
        -
        min_value;

    /* --------------------------------------------------------
     * Decisions
     * -------------------------------------------------------- */

    bad_p2p =
        (
            *p2p_uv
            >
            ATTENTION_MAX_P2P_UV
        );

    bad_rms_high =
        (
            *rms_uv
            >
            ATTENTION_MAX_RMS_UV
        );

    bad_rms_low =
        (
            *rms_uv
            <
            ATTENTION_MIN_RMS_UV
        );

#if ATTENTION_DEBUG_QUALITY

    printf(
        "ATTQ "
        "mean=%ld "
        "rms=%ld "
        "p2p=%ld "
        "badP=%u "
        "badRH=%u "
        "badRL=%u "
        "limP=%ld "
        "limRH=%ld "
        "limRL=%ld\r\n",

        (long)
        mean_value,

        (long)
        rms_value,

        (long)
        (*p2p_uv),

        bad_p2p
        ?
        1U
        :
        0U,

        bad_rms_high
        ?
        1U
        :
        0U,

        bad_rms_low
        ?
        1U
        :
        0U,

        (long)
        ATTENTION_MAX_P2P_UV,

        (long)
        ATTENTION_MAX_RMS_UV,

        (long)
        ATTENTION_MIN_RMS_UV
    );

#endif

    return
        bad_p2p
        ||
        bad_rms_high
        ||
        bad_rms_low;
}


/* ============================================================
 * Filtered-spectrum diagnostic
 *
 * IMPORTANT:
 *
 * This FFT is performed on the FILTERED 512-point window.
 *
 * Output ATTSP values are approximately permille of total
 * non-DC spectral power:
 *
 *     1000 = 100%
 *      500 = 50%
 *      100 = 10%
 *
 * Bands:
 *
 *     l1_4   = 1..4 Hz
 *     th4_8  = 4..8 Hz
 *     al8_13 = 8..13 Hz
 *     be13_30= 13..30 Hz
 *     h30_40 = 30..40 Hz
 *     m40_60 = 40..60 Hz
 *     m48_52 = 48..52 Hz
 *     hi60   = 60..125 Hz
 *
 * peak_x10:
 *
 *     500 = 50.0 Hz
 *     100 = 10.0 Hz
 * ============================================================ */

static void attention_debug_filtered_spectrum(
    const float32_t *samples,
    float32_t mean_uv,
    uint32_t window_index
)
{
#if ATTENTION_DEBUG_SPECTRUM

    uint32_t i;
    uint32_t k;

    float32_t frequency_hz;

    float32_t real_part;
    float32_t imag_part;
    float32_t power;

    float32_t total_power;

    float32_t power_1_4;
    float32_t power_4_8;
    float32_t power_8_13;
    float32_t power_13_30;
    float32_t power_30_40;
    float32_t power_40_60;
    float32_t power_48_52;
    float32_t power_60_125;

    float32_t peak_power;
    float32_t peak_frequency_hz;

    if (samples == NULL)
    {
        return;
    }

    if (ATTENTION_SPECTRUM_EVERY_N_WINDOWS == 0U)
    {
        return;
    }

    if (
        (
            window_index
            %
            ATTENTION_SPECTRUM_EVERY_N_WINDOWS
        )
        !=
        0U
    )
    {
        return;
    }

    total_power = 0.0f;

    power_1_4 = 0.0f;
    power_4_8 = 0.0f;
    power_8_13 = 0.0f;
    power_13_30 = 0.0f;
    power_30_40 = 0.0f;
    power_40_60 = 0.0f;
    power_48_52 = 0.0f;
    power_60_125 = 0.0f;

    peak_power = 0.0f;
    peak_frequency_hz = 0.0f;

    /* --------------------------------------------------------
     * Mean removal + Hann
     * -------------------------------------------------------- */

    for (
        i = 0U;
        i < ATTENTION_FFT_SIZE;
        i++
    )
    {
        s_fft_input[i] =
            (
                samples[i]
                -
                mean_uv
            )
            *
            s_hann[i];
    }

    /* --------------------------------------------------------
     * FFT
     * -------------------------------------------------------- */

    arm_rfft_fast_f32(
        &s_rfft,
        s_fft_input,
        s_fft_output,
        0
    );

    /* --------------------------------------------------------
     * Inspect all positive-frequency non-DC bins
     * -------------------------------------------------------- */

    for (
        k = 1U;
        k < (ATTENTION_FFT_SIZE / 2U);
        k++
    )
    {
        frequency_hz =
            (
                (float32_t)k
                *
                (float32_t)
                ATTENTION_SAMPLE_RATE_HZ
            )
            /
            (float32_t)
            ATTENTION_FFT_SIZE;

        real_part =
            s_fft_output[
                2U * k
            ];

        imag_part =
            s_fft_output[
                2U * k
                +
                1U
            ];

        power =
            real_part
            *
            real_part
            +
            imag_part
            *
            imag_part;

        total_power +=
            power;

        /* ----------------------------------------------------
         * Peak
         * ---------------------------------------------------- */

        if (power > peak_power)
        {
            peak_power =
                power;

            peak_frequency_hz =
                frequency_hz;
        }

        /* ----------------------------------------------------
         * 1..4 Hz
         * ---------------------------------------------------- */

        if (
            (frequency_hz >= 1.0f)
            &&
            (frequency_hz < 4.0f)
        )
        {
            power_1_4 +=
                power;
        }

        /* ----------------------------------------------------
         * 4..8 Hz
         * ---------------------------------------------------- */

        else if (
            (frequency_hz >= 4.0f)
            &&
            (frequency_hz < 8.0f)
        )
        {
            power_4_8 +=
                power;
        }

        /* ----------------------------------------------------
         * 8..13 Hz
         * ---------------------------------------------------- */

        else if (
            (frequency_hz >= 8.0f)
            &&
            (frequency_hz < 13.0f)
        )
        {
            power_8_13 +=
                power;
        }

        /* ----------------------------------------------------
         * 13..30 Hz
         * ---------------------------------------------------- */

        else if (
            (frequency_hz >= 13.0f)
            &&
            (frequency_hz < 30.0f)
        )
        {
            power_13_30 +=
                power;
        }

        /* ----------------------------------------------------
         * 30..40 Hz
         * ---------------------------------------------------- */

        else if (
            (frequency_hz >= 30.0f)
            &&
            (frequency_hz < 40.0f)
        )
        {
            power_30_40 +=
                power;
        }

        /* ----------------------------------------------------
         * 40..60 Hz
         * ---------------------------------------------------- */

        else if (
            (frequency_hz >= 40.0f)
            &&
            (frequency_hz < 60.0f)
        )
        {
            power_40_60 +=
                power;
        }

        /* ----------------------------------------------------
         * 60..125 Hz
         * ---------------------------------------------------- */

        else if (
            frequency_hz >= 60.0f
        )
        {
            power_60_125 +=
                power;
        }

        /* ----------------------------------------------------
         * Exact residual mains neighborhood:
         *
         * 48..52 Hz
         *
         * This intentionally overlaps power_40_60.
         * ---------------------------------------------------- */

        if (
            (frequency_hz >= 48.0f)
            &&
            (frequency_hz < 52.0f)
        )
        {
            power_48_52 +=
                power;
        }
    }

    printf(
        "ATTSP "
        "win=%lu "
        "peak_x10=%ld "
        "l1_4=%ld "
        "th4_8=%ld "
        "al8_13=%ld "
        "be13_30=%ld "
        "h30_40=%ld "
        "m40_60=%ld "
        "m48_52=%ld "
        "hi60=%ld\r\n",

        (unsigned long)
        window_index,

        (long)(
            peak_frequency_hz
            *
            10.0f
        ),

        attention_power_permille(
            power_1_4,
            total_power
        ),

        attention_power_permille(
            power_4_8,
            total_power
        ),

        attention_power_permille(
            power_8_13,
            total_power
        ),

        attention_power_permille(
            power_13_30,
            total_power
        ),

        attention_power_permille(
            power_30_40,
            total_power
        ),

        attention_power_permille(
            power_40_60,
            total_power
        ),

        attention_power_permille(
            power_48_52,
            total_power
        ),

        attention_power_permille(
            power_60_125,
            total_power
        )
    );

#else

    (void)samples;
    (void)mean_uv;
    (void)window_index;

#endif
}


/* ============================================================
 * Compute Attention frequency-band powers
 * ============================================================ */

static void attention_compute_band_powers(
    const float32_t *samples,
    float32_t mean_uv,
    float32_t *theta_power,
    float32_t *alpha_power,
    float32_t *beta_power
)
{
    uint32_t i;
    uint32_t k;

    float32_t frequency_hz;

    float32_t real_part;
    float32_t imag_part;

    float32_t power;

    *theta_power = 0.0f;
    *alpha_power = 0.0f;
    *beta_power = 0.0f;

    /* --------------------------------------------------------
     * Mean removal + Hann
     * -------------------------------------------------------- */

    for (
        i = 0U;
        i < ATTENTION_FFT_SIZE;
        i++
    )
    {
        s_fft_input[i] =
            (
                samples[i]
                -
                mean_uv
            )
            *
            s_hann[i];
    }

    /* --------------------------------------------------------
     * FFT
     * -------------------------------------------------------- */

    arm_rfft_fast_f32(
        &s_rfft,
        s_fft_input,
        s_fft_output,
        0
    );

    /* --------------------------------------------------------
     * Theta / Alpha / Beta
     * -------------------------------------------------------- */

    for (
        k = 1U;
        k < (ATTENTION_FFT_SIZE / 2U);
        k++
    )
    {
        frequency_hz =
            (
                (float32_t)k
                *
                (float32_t)
                ATTENTION_SAMPLE_RATE_HZ
            )
            /
            (float32_t)
            ATTENTION_FFT_SIZE;

        if (frequency_hz >= 30.0f)
        {
            break;
        }

        real_part =
            s_fft_output[
                2U * k
            ];

        imag_part =
            s_fft_output[
                2U * k
                +
                1U
            ];

        power =
            real_part
            *
            real_part
            +
            imag_part
            *
            imag_part;

        /* Theta 4..8 Hz */
        if (
            (frequency_hz >= 4.0f)
            &&
            (frequency_hz < 8.0f)
        )
        {
            *theta_power +=
                power;
        }

        /* Alpha 8..13 Hz */
        else if (
            (frequency_hz >= 8.0f)
            &&
            (frequency_hz < 13.0f)
        )
        {
            *alpha_power +=
                power;
        }

        /* Beta 13..30 Hz */
        else if (
            (frequency_hz >= 13.0f)
            &&
            (frequency_hz < 30.0f)
        )
        {
            *beta_power +=
                power;
        }
    }
}


/* ============================================================
 * Process one complete EEG window
 * ============================================================ */

static void attention_process_window(
    const float32_t *samples
)
{
    attention_result_t result;

    float32_t mean_uv;

    float32_t denominator;
    float32_t ratio;

    float32_t relative_ratio;
    float32_t raw_score;

    bool artifact;

    memset(
        &result,
        0,
        sizeof(result)
    );

    /* --------------------------------------------------------
     * Copy previous state
     * -------------------------------------------------------- */

    taskENTER_CRITICAL();

    result =
        s_result;

    taskEXIT_CRITICAL();

    result.processed_windows++;

    result.artifact =
        false;

    /* --------------------------------------------------------
     * 1. Signal quality
     * -------------------------------------------------------- */

    artifact =
        attention_window_has_artifact(
            samples,
            &mean_uv,
            &result.rms_uv,
            &result.p2p_uv
        );

    /* --------------------------------------------------------
     * 2. NEW:
     *
     * Always inspect filtered spectrum BEFORE artifact return.
     *
     * Therefore even rejected windows produce ATTSP.
     * -------------------------------------------------------- */

    attention_debug_filtered_spectrum(
        samples,
        mean_uv,
        result.processed_windows
    );

    /* --------------------------------------------------------
     * 3. Artifact rejection
     * -------------------------------------------------------- */

    if (artifact)
    {
        result.artifact =
            true;

        result.artifact_windows++;

        attention_publish_result(
            &result
        );

        return;
    }

    /* --------------------------------------------------------
     * 4. Attention FFT powers
     * -------------------------------------------------------- */

    attention_compute_band_powers(
        samples,
        mean_uv,
        &result.theta_power,
        &result.alpha_power,
        &result.beta_power
    );

    /* --------------------------------------------------------
     * Engagement ratio
     *
     *     beta / (theta + alpha)
     * -------------------------------------------------------- */

    denominator =
        result.theta_power
        +
        result.alpha_power
        +
        ATTENTION_POWER_EPSILON;

    ratio =
        result.beta_power
        /
        denominator;

    result.engagement_ratio =
        ratio;

    /* --------------------------------------------------------
     * Empty spectrum
     * -------------------------------------------------------- */

    if (
        (
            result.theta_power
            +
            result.alpha_power
            +
            result.beta_power
        )
        <=
        ATTENTION_POWER_EPSILON
    )
    {
        result.artifact =
            true;

        result.artifact_windows++;

#if ATTENTION_DEBUG_QUALITY

        printf(
            "ATTQ empty_spectrum=1\r\n"
        );

#endif

        attention_publish_result(
            &result
        );

        return;
    }

    /* ========================================================
     * Baseline calibration
     * ======================================================== */

    if (
        s_baseline_count
        <
        ATTENTION_BASELINE_WINDOWS
    )
    {
        s_baseline_sum +=
            ratio;

        s_baseline_count++;

        s_baseline_ratio =
            s_baseline_sum
            /
            (float32_t)
            s_baseline_count;

        result.baseline_ratio =
            s_baseline_ratio;

        result.score =
            50U;

        result.focused =
            false;

        result.valid =
            false;

        result.calibrating =
            true;

        if (
            s_baseline_count
            >=
            ATTENTION_BASELINE_WINDOWS
        )
        {
            s_smoothed_score =
                50.0f;

            result.valid =
                true;

            result.calibrating =
                false;
        }

#if ATTENTION_DEBUG_QUALITY

        printf(
            "ATTCAL "
            "count=%lu/%u "
            "ratio_x1000=%ld "
            "base_x1000=%ld\r\n",

            (unsigned long)
            s_baseline_count,

            (unsigned int)
            ATTENTION_BASELINE_WINDOWS,

            (long)(
                ratio
                *
                1000.0f
            ),

            (long)(
                s_baseline_ratio
                *
                1000.0f
            )
        );

#endif

        attention_publish_result(
            &result
        );

        return;
    }

    /* ========================================================
     * Relative score
     * ======================================================== */

    if (
        s_baseline_ratio
        <
        ATTENTION_POWER_EPSILON
    )
    {
        s_baseline_ratio =
            ATTENTION_POWER_EPSILON;
    }

    relative_ratio =
        ratio
        /
        s_baseline_ratio;

    relative_ratio =
        attention_clampf(
            relative_ratio,
            0.01f,
            100.0f
        );

    raw_score =
        100.0f
        *
        relative_ratio
        /
        (
            1.0f
            +
            relative_ratio
        );

    raw_score =
        attention_clampf(
            raw_score,
            0.0f,
            100.0f
        );

    /* --------------------------------------------------------
     * Score smoothing
     * -------------------------------------------------------- */

    s_smoothed_score =
        (
            1.0f
            -
            ATTENTION_SCORE_SMOOTH_ALPHA
        )
        *
        s_smoothed_score
        +
        ATTENTION_SCORE_SMOOTH_ALPHA
        *
        raw_score;

    s_smoothed_score =
        attention_clampf(
            s_smoothed_score,
            0.0f,
            100.0f
        );

    /* --------------------------------------------------------
     * Focus hysteresis
     * -------------------------------------------------------- */

    if (
        (!s_focused_state)
        &&
        (
            s_smoothed_score
            >=
            (float32_t)
            ATTENTION_FOCUS_ON_SCORE
        )
    )
    {
        s_focused_state =
            true;
    }

    else if (
        s_focused_state
        &&
        (
            s_smoothed_score
            <=
            (float32_t)
            ATTENTION_FOCUS_OFF_SCORE
        )
    )
    {
        s_focused_state =
            false;
    }

    /* --------------------------------------------------------
     * Result
     * -------------------------------------------------------- */

    result.score =
        (uint8_t)(
            s_smoothed_score
            +
            0.5f
        );

    result.focused =
        s_focused_state;

    result.valid =
        true;

    result.calibrating =
        false;

    result.baseline_ratio =
        s_baseline_ratio;

    /* --------------------------------------------------------
     * Slow baseline adaptation
     * -------------------------------------------------------- */

    s_baseline_ratio =
        (
            1.0f
            -
            ATTENTION_BASELINE_ADAPT_ALPHA
        )
        *
        s_baseline_ratio
        +
        ATTENTION_BASELINE_ADAPT_ALPHA
        *
        ratio;

#if ATTENTION_DEBUG_QUALITY

    printf(
        "ATTS "
        "ratio_x1000=%ld "
        "base_x1000=%ld "
        "rel_x1000=%ld "
        "raw=%ld "
        "smooth=%ld\r\n",

        (long)(
            ratio
            *
            1000.0f
        ),

        (long)(
            result.baseline_ratio
            *
            1000.0f
        ),

        (long)(
            relative_ratio
            *
            1000.0f
        ),

        (long)
        raw_score,

        (long)
        s_smoothed_score
    );

#endif

    attention_publish_result(
        &result
    );
}


/* ============================================================
 * Attention worker task
 * ============================================================ */

static void attention_thread(
    void *pvParameter
)
{
    (void)
    pvParameter;

    for (;;)
    {
        if (
            xQueueReceive(
                s_window_queue,
                &s_rx_window,
                portMAX_DELAY
            )
            ==
            pdTRUE
        )
        {
            attention_process_window(
                s_rx_window.samples
            );
        }
    }
}


/* ============================================================
 * Initialize
 * ============================================================ */

bool attention_init(void)
{
    BaseType_t task_result;

    arm_status fft_status;

    if (s_initialized)
    {
        return true;
    }

    /* --------------------------------------------------------
     * Clear history
     * -------------------------------------------------------- */

    memset(
        s_sample_ring,
        0,
        sizeof(s_sample_ring)
    );

    memset(
        &s_result,
        0,
        sizeof(s_result)
    );

    s_ring_write_index =
        0U;

    s_total_samples =
        0U;

    s_samples_since_window =
        0U;

    s_input_sample_count =
        0U;

    /* --------------------------------------------------------
     * Warm-up reset
     * -------------------------------------------------------- */

    s_filter_warmup_count =
        0U;

    s_filter_warmup_complete =
        false;

    /* --------------------------------------------------------
     * FILTSEQ reset
     * -------------------------------------------------------- */

    s_filtseq_printed =
        0U;

    /* --------------------------------------------------------
     * Filters
     * -------------------------------------------------------- */

    attention_filters_init();

    attention_filters_reset();

    /* --------------------------------------------------------
     * Baseline
     * -------------------------------------------------------- */

    s_baseline_sum =
        0.0f;

    s_baseline_ratio =
        1.0f;

    s_baseline_count =
        0U;

    s_smoothed_score =
        50.0f;

    s_focused_state =
        false;

    /* --------------------------------------------------------
     * Public state
     * -------------------------------------------------------- */

    g_attention_score =
        50U;

    g_attention_focused =
        0U;

    g_attention_valid =
        0U;

    s_result.score =
        50U;

    s_result.focused =
        false;

    s_result.valid =
        false;

    s_result.calibrating =
        true;

    s_result.artifact =
        false;

    /* --------------------------------------------------------
     * Hann
     * -------------------------------------------------------- */

    attention_build_hann_window();

    /* --------------------------------------------------------
     * FFT
     * -------------------------------------------------------- */

    fft_status =
        arm_rfft_fast_init_f32(
            &s_rfft,
            ATTENTION_FFT_SIZE
        );

    if (
        fft_status
        !=
        ARM_MATH_SUCCESS
    )
    {
        s_fft_ready =
            false;

        return false;
    }

    s_fft_ready =
        true;

    /* --------------------------------------------------------
     * Queue
     * -------------------------------------------------------- */

    s_window_queue =
        xQueueCreate(
            ATTENTION_WINDOW_QUEUE_LENGTH,
            sizeof(attention_window_t)
        );

    if (
        s_window_queue
        ==
        NULL
    )
    {
        s_fft_ready =
            false;

        return false;
    }

    /* --------------------------------------------------------
     * Worker
     * -------------------------------------------------------- */

    task_result =
        xTaskCreate(
            attention_thread,
            "attention_thread",
            ATTENTION_TASK_STACK_WORDS,
            NULL,
            ATTENTION_TASK_PRIORITY,
            &s_attention_task_handle
        );

    if (
        task_result
        !=
        pdPASS
    )
    {
        vQueueDelete(
            s_window_queue
        );

        s_window_queue =
            NULL;

        s_attention_task_handle =
            NULL;

        s_fft_ready =
            false;

        return false;
    }

    s_initialized =
        true;

    printf(
        "Attention filters: "
        "HP=1Hz "
        "Notch=50Hz(Q10) "
        "LP=40Hz(order4) "
        "Warmup=%lu "
        "FILTSEQ=%u "
        "ATTSP=%u\r\n",

        (unsigned long)
        ATTENTION_FILTER_WARMUP_SAMPLES,

        (unsigned int)
        ATTENTION_FILTSEQ_COUNT,

        (unsigned int)
        ATTENTION_DEBUG_SPECTRUM
    );

    return true;
}


/* ============================================================
 * Push one raw ADS1292R sample
 * ============================================================ */

void attention_push_sample(
    float32_t eeg_uv
)
{
    BaseType_t queue_result;

    float32_t filtered_uv;

    if (
        (!s_initialized)
        ||
        (!s_fft_ready)
        ||
        (s_window_queue == NULL)
    )
    {
        return;
    }

    /* --------------------------------------------------------
     * Count every raw input sample
     * -------------------------------------------------------- */

    s_input_sample_count++;

    /* --------------------------------------------------------
     * Complete filter chain
     * -------------------------------------------------------- */

    filtered_uv =
        attention_filter_sample(
            eeg_uv
        );

    /* ========================================================
     * Warm-up
     * ======================================================== */

    if (
        s_filter_warmup_count
        <
        ATTENTION_FILTER_WARMUP_SAMPLES
    )
    {
        s_filter_warmup_count++;

        if (
            s_filter_warmup_count
            ==
            ATTENTION_FILTER_WARMUP_SAMPLES
        )
        {
            s_filter_warmup_complete =
                true;

#if ATTENTION_DEBUG_QUALITY

            printf(
                "ATTWARM complete "
                "samples=%lu\r\n",

                (unsigned long)
                s_filter_warmup_count
            );

#endif
        }

        return;
    }

    if (!s_filter_warmup_complete)
    {
        s_filter_warmup_complete =
            true;
    }

    /* ========================================================
     * NEW:
     *
     * FILTSEQ prints first 40 consecutive samples AFTER warm-up.
     *
     * These are actual outputs of:
     *
     *     HP
     *      ->
     *     50 Hz notch
     *      ->
     *     40 Hz LP
     * ======================================================== */

#if ATTENTION_FILTSEQ_ENABLE

    if (
        s_filtseq_printed
        <
        ATTENTION_FILTSEQ_COUNT
    )
    {
        printf(
            "FILTSEQ "
            "N=%lu "
            "Y=%lduV\r\n",

            (unsigned long)
            s_input_sample_count,

            (long)
            filtered_uv
        );

        s_filtseq_printed++;
    }

#endif

    /* ========================================================
     * Store fully filtered sample
     * ======================================================== */

    s_sample_ring[
        s_ring_write_index
    ] =
        filtered_uv;

    s_ring_write_index =
        (
            s_ring_write_index
            +
            1U
        )
        %
        ATTENTION_FFT_SIZE;

    /* ========================================================
     * First complete 512-point post-warmup window
     * ======================================================== */

    if (
        s_total_samples
        <
        ATTENTION_FFT_SIZE
    )
    {
        s_total_samples++;

        if (
            s_total_samples
            <
            ATTENTION_FFT_SIZE
        )
        {
            return;
        }

        s_samples_since_window =
            0U;
    }

    /* ========================================================
     * Overlapping windows
     * ======================================================== */

    else
    {
        s_samples_since_window++;

        if (
            s_samples_since_window
            <
            ATTENTION_HOP_SIZE
        )
        {
            return;
        }

        s_samples_since_window =
            0U;
    }

    /* ========================================================
     * Extract latest chronological window
     * ======================================================== */

    attention_extract_latest_window();

    /* ========================================================
     * Non-blocking queue send
     * ======================================================== */

    queue_result =
        xQueueSend(
            s_window_queue,
            &s_tx_window,
            0U
        );

    if (
        queue_result
        !=
        pdTRUE
    )
    {
        taskENTER_CRITICAL();

        s_result.dropped_windows++;

        taskEXIT_CRITICAL();
    }
}


/* ============================================================
 * Get result
 * ============================================================ */

bool attention_get_result(
    attention_result_t *result
)
{
    if (
        (result == NULL)
        ||
        (!s_initialized)
    )
    {
        return false;
    }

    taskENTER_CRITICAL();

    *result =
        s_result;

    taskEXIT_CRITICAL();

    return true;
}


/* ============================================================
 * Reset baseline calibration
 *
 * Filters remain running.
 *
 * Warm-up is NOT restarted.
 * ============================================================ */

void attention_reset_calibration(void)
{
    if (!s_initialized)
    {
        return;
    }

    taskENTER_CRITICAL();

    s_baseline_sum =
        0.0f;

    s_baseline_ratio =
        1.0f;

    s_baseline_count =
        0U;

    s_smoothed_score =
        50.0f;

    s_focused_state =
        false;

    memset(
        &s_result,
        0,
        sizeof(s_result)
    );

    s_result.score =
        50U;

    s_result.focused =
        false;

    s_result.valid =
        false;

    s_result.calibrating =
        true;

    s_result.artifact =
        false;

    g_attention_score =
        50U;

    g_attention_focused =
        0U;

    g_attention_valid =
        0U;

    taskEXIT_CRITICAL();
}


/* ============================================================
 * Initialization state
 * ============================================================ */

bool attention_is_initialized(void)
{
    return s_initialized;
}
#include "ads1292.h"
#include "attention.h"
#include "interference_test.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "arm_math.h"


/* ============================================================
 * User configuration
 * ============================================================ */

/*
 * 0 = Real EEG input
 *     CH2 = IN2P - IN2N
 *
 * 1 = Internal 1 Hz test square wave
 */
#define ADS1292_INTERNAL_TEST        0


/*
 * Periodic UART print interval.
 *
 * IMPORTANT:
 *
 * Previous value:
 *     25 samples
 *
 * At 250 SPS:
 *     25 / 250 = 0.1 s
 *
 * This corresponds to exactly:
 *     5 cycles at 50 Hz
 *     6 cycles at 60 Hz
 *
 * Therefore 25 may repeatedly sample almost the same phase of
 * mains interference and hide a periodic waveform.
 *
 * New value:
 *     23 samples
 *
 * 23 / 250 = 0.092 s
 *
 * This is not an integer number of 50/60 Hz cycles.
 */
#define ADS1292_PRINT_INTERVAL       250U


/*
 * Consecutive raw-sample diagnostic.
 *
 * Enable:
 *     1 = print one block of consecutive samples
 *     0 = disable
 *
 * Current configuration:
 *     start at sample 1200
 *     print 40 consecutive samples
 *
 * At 250 SPS:
 *     40 / 250 = 160 ms
 *
 * Enough to observe approximately:
 *     8 cycles at 50 Hz
 *     9.6 cycles at 60 Hz
 */
#define ADS1292_RAWSEQ_ENABLE        0U
#define ADS1292_RAWSEQ_START         1200U
#define ADS1292_RAWSEQ_COUNT         40U


/* DRDY timeout in milliseconds */
#define ADS1292_DRDY_TIMEOUT_MS      3000U


/* ============================================================
 * Raw EEG interference diagnostics
 *
 * Statistics are accumulated sample-by-sample before attention.c
 * filtering. A low-priority logger task prints one record/second,
 * so the ADS1292 sampling task is not blocked by UART output.
 * ============================================================ */

#define ADS1292_EEGX_WINDOW_SAMPLES       250U
#define ADS1292_EEGX_SETTLE_SECONDS       5U
#define ADS1292_EEGX_QUEUE_LENGTH         4U
#define ADS1292_EEGX_LOGGER_STACK_WORDS   256U
#define ADS1292_EEGX_LOGGER_PRIORITY      1U

/* Disable the old periodic EEG line; EEGX replaces it. */
#define ADS1292_PERIODIC_EEG_PRINT_ENABLE 0U


/* ============================================================
 * Timing
 *
 * The schematic uses the ADS1292R internal clock.
 * Nominal fCLK is approximately 512 kHz.
 *
 * 4 * tCLK is approximately 7.8125 us.
 * ============================================================ */

#define ADS1292_CMD_DELAY_US         10U
#define ADS1292_CS_END_DELAY_US      10U
#define ADS1292_CS_HIGH_DELAY_US     5U


/* ============================================================
 * SPI instance
 * ============================================================ */

static const nrf_drv_spi_t spi =
    NRF_DRV_SPI_INSTANCE(ADS1292_SPI_INSTANCE);


/* ============================================================
 * Internal driver state
 * ============================================================ */

static bool m_spi_initialized = false;
static bool m_ads1292_ready = false;
static bool m_in_rdatac = false;

/* Kept only for compatibility with the previous interface. */
static volatile bool spi_xfer_done = false;

/*
 * Current ADS1292R task handle.
 * The DRDY ISR notifies this task.
 */
static TaskHandle_t m_ads1292_task_handle = NULL;

/* Number of missed/accumulated DRDY notifications. */
static volatile uint32_t m_ads1292_overrun_count = 0U;


/* ============================================================
 * Existing project data
 *
 * Names are retained to avoid breaking other project modules.
 * ============================================================ */

uint8_t ads1292_data[9] = {0};

int32_t save_data[CYC_ARRAY_LEN] = {0};
uint32_t write_pos = 0U;
uint32_t read_pos = 0U;

int32_t save_data_0[CYC_ARRAY_LEN] = {0};
uint32_t write_pos_0 = 0U;
uint32_t read_pos_0 = 0U;

int32_t save_data_1[CYC_ARRAY_LEN] = {0};
uint32_t write_pos_1 = 0U;
uint32_t read_pos_1 = 0U;

volatile uint8_t init_data_collected = 0U;

static uint32_t sample_count = 0U;
static uint16_t init_index = 0U;

int32_t chn_adc_value[2] = {0};

float ch_v[2] = {
    0.0f,
    0.0f
};

float vol_v = 0.0f;
int32_t vol_value = 0;
int32_t vol_uv = 0;

float32_t data1[INIT_SAMPLE_COUNT] = {0};
float32_t data2[INIT_SAMPLE_COUNT] = {0};


/* ============================================================
 * Calibration variables
 * ============================================================ */

static float32_t max_init_val = 0.0f;
static uint32_t maxIndex = 0U;

static float32_t min_init_val = 0.0f;
static uint32_t minIndex = 0U;


/* ============================================================
 * Raw EEG interference diagnostic state
 * ============================================================ */

typedef struct
{
    uint32_t cycle;
    uint32_t tick;
    uint32_t overrun_count;
    uint32_t dropped_records;

    int32_t mean_x100;
    int32_t rms_x100;
    int32_t p2p_x100;
    int32_t amplitude_50_x100;
    int32_t amplitude_100_x100;

    uint16_t sample_count;

    uint8_t phase;
    uint8_t mode;
    uint8_t settle;

} ads1292_eegx_record_t;


static QueueHandle_t s_eegx_queue = NULL;
static TaskHandle_t s_eegx_logger_task_handle = NULL;

static uint16_t s_eegx_sample_count = 0U;
static uint32_t s_eegx_seen_epoch = 0U;
static uint32_t s_eegx_dropped_records = 0U;

static float32_t s_eegx_mean = 0.0f;
static float32_t s_eegx_m2 = 0.0f;
static float32_t s_eegx_reference = 0.0f;
static float32_t s_eegx_minimum = 0.0f;
static float32_t s_eegx_maximum = 0.0f;

static float32_t s_eegx_50_real = 0.0f;
static float32_t s_eegx_50_imag = 0.0f;
static float32_t s_eegx_100_real = 0.0f;
static float32_t s_eegx_100_imag = 0.0f;

/* Five-sample quadrature tables: Fs=250 Hz, f=50/100 Hz. */
static const float32_t s_eegx_50_cos[5] =
{
    1.000000000000f,
    0.309016994375f,
   -0.809016994375f,
   -0.809016994375f,
    0.309016994375f
};

static const float32_t s_eegx_50_sin[5] =
{
    0.000000000000f,
    0.951056516295f,
    0.587785252292f,
   -0.587785252292f,
   -0.951056516295f
};

static const float32_t s_eegx_100_cos[5] =
{
    1.000000000000f,
   -0.809016994375f,
    0.309016994375f,
    0.309016994375f,
   -0.809016994375f
};

static const float32_t s_eegx_100_sin[5] =
{
    0.000000000000f,
    0.587785252292f,
   -0.951056516295f,
    0.951056516295f,
   -0.587785252292f
};


static int32_t ads1292_scale_x100(float32_t value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value * 100.0f + 0.5f);
    }

    return (int32_t)(value * 100.0f - 0.5f);
}


static void ads1292_eegx_reset(void)
{
    s_eegx_sample_count = 0U;

    s_eegx_mean = 0.0f;
    s_eegx_m2 = 0.0f;
    s_eegx_reference = 0.0f;
    s_eegx_minimum = 0.0f;
    s_eegx_maximum = 0.0f;

    s_eegx_50_real = 0.0f;
    s_eegx_50_imag = 0.0f;
    s_eegx_100_real = 0.0f;
    s_eegx_100_imag = 0.0f;
}


static float32_t ads1292_quadrature_amplitude(
    float32_t real_part,
    float32_t imag_part,
    uint16_t count
)
{
    float32_t magnitude_square;
    float32_t magnitude = 0.0f;

    if (count == 0U)
    {
        return 0.0f;
    }

    magnitude_square =
        real_part * real_part +
        imag_part * imag_part;

    if (magnitude_square <= 0.0f)
    {
        return 0.0f;
    }

    if (
        arm_sqrt_f32(
            magnitude_square,
            &magnitude
        )
        !=
        ARM_MATH_SUCCESS
    )
    {
        return 0.0f;
    }

    return
        2.0f * magnitude /
        (float32_t)count;
}


static void ads1292_eegx_logger_thread(void *pvParameter)
{
    ads1292_eegx_record_t record;

    (void)pvParameter;

    for (;;)
    {
        if (
            xQueueReceive(
                s_eegx_queue,
                &record,
                portMAX_DELAY
            )
            ==
            pdTRUE
        )
        {
            printf(
                "EEGX cycle=%lu phase=%u mode=%u settle=%u tick=%lu "
                "n=%u mean_x100=%ld rms_x100=%ld p2p_x100=%ld "
                "a50_x100=%ld a100_x100=%ld ovr=%lu drop=%lu\r\n",

                (unsigned long)record.cycle,
                (unsigned int)record.phase,
                (unsigned int)record.mode,
                (unsigned int)record.settle,
                (unsigned long)record.tick,
                (unsigned int)record.sample_count,
                (long)record.mean_x100,
                (long)record.rms_x100,
                (long)record.p2p_x100,
                (long)record.amplitude_50_x100,
                (long)record.amplitude_100_x100,
                (unsigned long)record.overrun_count,
                (unsigned long)record.dropped_records
            );
        }
    }
}


static bool ads1292_eegx_logger_init(void)
{
    BaseType_t task_result;

    if (s_eegx_queue != NULL)
    {
        return true;
    }

    s_eegx_queue = xQueueCreate(
        ADS1292_EEGX_QUEUE_LENGTH,
        sizeof(ads1292_eegx_record_t)
    );

    if (s_eegx_queue == NULL)
    {
        return false;
    }

    task_result = xTaskCreate(
        ads1292_eegx_logger_thread,
        "eegx_logger",
        ADS1292_EEGX_LOGGER_STACK_WORDS,
        NULL,
        ADS1292_EEGX_LOGGER_PRIORITY,
        &s_eegx_logger_task_handle
    );

    if (task_result != pdPASS)
    {
        vQueueDelete(s_eegx_queue);
        s_eegx_queue = NULL;
        s_eegx_logger_task_handle = NULL;
        return false;
    }

    return true;
}


static void ads1292_eegx_publish(void)
{
    ads1292_eegx_record_t record;

    float32_t mean;
    float32_t variance;
    float32_t rms = 0.0f;
    float32_t p2p;
    float32_t amplitude_50;
    float32_t amplitude_100;

    uint32_t now;
    uint32_t mode_start;
    uint32_t settle_ticks;

    if (
        (s_eegx_sample_count == 0U) ||
        (s_eegx_queue == NULL)
    )
    {
        return;
    }

    mean = s_eegx_mean;

    variance =
        s_eegx_m2 /
        (float32_t)s_eegx_sample_count;

    if (variance < 0.0f)
    {
        variance = 0.0f;
    }

    if (arm_sqrt_f32(variance, &rms) != ARM_MATH_SUCCESS)
    {
        rms = 0.0f;
    }

    p2p = s_eegx_maximum - s_eegx_minimum;

    amplitude_50 = ads1292_quadrature_amplitude(
        s_eegx_50_real,
        s_eegx_50_imag,
        s_eegx_sample_count
    );

    amplitude_100 = ads1292_quadrature_amplitude(
        s_eegx_100_real,
        s_eegx_100_imag,
        s_eegx_sample_count
    );

    now = (uint32_t)xTaskGetTickCount();
    mode_start = g_max30102_mode_start_tick;
    settle_ticks = (uint32_t)pdMS_TO_TICKS(
        ADS1292_EEGX_SETTLE_SECONDS * 1000UL
    );

    memset(&record, 0, sizeof(record));

    record.cycle = g_max30102_test_cycle;
    record.phase = g_max30102_test_phase;
    record.mode = g_max30102_test_mode;
    record.tick = now;
    record.sample_count = s_eegx_sample_count;

    record.settle =
        ((uint32_t)(now - mode_start) < settle_ticks)
        ?
        1U
        :
        0U;

    record.mean_x100 = ads1292_scale_x100(mean);
    record.rms_x100 = ads1292_scale_x100(rms);
    record.p2p_x100 = ads1292_scale_x100(p2p);
    record.amplitude_50_x100 = ads1292_scale_x100(amplitude_50);
    record.amplitude_100_x100 = ads1292_scale_x100(amplitude_100);

    record.overrun_count = m_ads1292_overrun_count;
    record.dropped_records = s_eegx_dropped_records;

    if (
        xQueueSend(
            s_eegx_queue,
            &record,
            0U
        )
        !=
        pdTRUE
    )
    {
        s_eegx_dropped_records++;
    }
}


static void ads1292_eegx_push_sample(float32_t raw_ch2_uv)
{
    uint32_t current_epoch;
    uint16_t table_index;
    uint16_t new_count;
    float32_t delta;
    float32_t delta_after_mean;
    float32_t centered_sample;

    current_epoch = g_max30102_mode_epoch;

    /* A mode transition invalidates a partial statistics window. */
    if (current_epoch != s_eegx_seen_epoch)
    {
        s_eegx_seen_epoch = current_epoch;
        ads1292_eegx_reset();
    }

    table_index =
        s_eegx_sample_count %
        5U;

    if (s_eegx_sample_count == 0U)
    {
        s_eegx_reference = raw_ch2_uv;
        s_eegx_minimum = raw_ch2_uv;
        s_eegx_maximum = raw_ch2_uv;
    }
    else
    {
        if (raw_ch2_uv < s_eegx_minimum)
        {
            s_eegx_minimum = raw_ch2_uv;
        }

        if (raw_ch2_uv > s_eegx_maximum)
        {
            s_eegx_maximum = raw_ch2_uv;
        }
    }

    /* Stable online mean and variance (Welford). */
    new_count = s_eegx_sample_count + 1U;

    delta = raw_ch2_uv - s_eegx_mean;
    s_eegx_mean += delta / (float32_t)new_count;
    delta_after_mean = raw_ch2_uv - s_eegx_mean;
    s_eegx_m2 += delta * delta_after_mean;

    /*
     * Remove a constant per-window reference before quadrature.
     * Constant removal does not alter the 50/100-Hz components and
     * reduces floating-point cancellation caused by electrode DC.
     */
    centered_sample = raw_ch2_uv - s_eegx_reference;

    s_eegx_50_real +=
        centered_sample *
        s_eegx_50_cos[table_index];

    s_eegx_50_imag -=
        centered_sample *
        s_eegx_50_sin[table_index];

    s_eegx_100_real +=
        centered_sample *
        s_eegx_100_cos[table_index];

    s_eegx_100_imag -=
        centered_sample *
        s_eegx_100_sin[table_index];

    s_eegx_sample_count = new_count;

    if (s_eegx_sample_count >= ADS1292_EEGX_WINDOW_SAMPLES)
    {
        ads1292_eegx_publish();
        ads1292_eegx_reset();
    }
}


/* ============================================================
 * GPIO diagnostics
 * ============================================================ */

static void ads1292_print_pinmap(void)
{
    printf(
        "ADS PINMAP: "
        "DRDY=%lu "
        "MISO=%lu "
        "SCK=%lu "
        "MOSI=%lu "
        "CS=%lu "
        "START=%lu "
        "RESET=%lu\r\n",

        (unsigned long)ADS1292_DRDY_PIN,
        (unsigned long)ADS1292_MISO_PIN,
        (unsigned long)ADS1292_SCK_PIN,
        (unsigned long)ADS1292_MOSI_PIN,
        (unsigned long)ADS1292_CS_PIN,
        (unsigned long)ADS1292_START_PIN,
        (unsigned long)ADS1292_RESET_PIN
    );
}


static void ads1292_print_gpio_diag(
    const char *tag
)
{
    uint32_t p0_out;
    uint32_t p0_dir;

    uint32_t p0_25_out;
    uint32_t p0_25_dir;
    uint32_t p0_25_cnf;

    if (tag == NULL)
    {
        tag = "ADS GPIO";
    }

    p0_out = NRF_P0->OUT;
    p0_dir = NRF_P0->DIR;

    p0_25_out =
        (p0_out >> 25U) &
        1UL;

    p0_25_dir =
        (p0_dir >> 25U) &
        1UL;

    p0_25_cnf =
        NRF_P0->PIN_CNF[25];

    printf(
        "%s: "
        "DRDY_IN=%lu "
        "START_OUT=%lu "
        "RESET_OUT=%lu\r\n",

        tag,

        (unsigned long)
        nrf_gpio_pin_read(
            ADS1292_DRDY_PIN
        ),

        (unsigned long)
        nrf_gpio_pin_out_read(
            ADS1292_START_PIN
        ),

        (unsigned long)
        nrf_gpio_pin_out_read(
            ADS1292_RESET_PIN
        )
    );

    printf(
        "%s: "
        "P0.25_RAW OUT=%lu "
        "DIR=%lu "
        "CNF=0x%08lX "
        "P0.OUT=0x%08lX "
        "P0.DIR=0x%08lX\r\n",

        tag,

        (unsigned long)p0_25_out,
        (unsigned long)p0_25_dir,
        (unsigned long)p0_25_cnf,
        (unsigned long)p0_out,
        (unsigned long)p0_dir
    );
}


/* ============================================================
 * SPI event handler
 *
 * Blocking SPI is currently used.
 * This function is retained for backward compatibility.
 * ============================================================ */

void spi_event_handler(
    nrf_drv_spi_evt_t const *p_event,
    void *p_context
)
{
    (void)p_event;
    (void)p_context;

    spi_xfer_done = true;
}


/* ============================================================
 * CS control
 * ============================================================ */

static void ads1292_cs_low(void)
{
    nrf_gpio_pin_clear(
        ADS1292_CS_PIN
    );
}


static void ads1292_cs_high(void)
{
    /*
     * Keep enough time after the final SCLK edge.
     */
    nrf_delay_us(
        ADS1292_CS_END_DELAY_US
    );

    nrf_gpio_pin_set(
        ADS1292_CS_PIN
    );

    /*
     * Guarantee sufficient CS-high pulse width.
     */
    nrf_delay_us(
        ADS1292_CS_HIGH_DELAY_US
    );
}


/* ============================================================
 * Low-level SPI transfer
 * ============================================================ */

static ret_code_t ads1292_spi_transfer(
    const uint8_t *tx,
    uint8_t *rx,
    uint8_t length
)
{
    ret_code_t err;

    if ((tx == NULL) ||
        (length == 0U))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    ads1292_cs_low();

    err =
        nrf_drv_spi_transfer(
            &spi,
            tx,
            length,
            rx,
            (rx != NULL) ? length : 0U
        );

    ads1292_cs_high();

    return err;
}


/* ============================================================
 * Device initialization
 * ============================================================ */

void device_init(void)
{
    ret_code_t err;

    nrf_drv_spi_config_t spi_config =
        NRF_DRV_SPI_DEFAULT_CONFIG;

    /* --------------------------------------------------------
     * Configure GPIO pins
     * -------------------------------------------------------- */

    nrf_gpio_cfg_output(
        ADS1292_CS_PIN
    );

    nrf_gpio_cfg_output(
        ADS1292_RESET_PIN
    );

    nrf_gpio_cfg_output(
        ADS1292_START_PIN
    );

    nrf_gpio_cfg_input(
        ADS1292_DRDY_PIN,
        NRF_GPIO_PIN_NOPULL
    );

    /* --------------------------------------------------------
     * Safe initial state
     * -------------------------------------------------------- */

    nrf_gpio_pin_set(
        ADS1292_CS_PIN
    );

    nrf_gpio_pin_clear(
        ADS1292_START_PIN
    );

    nrf_gpio_pin_set(
        ADS1292_RESET_PIN
    );

    /* --------------------------------------------------------
     * Initialize SPI only once
     * -------------------------------------------------------- */

    if (!m_spi_initialized)
    {
        spi_config.sck_pin =
            ADS1292_SCK_PIN;

        spi_config.mosi_pin =
            ADS1292_MOSI_PIN;

        spi_config.miso_pin =
            ADS1292_MISO_PIN;

        /*
         * CS is manually controlled.
         */
        spi_config.ss_pin =
            NRF_DRV_SPI_PIN_NOT_USED;

        spi_config.frequency =
            NRF_DRV_SPI_FREQ_250K;

        /*
         * SPI Mode 1:
         *
         * CPOL = 0
         * CPHA = 1
         */
        spi_config.mode =
            NRF_DRV_SPI_MODE_1;

        spi_config.bit_order =
            NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

        /*
         * NULL handler:
         * blocking SPI transfer.
         */
        err =
            nrf_drv_spi_init(
                &spi,
                &spi_config,
                NULL,
                NULL
            );

        if (err != NRF_SUCCESS)
        {
            printf(
                "ADS1292 SPI init failed, "
                "err=%lu\r\n",
                (unsigned long)err
            );

            m_spi_initialized = false;

            return;
        }

        m_spi_initialized = true;
    }

    /* --------------------------------------------------------
     * Hardware reset
     * -------------------------------------------------------- */

    nrf_gpio_pin_clear(
        ADS1292_START_PIN
    );

    nrf_gpio_pin_set(
        ADS1292_RESET_PIN
    );

    nrf_delay_ms(100U);

    nrf_gpio_pin_clear(
        ADS1292_RESET_PIN
    );

    nrf_delay_ms(2U);

    nrf_gpio_pin_set(
        ADS1292_RESET_PIN
    );

    nrf_delay_ms(600U);

    /*
     * Treat post-reset state as RDATAC mode.
     */
    m_in_rdatac = true;
}


/* ============================================================
 * Send a single command
 * ============================================================ */

ret_code_t ads1292_send_cmd(
    uint8_t cmd
)
{
    uint8_t tx[1];

    tx[0] = cmd;

    return
        ads1292_spi_transfer(
            tx,
            NULL,
            1U
        );
}


/* ============================================================
 * Exit RDATAC mode
 * ============================================================ */

static ret_code_t ads1292_exit_rdatac(void)
{
    ret_code_t err;

    if (!m_in_rdatac)
    {
        return NRF_SUCCESS;
    }

    err =
        ads1292_send_cmd(
            ADS1292_CMD_SDATAC
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrf_delay_us(
        ADS1292_CMD_DELAY_US
    );

    m_in_rdatac = false;

    return NRF_SUCCESS;
}


/* ============================================================
 * Write one register
 * ============================================================ */

ret_code_t ads1292_write_reg(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t tx[3];

    ret_code_t err;

    err =
        ads1292_exit_rdatac();

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    tx[0] =
        ADS1292_CMD_WREG |
        (reg & 0x1FU);

    /*
     * One register:
     * N - 1 = 0.
     */
    tx[1] = 0x00U;

    tx[2] = value;

    return
        ads1292_spi_transfer(
            tx,
            NULL,
            3U
        );
}


/* ============================================================
 * Read one register
 * ============================================================ */

ret_code_t ads1292_read_reg(
    uint8_t reg,
    uint8_t *value
)
{
    uint8_t tx[3];
    uint8_t rx[3];

    ret_code_t err;

    if (value == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err =
        ads1292_exit_rdatac();

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    memset(
        rx,
        0,
        sizeof(rx)
    );

    tx[0] =
        ADS1292_CMD_RREG |
        (reg & 0x1FU);

    /*
     * One register:
     * N - 1 = 0.
     */
    tx[1] = 0x00U;

    /*
     * Dummy byte.
     */
    tx[2] = 0x00U;

    err =
        ads1292_spi_transfer(
            tx,
            rx,
            3U
        );

    if (err == NRF_SUCCESS)
    {
        *value = rx[2];
    }

    return err;
}


/* ============================================================
 * Write register and verify by readback
 * ============================================================ */

static ret_code_t ads1292_write_verify(
    uint8_t reg,
    uint8_t value
)
{
    ret_code_t err;

    uint8_t readback = 0U;

    err =
        ads1292_write_reg(
            reg,
            value
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err =
        ads1292_read_reg(
            reg,
            &readback
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    if (readback != value)
    {
        printf(
            "ADS register verify failed: "
            "reg=0x%02X "
            "write=0x%02X "
            "read=0x%02X\r\n",

            reg,
            value,
            readback
        );

        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}


/* ============================================================
 * ADS1292R initialization
 * ============================================================ */

void ads1292_init(void)
{
    ret_code_t err;

    uint8_t id = 0U;

    uint8_t rld_sens_readback = 0U;
    uint8_t resp2_readback = 0U;

    m_ads1292_ready = false;

    /* --------------------------------------------------------
     * Initialize lower layer if required
     * -------------------------------------------------------- */

    if (!m_spi_initialized)
    {
        device_init();
    }

    if (!m_spi_initialized)
    {
        printf(
            "ADS1292 SPI is not ready\r\n"
        );

        return;
    }

    /* --------------------------------------------------------
     * Exit RDATAC before register access
     * -------------------------------------------------------- */

    err =
        ads1292_exit_rdatac();

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 SDATAC failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* --------------------------------------------------------
     * Read device ID
     * -------------------------------------------------------- */

    err =
        ads1292_read_reg(
            ADS1292_REG_ID,
            &id
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 ID read failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    printf(
        "ADS1292 ID value: 0x%02X\r\n",
        id
    );

    if (id != ADS1292_EXPECTED_ID)
    {
        printf(
            "ADS1292 ID error, "
            "expected 0x73\r\n"
        );

        return;
    }

    /* ========================================================
     * CONFIG1 = 0x01
     * 250 SPS
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_CONFIG1,
            0x01U
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 CONFIG1 failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * CONFIG2
     * ======================================================== */

#if ADS1292_INTERNAL_TEST

    /*
     * Internal reference enabled
     * Test signal enabled
     * 1 Hz square wave
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_CONFIG2,
            0xA3U
        );

#else

    /*
     * Internal reference enabled
     * Test signal disabled
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_CONFIG2,
            0xA0U
        );

#endif

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 CONFIG2 failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /*
     * Allow internal reference to settle.
     */
    vTaskDelay(
        pdMS_TO_TICKS(200U)
    );

    /* ========================================================
     * LOFF = 0x10
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_LOFF,
            0x10U
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 LOFF failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * CH1SET = 0x81
     *
     * CH1 powered down
     * input short
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_CH1SET,
            0x81U
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 CH1SET failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * CH2 configuration
     * ======================================================== */

#if ADS1292_INTERNAL_TEST

    /*
     * Gain = 6
     * MUX = internal test
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_CH2SET,
            0x05U
        );

#else

    /*
     * CH2SET = 0x60
     *
     * Gain = 12
     * normal electrode input
     *
     * CH2 = IN2P - IN2N
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_CH2SET,
            0x60U
        );

#endif

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 CH2SET failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * RLD configuration
     * ======================================================== */

#if ADS1292_INTERNAL_TEST

    /*
     * Internal test:
     * disable RLD.
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_RLD_SENS,
            0x00U
        );

#else

#if ADS1292_ENABLE_RLD

    /*
     * RLD_SENS = 0x2C
     *
     * bit5 PDB_RLD = 1
     * bit3 RLD2N   = 1
     * bit2 RLD2P   = 1
     *
     * Enable RLD and sense CH2 IN2P + IN2N.
     */
    err =
        ads1292_write_verify(
            ADS1292_REG_RLD_SENS,
            ADS1292_RLD_SENS_CH2
        );

#else

    err =
        ads1292_write_verify(
            ADS1292_REG_RLD_SENS,
            0x00U
        );

#endif /* ADS1292_ENABLE_RLD */

#endif /* ADS1292_INTERNAL_TEST */

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 RLD_SENS failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * LOFF_SENS = 0x00
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_LOFF_SENS,
            0x00U
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 LOFF_SENS failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * RESP1 = 0x02
     * Respiration disabled
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_RESP1,
            0x02U
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 RESP1 failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * RESP2 = 0x03
     *
     * Internal RLD reference
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_RESP2,
            ADS1292_RESP2_RLD_INT
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 RESP2 failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * GPIO = 0x0C
     * ======================================================== */

    err =
        ads1292_write_verify(
            ADS1292_REG_GPIO,
            0x0CU
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 GPIO failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    /* ========================================================
     * Final RLD readback
     * ======================================================== */

    err =
        ads1292_read_reg(
            ADS1292_REG_RLD_SENS,
            &rld_sens_readback
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 RLD_SENS final read failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    err =
        ads1292_read_reg(
            ADS1292_REG_RESP2,
            &resp2_readback
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 RESP2 final read failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        return;
    }

    printf(
        "ADS RLD VERIFY: "
        "RLD_SENS=0x%02X "
        "RESP2=0x%02X\r\n",

        rld_sens_readback,
        resp2_readback
    );

    m_ads1292_ready = true;

    /* ========================================================
     * Final mode print
     * ======================================================== */

#if ADS1292_INTERNAL_TEST

    printf(
        "ADS1292 config OK: "
        "CH2 internal 1 Hz test, "
        "RLD OFF\r\n"
    );

#else

#if ADS1292_ENABLE_RLD

    printf(
        "ADS1292 config OK: "
        "CH2 EEG, "
        "250 SPS, "
        "Gain=12, "
        "RLD ON, "
        "sense=IN2P+IN2N\r\n"
    );

#else

    printf(
        "ADS1292 config OK: "
        "CH2 EEG, "
        "250 SPS, "
        "Gain=12, "
        "RLD OFF\r\n"
    );

#endif

#endif
}


/* ============================================================
 * Start continuous sampling
 * ============================================================ */

ret_code_t ads1292_start_sampling(void)
{
    ret_code_t err;

    if (!m_ads1292_ready)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    nrf_gpio_pin_clear(
        ADS1292_START_PIN
    );

    nrf_delay_us(20U);

    /* Start conversion */
    err =
        ads1292_send_cmd(
            ADS1292_CMD_START
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrf_delay_us(20U);

    /* Enter RDATAC */
    err =
        ads1292_send_cmd(
            ADS1292_CMD_RDATAC
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrf_delay_us(
        ADS1292_CMD_DELAY_US
    );

    m_in_rdatac = true;

    ads1292_print_gpio_diag(
        "After START+RDATAC"
    );

    return NRF_SUCCESS;
}


/* ============================================================
 * Stop sampling
 * ============================================================ */

ret_code_t ads1292_stop_sampling(void)
{
    ret_code_t err;

    nrf_gpio_pin_clear(
        ADS1292_START_PIN
    );

    nrf_delay_us(20U);

    err =
        ads1292_exit_rdatac();

    return err;
}


/* ============================================================
 * Standby mode
 * ============================================================ */

void ads1292_standby(void)
{
    ret_code_t err;

    if (!m_spi_initialized)
    {
        return;
    }

    err =
        ads1292_stop_sampling();

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 stop failed: %lu\r\n",
            (unsigned long)err
        );

        return;
    }

    err =
        ads1292_send_cmd(
            ADS1292_CMD_STANDBY
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 standby failed: %lu\r\n",
            (unsigned long)err
        );

        return;
    }

    printf(
        "ADS1292 standby mode\r\n"
    );
}


/* ============================================================
 * Wakeup from standby
 * ============================================================ */

void ads1292_wakeup(void)
{
    ret_code_t err;

    if (!m_spi_initialized)
    {
        return;
    }

    err =
        ads1292_send_cmd(
            ADS1292_CMD_WAKEUP
        );

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 wakeup failed: %lu\r\n",
            (unsigned long)err
        );

        return;
    }

    vTaskDelay(
        pdMS_TO_TICKS(20U)
    );

    err =
        ads1292_start_sampling();

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 restart failed: %lu\r\n",
            (unsigned long)err
        );

        return;
    }

    printf(
        "ADS1292 wakeup mode\r\n"
    );
}


/* ============================================================
 * Sign extension:
 * 24-bit two's complement -> int32_t
 * ============================================================ */

int32_t ads1292_sign_extend_24(
    const uint8_t data[3]
)
{
    uint32_t value;

    if (data == NULL)
    {
        return 0;
    }

    value =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2]);

    if ((value & 0x00800000UL) != 0UL)
    {
        value |= 0xFF000000UL;
    }

    return (int32_t)value;
}


/* ============================================================
 * Convert ADC code to microvolts
 *
 * Vinput =
 *
 *     code * VREF
 *     -----------
 *     Gain * 2^23
 * ============================================================ */

float ads1292_code_to_uv(
    int32_t code,
    uint8_t gain
)
{
    const float vref_uv =
        ADS1292_VREF_VOLTS *
        1000000.0f;

    if (gain == 0U)
    {
        return 0.0f;
    }

    return
        (
            (float)code *
            vref_uv
        )
        /
        (
            (float)gain *
            8388608.0f
        );
}


/* ============================================================
 * Read one complete 9-byte frame
 * ============================================================ */

ret_code_t ads1292_read_frame(
    ads1292_frame_t *frame
)
{
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};

    ret_code_t err;

    if (frame == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (!m_in_rdatac)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err =
        ads1292_spi_transfer(
            tx,
            rx,
            9U
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    memcpy(
        ads1292_data,
        rx,
        9U
    );

    frame->status =
        ((uint32_t)rx[0] << 16) |
        ((uint32_t)rx[1] << 8) |
        ((uint32_t)rx[2]);

    frame->ch1 =
        ads1292_sign_extend_24(
            &rx[3]
        );

    frame->ch2 =
        ads1292_sign_extend_24(
            &rx[6]
        );

    return NRF_SUCCESS;
}


/* ============================================================
 * DRDY ISR
 * ============================================================ */

static void drdy_isr_handler(
    nrfx_gpiote_pin_t pin,
    nrf_gpiote_polarity_t action
)
{
    BaseType_t higher_priority_task_woken =
        pdFALSE;

    if ((pin == ADS1292_DRDY_PIN) &&
        (action == NRF_GPIOTE_POLARITY_HITOLO))
    {
        if (m_ads1292_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(
                m_ads1292_task_handle,
                &higher_priority_task_woken
            );

            portYIELD_FROM_ISR(
                higher_priority_task_woken
            );
        }
    }
}


/* ============================================================
 * DRDY interrupt initialization
 * ============================================================ */

static ret_code_t ads1292_drdy_irq_init(void)
{
    ret_code_t err;

    nrfx_gpiote_in_config_t in_config =
        NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

    in_config.pull =
        NRF_GPIO_PIN_NOPULL;

    err =
        nrfx_gpiote_init();

    if ((err != NRF_SUCCESS) &&
        (err != NRFX_ERROR_INVALID_STATE))
    {
        return err;
    }

    nrf_gpio_cfg_input(
        ADS1292_DRDY_PIN,
        NRF_GPIO_PIN_NOPULL
    );

    err =
        nrfx_gpiote_in_init(
            ADS1292_DRDY_PIN,
            &in_config,
            drdy_isr_handler
        );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrfx_gpiote_in_event_enable(
        ADS1292_DRDY_PIN,
        true
    );

    return NRF_SUCCESS;
}


/* ============================================================
 * Existing calibration function
 * ============================================================ */

void ADS1292_val_init(
    float32_t *data,
    float32_t *a,
    float32_t *b
)
{
    float32_t range;

    if ((data == NULL) ||
        (a == NULL) ||
        (b == NULL))
    {
        return;
    }

    arm_max_f32(
        data,
        INIT_SAMPLE_COUNT,
        &max_init_val,
        &maxIndex
    );

    arm_min_f32(
        data,
        INIT_SAMPLE_COUNT,
        &min_init_val,
        &minIndex
    );

    range =
        max_init_val -
        min_init_val;

    if (range == 0.0f)
    {
        *a = 0.0f;
        *b = 0.0f;

        return;
    }

    *a =
        180.0f /
        range;

    *b =
        220.0f -
        (*a) *
        max_init_val;
}


/* ============================================================
 * Existing respiration calibration function
 * ============================================================ */

void bre_val_init(
    float32_t *data,
    float32_t *a,
    float32_t *b
)
{
    float32_t range;

    if ((data == NULL) ||
        (a == NULL) ||
        (b == NULL))
    {
        return;
    }

    arm_max_f32(
        data,
        INIT_SAMPLE_COUNT,
        &max_init_val,
        &maxIndex
    );

    arm_min_f32(
        data,
        INIT_SAMPLE_COUNT,
        &min_init_val,
        &minIndex
    );

    range =
        max_init_val -
        min_init_val;

    if (range == 0.0f)
    {
        *a = 0.0f;
        *b = 0.0f;

        return;
    }

    *a =
        2.0f /
        range;

    *b =
        1.0f -
        (*a) *
        max_init_val;
}


/* ============================================================
 * FreeRTOS sampling task
 * ============================================================ */

void spi_read(
    void *pvParameter
)
{
    ret_code_t err;

    ads1292_frame_t frame;

    uint32_t notify_count;

    float ch1_uv;
    float ch2_uv;

    (void)pvParameter;

    /* --------------------------------------------------------
     * Save task handle for DRDY ISR
     * -------------------------------------------------------- */

    m_ads1292_task_handle =
        xTaskGetCurrentTaskHandle();

    s_eegx_seen_epoch = g_max30102_mode_epoch;
    ads1292_eegx_reset();

    if (!ads1292_eegx_logger_init())
    {
        printf("EEGX logger init failed\r\n");
    }
    else
    {
        printf("EEGX logger started\r\n");
    }

    printf(
        "\r\nADS1292 task start\r\n"
    );

    ads1292_print_pinmap();

    /* ========================================================
     * 1. GPIO + SPI + reset
     * ======================================================== */

    device_init();

    ads1292_print_gpio_diag(
        "After device_init"
    );

    if (!m_spi_initialized)
    {
        printf(
            "ADS1292 device init failed\r\n"
        );

        m_ads1292_task_handle = NULL;

        vTaskDelete(NULL);

        return;
    }

    /* ========================================================
     * 2. Configure ADS1292R
     * ======================================================== */

    ads1292_init();

    ads1292_print_gpio_diag(
        "After ads1292_init"
    );

    if (!m_ads1292_ready)
    {
        printf(
            "ADS1292 config failed\r\n"
        );

        m_ads1292_task_handle = NULL;

        vTaskDelete(NULL);

        return;
    }

    /* ========================================================
     * 3. Initialize DRDY interrupt
     * ======================================================== */

    err =
        ads1292_drdy_irq_init();

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 DRDY init failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        m_ads1292_task_handle = NULL;

        vTaskDelete(NULL);

        return;
    }

    printf(
        "ADS1292 DRDY IRQ OK\r\n"
    );

    ads1292_print_gpio_diag(
        "After DRDY IRQ init"
    );

    /* ========================================================
     * 4. Start sampling
     * ======================================================== */

    err =
        ads1292_start_sampling();

    if (err != NRF_SUCCESS)
    {
        printf(
            "ADS1292 start failed, "
            "err=%lu\r\n",

            (unsigned long)err
        );

        m_ads1292_task_handle = NULL;

        vTaskDelete(NULL);

        return;
    }

    printf(
        "ADS1292 sampling started\r\n"
    );

    /* --------------------------------------------------------
     * Start Attention
     * -------------------------------------------------------- */

    if (!attention_init())
    {
        printf(
            "Attention estimator init failed\r\n"
        );
    }
    else
    {
        printf(
            "Attention estimator started\r\n"
        );
    }

    /* ========================================================
     * 5. Sampling loop
     * ======================================================== */

    for (;;)
    {
        /* ----------------------------------------------------
         * Wait for DRDY
         * ---------------------------------------------------- */

        notify_count =
            ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(
                    ADS1292_DRDY_TIMEOUT_MS
                )
            );

        if (notify_count == 0U)
        {
            printf(
                "ADS1292 DRDY timeout\r\n"
            );

            ads1292_print_gpio_diag(
                "At DRDY timeout"
            );

            continue;
        }

        /* ----------------------------------------------------
         * Accumulated DRDY notifications
         * ---------------------------------------------------- */

        if (notify_count > 1U)
        {
            m_ads1292_overrun_count +=
                notify_count -
                1U;
        }

        /* ----------------------------------------------------
         * Read one frame
         * ---------------------------------------------------- */

        err =
            ads1292_read_frame(
                &frame
            );

        if (err != NRF_SUCCESS)
        {
            printf(
                "ADS1292 read failed, "
                "err=%lu\r\n",

                (unsigned long)err
            );

            continue;
        }

        /* ----------------------------------------------------
         * Validate STATUS high nibble
         * ---------------------------------------------------- */

        if ((ads1292_data[0] & 0xF0U) != 0xC0U)
        {
            continue;
        }

        sample_count++;

        /* ====================================================
         * CH1
         * ==================================================== */

        chn_adc_value[0] =
            frame.ch1;

        ch1_uv =
            ads1292_code_to_uv(
                frame.ch1,
                12U
            );

        ch_v[0] =
            ch1_uv /
            1000000.0f;

        /* ====================================================
         * CH2
         * ==================================================== */

        chn_adc_value[1] =
            frame.ch2;

#if ADS1292_INTERNAL_TEST

        ch2_uv =
            ads1292_code_to_uv(
                frame.ch2,
                6U
            );

#else

        ch2_uv =
            ads1292_code_to_uv(
                frame.ch2,
                ADS1292_CH2_GAIN
            );

#endif

        ch_v[1] =
            ch2_uv /
            1000000.0f;

        /* ====================================================
         * NEW DIAGNOSTIC:
         *
         * Print 40 consecutive samples once.
         *
         * This is intentionally placed after ch2_uv conversion.
         * ==================================================== */

#if ADS1292_RAWSEQ_ENABLE

        if (
            (sample_count >= ADS1292_RAWSEQ_START) &&
            (
                sample_count <
                (
                    ADS1292_RAWSEQ_START +
                    ADS1292_RAWSEQ_COUNT
                )
            )
        )
        {
            printf(
                "RAWSEQ "
                "N=%lu "
                "CH2=%lduV "
                "RAW=%ld "
                "OVR=%lu\r\n",

                (unsigned long)sample_count,
                (long)ch2_uv,
                (long)frame.ch2,
                (unsigned long)
                m_ads1292_overrun_count
            );
        }

#endif /* ADS1292_RAWSEQ_ENABLE */

        /* ----------------------------------------------------
         * Raw, unfiltered EEG interference metrics.
         *
         * Keep this before attention_push_sample(), because
         * attention.c applies HP/notch/LP filtering.
         * ---------------------------------------------------- */

        ads1292_eegx_push_sample(
            ch2_uv
        );

        /* ----------------------------------------------------
         * Forward EEG to Attention
         * ---------------------------------------------------- */

        attention_push_sample(
            ch2_uv
        );

        /* ----------------------------------------------------
         * Compatibility variables
         * ---------------------------------------------------- */

        vol_value =
            (int32_t)ch2_uv;

        vol_uv =
            vol_value;

        /* ====================================================
         * CH1 ring buffer
         * ==================================================== */

        if (
            (
                (write_pos_0 + 1U) %
                CYC_ARRAY_LEN
            )
            !=
            read_pos_0
        )
        {
            save_data_0[write_pos_0] =
                (int32_t)ch1_uv;

            write_pos_0 =
                (write_pos_0 + 1U) %
                CYC_ARRAY_LEN;
        }

        /* ====================================================
         * CH2 ring buffer
         * ==================================================== */

        if (
            (
                (write_pos_1 + 1U) %
                CYC_ARRAY_LEN
            )
            !=
            read_pos_1
        )
        {
            save_data_1[write_pos_1] =
                (int32_t)ch2_uv;

            write_pos_1 =
                (write_pos_1 + 1U) %
                CYC_ARRAY_LEN;
        }

        /* ====================================================
         * Common ring buffer
         * ==================================================== */

        if (
            (
                (write_pos + 1U) %
                CYC_ARRAY_LEN
            )
            !=
            read_pos
        )
        {
            save_data[write_pos] =
                (int32_t)ch2_uv;

            write_pos =
                (write_pos + 1U) %
                CYC_ARRAY_LEN;
        }

        /* ====================================================
         * Initial data collection
         * ==================================================== */

        if (!init_data_collected)
        {
            if (
                (sample_count >= INIT_SAMPLE_START) &&
                (sample_count <= INIT_SAMPLE_END)
            )
            {
                if (init_index < INIT_SAMPLE_COUNT)
                {
                    data1[init_index] =
                        ch1_uv;

                    data2[init_index] =
                        ch2_uv;

                    init_index++;
                }
            }
            else if (
                sample_count >
                INIT_SAMPLE_END
            )
            {
                init_data_collected =
                    1U;

                printf(
                    "ADS1292 init data "
                    "collection complete\r\n"
                );
            }
        }

        /* ====================================================
         * Periodic UART print
         *
         * NEW:
         * interval = 23, not 25.
         * ==================================================== */

#if ADS1292_PERIODIC_EEG_PRINT_ENABLE

        if (
            (
                sample_count %
                ADS1292_PRINT_INTERVAL
            )
            ==
            0U
        )
        {
#if ADS1292_INTERNAL_TEST

            printf(
                "TEST "
                "N=%lu "
                "CH2=%lduV "
                "RAW=%ld "
                "OVR=%lu\r\n",

                (unsigned long)sample_count,
                (long)ch2_uv,
                (long)frame.ch2,
                (unsigned long)
                m_ads1292_overrun_count
            );

#else

            printf(
                "EEG "
                "N=%lu "
                "CH2=%lduV "
                "RAW=%ld "
                "OVR=%lu\r\n",

                (unsigned long)sample_count,
                (long)ch2_uv,
                (long)frame.ch2,
                (unsigned long)
                m_ads1292_overrun_count
            );

#endif
        }

#endif /* ADS1292_PERIODIC_EEG_PRINT_ENABLE */
    }
}


/* ============================================================
 * Standalone test entry
 * ============================================================ */

int ads1292_main(void)
{
    device_init();

    if (!m_spi_initialized)
    {
        return -1;
    }

    ads1292_init();

    if (!m_ads1292_ready)
    {
        return -2;
    }

    return 0;
}
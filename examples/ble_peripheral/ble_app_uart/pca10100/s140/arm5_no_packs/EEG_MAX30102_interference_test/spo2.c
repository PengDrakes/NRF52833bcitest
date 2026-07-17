#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrfx_gpiote.h"

#include "FreeRTOS.h"
#include "task.h"

#include "spo2.h"
#include "max30102_fir.h"
#include "bluetooth.h"
#include "interference_test.h"


/* ============================================================
 * Experiment configuration
 * ============================================================ */

/* Duration of each experimental phase. */
#define MAX30102_TEST_STAGE_SECONDS       60U

/* Ignore the first N seconds after each mode transition in analysis. */
#define MAX30102_TEST_SETTLE_SECONDS      5U

/* 1 = repeat the five-stage sequence forever; 0 = stop after one cycle. */
#define MAX30102_TEST_REPEAT              0U

/* Disable slow I2C scanning during normal experiments. */
#define MAX30102_ENABLE_I2C_SCAN          0U

/* Normal LED current used by the original project. */
#define MAX30102_LED_CURRENT_NORMAL       0x3FU

/* MAX30102 address. */
#define MAX30102_ADDR                     0x57U

/* Contact threshold retained from the original project. */
#define PPG_DATA_THRESHOLD                100000UL

/* Number of filtered PPG samples used by the original SpO2 algorithm. */
#define CACHE_NUMS                        300U

/* FIFO output rate after averaging is expected to be approximately 50 Hz. */
#define MAX30102_TASK_IDLE_DELAY_MS       1U


/* ============================================================
 * Public compatibility variables
 * ============================================================ */

uint32_t spo2 = 0U;
int spo2_data_ready = 0;

volatile int max30102_int_flag = 0;


/* ============================================================
 * Public experiment state
 * ============================================================ */

volatile uint8_t  g_max30102_test_mode = (uint8_t)MAX30102_TEST_OFF;
volatile uint8_t  g_max30102_test_phase = 0U;
volatile uint32_t g_max30102_test_cycle = 0U;
volatile uint32_t g_max30102_mode_epoch = 0U;
volatile uint32_t g_max30102_mode_start_tick = 0U;


/* ============================================================
 * Internal state
 * ============================================================ */

static const nrf_drv_twi_t m_twi_master =
    NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

static uint8_t fifo_data[6];
static float max30102_data[2];
static float fir_output[2];

static float ppg_data_cache_RED[CACHE_NUMS];
static float ppg_data_cache_IR[CACHE_NUMS];

static bool s_twi_initialized = false;
static bool s_sensor_initialized = false;

static uint8_t s_stage_index = 0U;
static TickType_t s_stage_start_tick = 0U;
static bool s_sequence_finished = false;


/*
 * Required automatic experiment sequence:
 *
 *     OFF
 *       -> DIGITAL_ONLY
 *       -> BOTH
 *       -> DIGITAL_ONLY
 *       -> OFF
 */
static const max30102_test_mode_t s_test_sequence[] =
{
    MAX30102_TEST_OFF,
    MAX30102_TEST_DIGITAL_ONLY,
    MAX30102_TEST_BOTH,
    MAX30102_TEST_DIGITAL_ONLY,
    MAX30102_TEST_OFF
};

#define MAX30102_TEST_STAGE_COUNT \
    ((uint8_t)(sizeof(s_test_sequence) / sizeof(s_test_sequence[0])))


/* ============================================================
 * Helpers
 * ============================================================ */

static const char *max30102_mode_name(max30102_test_mode_t mode)
{
    switch (mode)
    {
        case MAX30102_TEST_OFF:
            return "OFF";

        case MAX30102_TEST_DIGITAL_ONLY:
            return "DIGITAL_ONLY";

        case MAX30102_TEST_RED_ONLY:
            return "RED_ONLY";

        case MAX30102_TEST_IR_ONLY:
            return "IR_ONLY";

        case MAX30102_TEST_BOTH:
            return "BOTH";

        default:
            return "UNKNOWN";
    }
}


static bool max30102_mode_is_active(max30102_test_mode_t mode)
{
    return mode != MAX30102_TEST_OFF;
}


static void max30102_clear_ppg_cache(void)
{
    memset(ppg_data_cache_RED, 0, sizeof(ppg_data_cache_RED));
    memset(ppg_data_cache_IR, 0, sizeof(ppg_data_cache_IR));
}


/* ============================================================
 * TWI initialization
 * ============================================================ */

static ret_code_t twi_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t config =
    {
        .scl                = TWI_SCL_M,
        .sda                = TWI_SDA_M,
        .frequency          = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = true,
        .hold_bus_uninit    = false
    };

    if (s_twi_initialized)
    {
        return NRF_SUCCESS;
    }

    ret = nrf_drv_twi_init(
        &m_twi_master,
        &config,
        NULL,
        NULL
    );

    if (ret == NRF_SUCCESS)
    {
        nrf_drv_twi_enable(&m_twi_master);
        s_twi_initialized = true;
        printf("MAX30102 I2C init success\r\n");
    }
    else if (ret == NRF_ERROR_INVALID_STATE)
    {
        /*
         * The project may already have initialized this instance.
         * Keep the existing instance enabled.
         */
        s_twi_initialized = true;
        printf("MAX30102 I2C already initialized\r\n");
        ret = NRF_SUCCESS;
    }
    else
    {
        printf(
            "MAX30102 I2C init failed err=0x%lX\r\n",
            (unsigned long)ret
        );
    }

    return ret;
}


/* ============================================================
 * MAX30102 interrupt
 * ============================================================ */

static void int_isr_handler(
    nrfx_gpiote_pin_t pin,
    nrf_gpiote_polarity_t action
)
{
    if (
        (pin == INT_PIN) &&
        (action == NRF_GPIOTE_POLARITY_HITOLO)
    )
    {
        max30102_int_flag = 1;
    }
}


static ret_code_t int_pin_config(void)
{
    ret_code_t err;

    nrfx_gpiote_in_config_t in_config =
        NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);

    err = nrfx_gpiote_init();

    if (
        (err != NRF_SUCCESS) &&
        (err != NRFX_ERROR_INVALID_STATE)
    )
    {
        return err;
    }

    nrf_gpio_cfg_input(
        INT_PIN,
        NRF_GPIO_PIN_PULLUP
    );

    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err = nrfx_gpiote_in_init(
        INT_PIN,
        &in_config,
        int_isr_handler
    );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrfx_gpiote_in_event_enable(
        INT_PIN,
        true
    );

    printf("MAX30102 INT pin configured\r\n");

    return NRF_SUCCESS;
}


/* ============================================================
 * Low-level I2C
 * ============================================================ */

ret_code_t max30102_i2c_write(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t buffer[2];
    ret_code_t result;

    buffer[0] = reg;
    buffer[1] = value;

    result = nrf_drv_twi_tx(
        &m_twi_master,
        MAX30102_ADDR,
        buffer,
        sizeof(buffer),
        false
    );

    if (result != NRF_SUCCESS)
    {
        printf(
            "MAX30102 write failed reg=0x%02X err=0x%lX\r\n",
            reg,
            (unsigned long)result
        );
    }

    return result;
}


ret_code_t max30102_i2c_read(
    uint8_t reg,
    uint8_t *p_value,
    uint8_t length
)
{
    ret_code_t err_code;

    if (
        (p_value == NULL) ||
        (length == 0U)
    )
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = nrf_drv_twi_tx(
        &m_twi_master,
        MAX30102_ADDR,
        &reg,
        1U,
        true
    );

    if (err_code != NRF_SUCCESS)
    {
        printf(
            "MAX30102 read-TX failed reg=0x%02X err=0x%lX\r\n",
            reg,
            (unsigned long)err_code
        );

        return err_code;
    }

    err_code = nrf_drv_twi_rx(
        &m_twi_master,
        MAX30102_ADDR,
        p_value,
        length
    );

    if (err_code != NRF_SUCCESS)
    {
        printf(
            "MAX30102 read-RX failed reg=0x%02X err=0x%lX\r\n",
            reg,
            (unsigned long)err_code
        );
    }

    return err_code;
}


/* ============================================================
 * Optional I2C scan
 * ============================================================ */

#if MAX30102_ENABLE_I2C_SCAN

static ret_code_t i2c_probe_address(uint8_t address)
{
    uint8_t probe = 0U;

    return nrf_drv_twi_tx(
        &m_twi_master,
        address,
        &probe,
        1U,
        false
    );
}


static void scan_i2c_bus(void)
{
    uint8_t address;
    bool found = false;

    printf("I2C scan start\r\n");

    for (address = 0x03U; address < 0x78U; address++)
    {
        ret_code_t err = i2c_probe_address(address);

        if (err == NRF_SUCCESS)
        {
            printf("I2C device: 0x%02X\r\n", address);
            found = true;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    if (!found)
    {
        printf("I2C scan found no device\r\n");
    }
}

#endif /* MAX30102_ENABLE_I2C_SCAN */


/* ============================================================
 * FIFO
 * ============================================================ */

void max30102_fifo_read(float *output_data)
{
    ret_code_t err_code;
    uint32_t data_ir;
    uint32_t data_red;

    if (output_data == NULL)
    {
        return;
    }

    err_code = max30102_i2c_read(
        FIFO_DATA_REG,
        fifo_data,
        sizeof(fifo_data)
    );

    if (err_code != NRF_SUCCESS)
    {
        output_data[0] = 0.0f;
        output_data[1] = 0.0f;
        return;
    }

    data_ir =
        (
            ((uint32_t)fifo_data[0] << 16U) |
            ((uint32_t)fifo_data[1] << 8U)  |
            ((uint32_t)fifo_data[2])
        ) &
        0x03FFFFUL;

    data_red =
        (
            ((uint32_t)fifo_data[3] << 16U) |
            ((uint32_t)fifo_data[4] << 8U)  |
            ((uint32_t)fifo_data[5])
        ) &
        0x03FFFFUL;

    output_data[0] = (float)data_ir;
    output_data[1] = (float)data_red;
}


static void max30102_clear_interrupt_status(void)
{
    uint8_t status1 = 0U;
    uint8_t status2 = 0U;

    (void)max30102_i2c_read(
        INTERRUPT_STATUS1,
        &status1,
        1U
    );

    (void)max30102_i2c_read(
        INTERRUPT_STATUS2,
        &status2,
        1U
    );
}


static void max30102_clear_fifo(void)
{
    (void)max30102_i2c_write(FIFO_WR_POINTER, 0x00U);
    (void)max30102_i2c_write(FIFO_OV_COUNTER, 0x00U);
    (void)max30102_i2c_write(FIFO_RD_POINTER, 0x00U);
}


/* ============================================================
 * Sensor initialization
 * ============================================================ */

void max30102_init(void)
{
    ret_code_t result;

    s_sensor_initialized = false;

    /* Reset. */
    result = max30102_i2c_write(
        MODE_CONFIGURATION,
        0x40U
    );

    if (result != NRF_SUCCESS)
    {
        printf("MAX30102 reset failed\r\n");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10U));

    /* Interrupts: FIFO almost full, new PPG data, ALC overflow. */
    if (max30102_i2c_write(INTERRUPT_ENABLE1, 0xE0U) != NRF_SUCCESS)
    {
        return;
    }

    if (max30102_i2c_write(INTERRUPT_ENABLE2, 0x00U) != NRF_SUCCESS)
    {
        return;
    }

    max30102_clear_fifo();

    /*
     * FIFO averaging = 4, rollover enabled, almost-full threshold = 15.
     * Retained from the original project.
     */
    if (max30102_i2c_write(FIFO_CONFIGURATION, 0x4FU) != NRF_SUCCESS)
    {
        return;
    }

    /*
     * SPO2_CONFIGURATION = 0x2A:
     * range=4096 nA, sample rate=200 SPS, pulse width=215 us.
     */
    if (max30102_i2c_write(SPO2_CONFIGURATION, 0x2AU) != NRF_SUCCESS)
    {
        return;
    }

    /* Start safely with both LEDs disabled. */
    if (max30102_i2c_write(LED1_PULSE_AMPLITUDE, 0x00U) != NRF_SUCCESS)
    {
        return;
    }

    if (max30102_i2c_write(LED2_PULSE_AMPLITUDE, 0x00U) != NRF_SUCCESS)
    {
        return;
    }

    /* Enter shutdown until the experiment controller selects a mode. */
    if (max30102_i2c_write(MODE_CONFIGURATION, 0x80U) != NRF_SUCCESS)
    {
        return;
    }

    max30102_clear_interrupt_status();

    max30102_fir_init();
    max30102_clear_ppg_cache();

    s_sensor_initialized = true;

    printf(
        "MAX30102 configured: SR=200 FIFO_AVG=4 LED=0 shutdown\r\n"
    );
}


void test_i2c_communication(void)
{
    uint8_t part_id = 0U;
    ret_code_t ret;

    if (nrf_drv_twi_is_busy(&m_twi_master))
    {
        printf("MAX30102 TWI busy\r\n");
        return;
    }

    ret = max30102_i2c_read(
        PART_ID,
        &part_id,
        1U
    );

    if (ret == NRF_SUCCESS)
    {
        printf("MAX30102 PART_ID=0x%02X\r\n", part_id);
    }
    else
    {
        printf(
            "MAX30102 PART_ID read failed err=0x%lX\r\n",
            (unsigned long)ret
        );
    }
}


/* ============================================================
 * Mode control
 * ============================================================ */

ret_code_t max30102_set_test_mode(max30102_test_mode_t mode)
{
    ret_code_t err;
    uint8_t red_current = 0x00U;
    uint8_t ir_current = 0x00U;
    uint8_t mode_value = 0x80U;
    TickType_t now;

    if (!s_sensor_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    switch (mode)
    {
        case MAX30102_TEST_OFF:
        {
            red_current = 0x00U;
            ir_current = 0x00U;
            mode_value = 0x80U;
            break;
        }

        case MAX30102_TEST_DIGITAL_ONLY:
        {
            red_current = 0x00U;
            ir_current = 0x00U;
            mode_value = 0x03U;
            break;
        }

        case MAX30102_TEST_RED_ONLY:
        {
            red_current = MAX30102_LED_CURRENT_NORMAL;
            ir_current = 0x00U;
            mode_value = 0x03U;
            break;
        }

        case MAX30102_TEST_IR_ONLY:
        {
            red_current = 0x00U;
            ir_current = MAX30102_LED_CURRENT_NORMAL;
            mode_value = 0x03U;
            break;
        }

        case MAX30102_TEST_BOTH:
        {
            red_current = MAX30102_LED_CURRENT_NORMAL;
            ir_current = MAX30102_LED_CURRENT_NORMAL;
            mode_value = 0x03U;
            break;
        }

        default:
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }

    /* Stop conversions during transition. */
    err = max30102_i2c_write(
        MODE_CONFIGURATION,
        0x80U
    );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(5U));

    err = max30102_i2c_write(
        LED1_PULSE_AMPLITUDE,
        red_current
    );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = max30102_i2c_write(
        LED2_PULSE_AMPLITUDE,
        ir_current
    );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    max30102_clear_fifo();
    max30102_clear_interrupt_status();
    max30102_int_flag = 0;

    err = max30102_i2c_write(
        MODE_CONFIGURATION,
        mode_value
    );

    if (err != NRF_SUCCESS)
    {
        return err;
    }

    now = xTaskGetTickCount();

    taskENTER_CRITICAL();

    g_max30102_test_mode = (uint8_t)mode;
    g_max30102_mode_start_tick = (uint32_t)now;
    g_max30102_mode_epoch++;

    taskEXIT_CRITICAL();

    printf(
        "XMARK cycle=%lu phase=%u mode=%u name=%s tick=%lu "
        "red=0x%02X ir=0x%02X settle_s=%u\r\n",
        (unsigned long)g_max30102_test_cycle,
        (unsigned int)g_max30102_test_phase,
        (unsigned int)mode,
        max30102_mode_name(mode),
        (unsigned long)now,
        red_current,
        ir_current,
        (unsigned int)MAX30102_TEST_SETTLE_SECONDS
    );

    return NRF_SUCCESS;
}


static bool max30102_start_current_stage(void)
{
    ret_code_t err;
    max30102_test_mode_t mode;

    mode = s_test_sequence[s_stage_index];

    taskENTER_CRITICAL();
    g_max30102_test_phase = s_stage_index;
    taskEXIT_CRITICAL();

    err = max30102_set_test_mode(mode);

    if (err != NRF_SUCCESS)
    {
        printf(
            "XMARK switch failed cycle=%lu phase=%u mode=%u err=0x%lX\r\n",
            (unsigned long)g_max30102_test_cycle,
            (unsigned int)s_stage_index,
            (unsigned int)mode,
            (unsigned long)err
        );

        return false;
    }

    s_stage_start_tick = xTaskGetTickCount();

    printf(
        "XSTAGE cycle=%lu phase=%u/%u duration_s=%u\r\n",
        (unsigned long)g_max30102_test_cycle,
        (unsigned int)(s_stage_index + 1U),
        (unsigned int)MAX30102_TEST_STAGE_COUNT,
        (unsigned int)MAX30102_TEST_STAGE_SECONDS
    );

    return true;
}


static void max30102_test_sequence_start(void)
{
    s_stage_index = 0U;
    s_sequence_finished = false;

    taskENTER_CRITICAL();
    g_max30102_test_cycle = 1U;
    taskEXIT_CRITICAL();

    (void)max30102_start_current_stage();
}


static void max30102_test_sequence_update(void)
{
    TickType_t now;
    TickType_t elapsed;
    TickType_t stage_ticks;

    if (s_sequence_finished)
    {
        return;
    }

    now = xTaskGetTickCount();
    elapsed = now - s_stage_start_tick;
    stage_ticks = pdMS_TO_TICKS(
        MAX30102_TEST_STAGE_SECONDS * 1000UL
    );

    if (elapsed < stage_ticks)
    {
        return;
    }

    s_stage_index++;

    if (s_stage_index >= MAX30102_TEST_STAGE_COUNT)
    {
#if MAX30102_TEST_REPEAT

        s_stage_index = 0U;

        taskENTER_CRITICAL();
        g_max30102_test_cycle++;
        taskEXIT_CRITICAL();

#else

        s_sequence_finished = true;
        s_stage_index = MAX30102_TEST_STAGE_COUNT - 1U;

        printf(
            "XEND cycle=%lu tick=%lu\r\n",
            (unsigned long)g_max30102_test_cycle,
            (unsigned long)now
        );

        return;

#endif
    }

    (void)max30102_start_current_stage();
}


/* ============================================================
 * Original HR / SpO2 estimators with bounds checks
 * ============================================================ */

uint16_t max30102_getHeartRate(
    float *input_data,
    uint16_t cache_nums
)
{
    float mean = 0.0f;
    uint16_t i;
    uint16_t first_crossing = 0U;
    uint16_t interval = 0U;
    bool first_found = false;

    if (
        (input_data == NULL) ||
        (cache_nums < 3U)
    )
    {
        return 0U;
    }

    for (i = 0U; i < cache_nums; i++)
    {
        mean += input_data[i];
    }

    mean /= (float)cache_nums;

    for (i = 0U; (i + 1U) < cache_nums; i++)
    {
        if (
            (input_data[i] > mean) &&
            (input_data[i + 1U] <= mean)
        )
        {
            if (!first_found)
            {
                first_crossing = i;
                first_found = true;
            }
            else
            {
                interval = i - first_crossing;
                break;
            }
        }
    }

    if (
        (interval > 15U) &&
        (interval < 150U)
    )
    {
        return (uint16_t)(3000U / interval);
    }

    return 0U;
}


float max30102_getSpO2(
    float *ir_input_data,
    float *red_input_data,
    uint16_t cache_nums
)
{
    float ir_max;
    float ir_min;
    float red_max;
    float red_min;
    float ir_ac;
    float red_ac;
    float denominator;
    float ratio;
    uint16_t i;

    if (
        (ir_input_data == NULL) ||
        (red_input_data == NULL) ||
        (cache_nums < 2U)
    )
    {
        return 0.0f;
    }

    ir_max = ir_input_data[0];
    ir_min = ir_input_data[0];
    red_max = red_input_data[0];
    red_min = red_input_data[0];

    for (i = 1U; i < cache_nums; i++)
    {
        if (ir_input_data[i] > ir_max)
        {
            ir_max = ir_input_data[i];
        }

        if (ir_input_data[i] < ir_min)
        {
            ir_min = ir_input_data[i];
        }

        if (red_input_data[i] > red_max)
        {
            red_max = red_input_data[i];
        }

        if (red_input_data[i] < red_min)
        {
            red_min = red_input_data[i];
        }
    }

    ir_ac = ir_max - ir_min;
    red_ac = red_max - red_min;
    denominator = red_ac * ir_min;

    if (
        (ir_ac <= 0.0f) ||
        (red_ac <= 0.0f) ||
        (ir_min <= 0.0f) ||
        (red_min <= 0.0f) ||
        (denominator == 0.0f)
    )
    {
        return 0.0f;
    }

    ratio =
        (ir_ac * red_min) /
        denominator;

    return
        (-45.060f * ratio * ratio) +
        (30.354f * ratio) +
        94.845f;
}


static void send_spo2_to_ble(uint16_t sample_count)
{
    int32_t spo2_value;
    char ble_buffer[20];
    int ble_len;

    spo2_value = (int32_t)max30102_getSpO2(
        ppg_data_cache_IR,
        ppg_data_cache_RED,
        sample_count
    );

    if (spo2_value < 50)
    {
        spo2_value = 50;
    }
    else if (spo2_value > 100)
    {
        spo2_value = 100;
    }

    spo2 = (uint32_t)spo2_value;

    ble_len = snprintf(
        ble_buffer,
        sizeof(ble_buffer),
        "SPO2:%ld\n",
        (long)spo2_value
    );

    if (ble_len > 0)
    {
        ble_send_data(
            (uint8_t *)ble_buffer,
            (uint16_t)ble_len
        );
    }
}


/* ============================================================
 * MAX30102 task
 * ============================================================ */

void spo2_read(void *pvParameter)
{
    ret_code_t err;
    uint16_t cache_counter = 0U;
    uint16_t no_touch_counter = 0U;
    max30102_test_mode_t mode;

    (void)pvParameter;

    err = twi_master_init();

    if (err != NRF_SUCCESS)
    {
        printf("MAX30102 task stopped: I2C init failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100U));

    err = int_pin_config();

    if (err != NRF_SUCCESS)
    {
        printf(
            "MAX30102 task stopped: INT init failed err=0x%lX\r\n",
            (unsigned long)err
        );

        vTaskDelete(NULL);
        return;
    }

#if MAX30102_ENABLE_I2C_SCAN
    scan_i2c_bus();
#endif

    test_i2c_communication();
    max30102_init();

    if (!s_sensor_initialized)
    {
        printf("MAX30102 task stopped: sensor init failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    max30102_test_sequence_start();

    for (;;)
    {
        max30102_test_sequence_update();

        mode = (max30102_test_mode_t)g_max30102_test_mode;

        /* OFF mode intentionally performs no sensor bus traffic. */
        if (!max30102_mode_is_active(mode))
        {
            max30102_int_flag = 0;
            cache_counter = 0U;
            no_touch_counter = 0U;

            vTaskDelay(pdMS_TO_TICKS(MAX30102_TASK_IDLE_DELAY_MS));
            continue;
        }

        if (max30102_int_flag != 0)
        {
            max30102_clear_interrupt_status();
            max30102_int_flag = 0;

            max30102_fifo_read(max30102_data);

            /*
             * Keep FIR execution in every active mode so DIGITAL_ONLY
             * has nearly the same software path as BOTH.
             */
            ir_max30102_fir(
                &max30102_data[0],
                &fir_output[0]
            );

            red_max30102_fir(
                &max30102_data[1],
                &fir_output[1]
            );

            /* Only BOTH mode produces a valid SpO2 pair. */
            if (mode == MAX30102_TEST_BOTH)
            {
                if (
                    (max30102_data[0] > (float)PPG_DATA_THRESHOLD) &&
                    (max30102_data[1] > (float)PPG_DATA_THRESHOLD)
                )
                {
                    ppg_data_cache_IR[cache_counter] = fir_output[0];
                    ppg_data_cache_RED[cache_counter] = fir_output[1];

                    cache_counter++;
                    no_touch_counter = 0U;

                    if (cache_counter >= CACHE_NUMS)
                    {
                        uint16_t heart_rate;
                        int32_t spo2_value;

                        send_spo2_to_ble(CACHE_NUMS);

                        heart_rate = max30102_getHeartRate(
                            ppg_data_cache_IR,
                            CACHE_NUMS
                        );

                        spo2_value = (int32_t)max30102_getSpO2(
                            ppg_data_cache_IR,
                            ppg_data_cache_RED,
                            CACHE_NUMS
                        );

                        printf(
                            "PPG cycle=%lu phase=%u HR=%u SPO2=%ld\r\n",
                            (unsigned long)g_max30102_test_cycle,
                            (unsigned int)g_max30102_test_phase,
                            (unsigned int)heart_rate,
                            (long)spo2_value
                        );

                        cache_counter = 0U;
                    }
                }
                else
                {
                    no_touch_counter++;

                    if (no_touch_counter >= 5U)
                    {
                        cache_counter = 0U;
                    }
                }
            }
            else
            {
                /* Do not carry PPG data between experimental modes. */
                cache_counter = 0U;
                no_touch_counter = 0U;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MAX30102_TASK_IDLE_DELAY_MS));
    }
}

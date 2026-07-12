#ifndef __ADS1292_H__
#define __ADS1292_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "app_error.h"

#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"


/* ============================================================
 * ADS1292R <-> nRF52833 pin mapping
 *
 * Fixed according to the current working SPI mapping and the
 * corrected RESET mapping used for this diagnostic build.
 * ============================================================ */

/* DRDY# -> P0.31 */
#define ADS1292_DRDY_PIN      NRF_GPIO_PIN_MAP(0, 31)

/* DOUT -> P0.29 */
#define ADS1292_MISO_PIN      NRF_GPIO_PIN_MAP(0, 29)

/* SCLK -> P0.02 */
#define ADS1292_SCK_PIN       NRF_GPIO_PIN_MAP(0, 2)

/* DIN -> P1.05 */
#define ADS1292_MOSI_PIN      NRF_GPIO_PIN_MAP(1, 5)

/* CS# -> P0.28 */
#define ADS1292_CS_PIN        NRF_GPIO_PIN_MAP(0, 28)

/* START -> P0.03 */
#define ADS1292_START_PIN     NRF_GPIO_PIN_MAP(0, 3)

/* PWDN#/RESET# -> corrected GPIO mapping P0.25 */
#define ADS1292_RESET_PIN     NRF_GPIO_PIN_MAP(0, 25)


/* ============================================================
 * ADS1292R configuration
 * ============================================================ */

/*
 * SPI2 is used by this driver.
 * Required sdk_config.h settings:
 *
 * SPI_ENABLED  = 1
 * SPI2_ENABLED = 1
 */
#define ADS1292_SPI_INSTANCE       2

/* CH2 PGA gain in real EEG mode */
#define ADS1292_CH2_GAIN           12U

/* Internal reference voltage */
#define ADS1292_VREF_VOLTS         2.42f

/* Expected ADS1292R device ID */
#define ADS1292_EXPECTED_ID        0x73U


/* ============================================================
 * ADS1292R RLD configuration
 * ============================================================ */

/*
 * 1 = Enable RLD in real EEG mode
 * 0 = Disable RLD
 *
 * IMPORTANT:
 * Enable this only when the third body electrode is physically
 * connected to the board's RLD external electrode path.
 *
 * Current board:
 *     IN2P    -> EEG measurement electrode
 *     IN2N    -> EEG reference electrode
 *     EXT_INV -> third RLD body electrode
 */
#define ADS1292_ENABLE_RLD         1U

/*
 * RLD_SENS = 0x2C
 *
 * Register address: 0x06
 *
 * bit 7:6 CHOP[1:0] = 00
 * bit 5   PDB_RLD   = 1
 *         Enable RLD buffer
 *
 * bit 4   RLD_LOFF_SENS = 0
 *         RLD lead-off sensing disabled
 *
 * bit 3   RLD2N = 1
 *         CH2 negative input IN2N participates
 *         in RLD derivation
 *
 * bit 2   RLD2P = 1
 *         CH2 positive input IN2P participates
 *         in RLD derivation
 *
 * bit 1   RLD1N = 0
 * bit 0   RLD1P = 0
 *
 * Binary:
 *     0010 1100 = 0x2C
 */
#define ADS1292_RLD_SENS_CH2       0x2CU

/*
 * RESP2 = 0x03
 *
 * Register address: 0x0A
 *
 * bit 7 CALIB_ON   = 0
 * bit 6:3          = 0
 * bit 2 RESP_FREQ  = 0
 * bit 1 RLDREF_INT = 1
 * bit 0            = 1, required
 *
 * Internal RLD reference selected.
 */
#define ADS1292_RESP2_RLD_INT      0x03U


/* ============================================================
 * ADS1292R commands
 * ============================================================ */

#define ADS1292_CMD_WAKEUP         0x02U
#define ADS1292_CMD_STANDBY        0x04U
#define ADS1292_CMD_RESET          0x06U
#define ADS1292_CMD_START          0x08U
#define ADS1292_CMD_STOP           0x0AU

#define ADS1292_CMD_RDATAC         0x10U
#define ADS1292_CMD_SDATAC         0x11U
#define ADS1292_CMD_RDATA          0x12U

#define ADS1292_CMD_OFFSETCAL      0x1AU

#define ADS1292_CMD_RREG           0x20U
#define ADS1292_CMD_WREG           0x40U


/* ============================================================
 * ADS1292R register addresses
 * ============================================================ */

#define ADS1292_REG_ID             0x00U
#define ADS1292_REG_CONFIG1        0x01U
#define ADS1292_REG_CONFIG2        0x02U
#define ADS1292_REG_LOFF           0x03U
#define ADS1292_REG_CH1SET         0x04U
#define ADS1292_REG_CH2SET         0x05U
#define ADS1292_REG_RLD_SENS       0x06U
#define ADS1292_REG_LOFF_SENS      0x07U
#define ADS1292_REG_LOFF_STAT      0x08U
#define ADS1292_REG_RESP1          0x09U
#define ADS1292_REG_RESP2          0x0AU
#define ADS1292_REG_GPIO           0x0BU


/* ============================================================
 * Data buffer configuration
 * ============================================================ */

#define CYC_ARRAY_LEN              2000U

#define INIT_SAMPLE_COUNT          1000U
#define INIT_SAMPLE_START          1U
#define INIT_SAMPLE_END            1000U


/* ============================================================
 * ADS1292R data frame
 * ============================================================ */

typedef struct
{
    uint32_t status;
    int32_t  ch1;
    int32_t  ch2;

} ads1292_frame_t;


/* ============================================================
 * Existing project buffers
 * ============================================================ */

extern int32_t save_data[CYC_ARRAY_LEN];
extern uint32_t write_pos;
extern uint32_t read_pos;

extern int32_t save_data_0[CYC_ARRAY_LEN];
extern uint32_t write_pos_0;
extern uint32_t read_pos_0;

extern int32_t save_data_1[CYC_ARRAY_LEN];
extern uint32_t write_pos_1;
extern uint32_t read_pos_1;

extern volatile uint8_t init_data_collected;

extern float32_t data1[INIT_SAMPLE_COUNT];
extern float32_t data2[INIT_SAMPLE_COUNT];


/* ============================================================
 * Legacy-compatible API
 * ============================================================ */

void spi_event_handler(
    nrf_drv_spi_evt_t const *p_event,
    void *p_context
);

void device_init(void);

void ads1292_init(void);

void ads1292_standby(void);

void ads1292_wakeup(void);

void spi_read(void *pvParameter);

void ADS1292_val_init(
    float32_t *data,
    float32_t *a,
    float32_t *b
);

void bre_val_init(
    float32_t *data,
    float32_t *a,
    float32_t *b
);

int ads1292_main(void);


/* ============================================================
 * Low-level driver API
 * ============================================================ */

ret_code_t ads1292_send_cmd(uint8_t cmd);

ret_code_t ads1292_write_reg(
    uint8_t reg,
    uint8_t value
);

ret_code_t ads1292_read_reg(
    uint8_t reg,
    uint8_t *value
);

ret_code_t ads1292_start_sampling(void);

ret_code_t ads1292_stop_sampling(void);

ret_code_t ads1292_read_frame(
    ads1292_frame_t *frame
);

int32_t ads1292_sign_extend_24(
    const uint8_t data[3]
);

float ads1292_code_to_uv(
    int32_t code,
    uint8_t gain
);


#endif /* __ADS1292_H__ */
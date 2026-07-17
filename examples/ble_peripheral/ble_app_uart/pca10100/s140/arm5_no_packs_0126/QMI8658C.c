#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_util_platform.h"
#include "bluetooth.h"
#include "FreeRTOS.h"
#include "nrf_drv_twi.h"
#include "QMI8658C.h"
#include "task.h"
#include "../../../debug_print.h"

#define QMI8658C_ADDR           0x6B
#define QMI8658C_WHO_AM_I_REG   0x00
#define QMI8658C_CTRL1_REG      0x02
#define QMI8658C_CTRL2_REG      0x03
#define QMI8658C_CTRL3_REG      0x04
#define QMI8658C_CTRL5_REG      0x06
#define QMI8658C_CTRL7_REG      0x08
#define QMI8658C_STATUS0_REG    0x2E
#define QMI8658C_DATA_REG       0x35
#define QMI8658C_RESET_REG      0x60

#define QMI8658C_WHO_AM_I_VALUE 0x05
#define QMI8658C_SAMPLE_BYTES   12
#define QMI8658C_PACKET_BYTES   14

/* Packet: A5 02 AX_L AX_H AY_L AY_H AZ_L AZ_H GX_L GX_H GY_L GY_H GZ_L GZ_H */
#define QMI8658C_PACKET_HEADER  0xA5
#define QMI8658C_PACKET_TYPE    0x02

static const nrf_drv_twi_t m_qmi_twi = NRF_DRV_TWI_INSTANCE(QMI_TWI_INST);

static ret_code_t qmi_twi_master_init(void)
{
    const nrf_drv_twi_config_t config =
    {
        .scl                = QMI_SCL,
        .sda                = QMI_SDA,
        .frequency          = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
        .clear_bus_init     = true,
        .hold_bus_uninit    = false
    };

    ret_code_t err_code = nrf_drv_twi_init(&m_qmi_twi, &config, NULL, NULL);
    if (err_code == NRF_SUCCESS)
    {
        nrf_drv_twi_enable(&m_qmi_twi);
    }

    return err_code;
}

static ret_code_t qmi8658c_i2c_write(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    return nrf_drv_twi_tx(&m_qmi_twi,
                          QMI8658C_ADDR,
                          buffer,
                          sizeof(buffer),
                          false);
}

static ret_code_t qmi8658c_i2c_read(uint8_t reg, uint8_t * p_value, uint8_t length)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_qmi_twi,
                              QMI8658C_ADDR,
                              &reg,
                              1,
                              true);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return nrf_drv_twi_rx(&m_qmi_twi,
                          QMI8658C_ADDR,
                          p_value,
                          length);
}

static ret_code_t qmi8658c_init(void)
{
    uint8_t id = 0;
    ret_code_t err_code;

    /* The QMI8658C datasheet specifies 0xB0 for software reset. */
    err_code = qmi8658c_i2c_write(QMI8658C_RESET_REG, 0xB0);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    err_code = qmi8658c_i2c_read(QMI8658C_WHO_AM_I_REG, &id, 1);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    printf("QMI8658 WHO_AM_I=0x%02X\r\n", id);
    if (id != QMI8658C_WHO_AM_I_VALUE)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    /* CTRL1: address auto-increment enabled, little-endian output. */
    err_code = qmi8658c_i2c_write(QMI8658C_CTRL1_REG, 0x40);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* CTRL2: accelerometer +/-16 g, approximately 112 Hz in 6-DOF mode. */
    err_code = qmi8658c_i2c_write(QMI8658C_CTRL2_REG, 0x36);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* CTRL3: gyroscope +/-512 dps, approximately 112 Hz. */
    err_code = qmi8658c_i2c_write(QMI8658C_CTRL3_REG, 0x56);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* CTRL5: enable accelerometer and gyroscope low-pass filters. */
    err_code = qmi8658c_i2c_write(QMI8658C_CTRL5_REG, 0x11);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* CTRL7: enable accelerometer and gyroscope. */
    err_code = qmi8658c_i2c_write(QMI8658C_CTRL7_REG, 0x03);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /* Allow the gyro and filters to settle before the first sample. */
    vTaskDelay(pdMS_TO_TICKS(200));
    return NRF_SUCCESS;
}

void qmi8658c_read(void * pvParameter)
{
    uint8_t status = 0;
    uint8_t data[QMI8658C_SAMPLE_BYTES];
    uint8_t packet[QMI8658C_PACKET_BYTES] =
    {
        QMI8658C_PACKET_HEADER,
        QMI8658C_PACKET_TYPE
    };
    uint8_t send_counter = 0;
    ret_code_t err_code;

    (void)pvParameter;

    err_code = qmi_twi_master_init();
    if (err_code != NRF_SUCCESS)
    {
        printf("QMI TWI init failed: 0x%X\r\n", err_code);
        vTaskDelete(NULL);
        return;
    }

    err_code = qmi8658c_init();
    if (err_code != NRF_SUCCESS)
    {
        printf("QMI init failed: 0x%X\r\n", err_code);
        vTaskDelete(NULL);
        return;
    }

    printf("QMI8658 sampling started\r\n");

    for (;;)
    {
        err_code = qmi8658c_i2c_read(QMI8658C_STATUS0_REG, &status, 1);

        /* STATUS0 bits 0 and 1 indicate new accelerometer and gyro data. */
        if ((err_code == NRF_SUCCESS) && ((status & 0x03) == 0x03))
        {
            err_code = qmi8658c_i2c_read(QMI8658C_DATA_REG,
                                         data,
                                         sizeof(data));
            if (err_code == NRF_SUCCESS)
            {
                /* Sample near 100 Hz and send to Flutter at about 10 Hz. */
                send_counter++;
                if (send_counter >= 10)
                {
                    int16_t ax = (int16_t)(((uint16_t)data[1]  << 8) | data[0]);
                    int16_t ay = (int16_t)(((uint16_t)data[3]  << 8) | data[2]);
                    int16_t az = (int16_t)(((uint16_t)data[5]  << 8) | data[4]);
                    int16_t gx = (int16_t)(((uint16_t)data[7]  << 8) | data[6]);
                    int16_t gy = (int16_t)(((uint16_t)data[9]  << 8) | data[8]);
                    int16_t gz = (int16_t)(((uint16_t)data[11] << 8) | data[10]);

                    send_counter = 0;
                    memcpy(&packet[2], data, sizeof(data));
                    ble_send_data(packet, sizeof(packet));

                    printf("IMU A:%d,%d,%d G:%d,%d,%d\r\n",
                           ax, ay, az, gx, gy, gz);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

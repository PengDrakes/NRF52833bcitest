#ifndef INTERFERENCE_TEST_H
#define INTERFERENCE_TEST_H

#include <stdint.h>

/*
 * MAX30102 / EEG interference experiment modes.
 *
 * OFF:
 *     MAX30102 shutdown, LEDs off.
 *
 * DIGITAL_ONLY:
 *     MAX30102 SpO2 engine running, both LED currents are zero.
 *     Used to observe digital/I2C/task-load interference.
 *
 * RED_ONLY / IR_ONLY:
 *     Reserved manual diagnostic modes.
 *
 * BOTH:
 *     Normal red + IR SpO2 acquisition.
 */
typedef enum
{
    MAX30102_TEST_OFF = 0,
    MAX30102_TEST_DIGITAL_ONLY = 1,
    MAX30102_TEST_RED_ONLY = 2,
    MAX30102_TEST_IR_ONLY = 3,
    MAX30102_TEST_BOTH = 4

} max30102_test_mode_t;

/* Current experiment state, written by spo2.c and read by ads1292.c. */
extern volatile uint8_t  g_max30102_test_mode;
extern volatile uint8_t  g_max30102_test_phase;
extern volatile uint32_t g_max30102_test_cycle;

/*
 * Incremented after every successful mode switch.
 * ADS1292 uses this to discard a partial one-second EEG statistics window.
 */
extern volatile uint32_t g_max30102_mode_epoch;

/* FreeRTOS tick at which the current mode became active. */
extern volatile uint32_t g_max30102_mode_start_tick;

#endif /* INTERFERENCE_TEST_H */

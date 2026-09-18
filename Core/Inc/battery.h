/**
 * @file    battery.h
 * @brief   Battery voltage monitoring.
 */

#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Initialize battery monitoring ADC channel.
 */
void battery_init(void);

/**
 * @brief  Read battery voltage.
 * @return Battery voltage in millivolts.
 */
uint16_t battery_read_mv(void);

/**
 * @brief  Check if battery is low.
 * @return true if voltage below BATTERY_LOW_MV threshold.
 */
bool battery_is_low(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */

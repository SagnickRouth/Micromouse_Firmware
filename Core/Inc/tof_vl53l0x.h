#ifndef TOF_VL53L0X_H
#define TOF_VL53L0X_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t address;
    uint8_t stop_variable;
    uint16_t timeout_ms;
    bool initialized;
} VL53L0X_Device;

bool vl53l0x_init(VL53L0X_Device *dev, I2C_HandleTypeDef *i2c, uint8_t address);
bool vl53l0x_set_address(VL53L0X_Device *dev, uint8_t new_address);
uint16_t vl53l0x_read_range_mm(VL53L0X_Device *dev);
bool vl53l0x_timeout_occurred(VL53L0X_Device *dev);

#endif

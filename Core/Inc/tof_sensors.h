#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H
#include <stdint.h>
#include <stdbool.h>
#include "tof_vl53l0x.h"

typedef struct {
    /*
     * Calibrated beam distance in mm.
     * These values use the per-sensor calibration derived from the user's
     * 50-300 mm reference measurements.
     */
    uint16_t left;
    uint16_t front_left;
    uint16_t front_right;
    uint16_t right;

    /* Unmodified VL53L0X range values, useful for diagnostics. */
    uint16_t left_raw;
    uint16_t front_left_raw;
    uint16_t front_right_raw;
    uint16_t right_raw;

    uint8_t left_status;
    uint8_t front_left_status;
    uint8_t front_right_status;
    uint8_t right_status;
    uint8_t left_address;
    uint8_t front_left_address;
    uint8_t front_right_address;
    uint8_t right_address;
    uint8_t left_model;
    uint8_t front_left_model;
    uint8_t front_right_model;
    uint8_t right_model;
    bool initialized;
} ToFSensors;

bool tof_sensors_init(void);
void tof_sensors_update(void);
const ToFSensors *tof_sensors_get(void);
#endif

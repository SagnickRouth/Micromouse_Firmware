#ifndef TOF_SENSORS_H
#define TOF_SENSORS_H
#include <stdint.h>
#include <stdbool.h>
#include "tof_vl53l0x.h"

typedef struct {
    uint16_t left;
    uint16_t front_left;
    uint16_t front_right;
    uint16_t right;
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

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
    bool initialized;
} ToFSensors;

bool tof_sensors_init(void);
void tof_sensors_update(void);
const ToFSensors *tof_sensors_get(void);
#endif

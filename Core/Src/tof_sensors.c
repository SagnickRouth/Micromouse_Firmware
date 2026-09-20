#include "tof_sensors.h"
#include "config.h"
#include "main.h"
#include "motion.h"

extern I2C_HandleTypeDef hi2c1;

static VL53L0X_Device s[4];
static ToFSensors data = {0};

/*
 * Per-sensor linear calibration derived from the user's measured data:
 *
 *   true_mm = (raw_mm - offset_mm) / gain
 *
 * Reference points used:
 *   true: 50, 75, 100, 150, 200, 300 mm
 *
 * This compensates both the sensor-specific offset and the small scale error.
 * It is intentionally applied to the beam distance only. The diagonal
 * LD/RD readings still need geometric projection using their actual mounting
 * angle before they represent perpendicular distance to a side wall.
 */
#define TOF_LF_GAIN      1.033898305f
#define TOF_LF_OFFSET_MM 6.389830508f

#define TOF_LD_GAIN      1.029636804f
#define TOF_LD_OFFSET_MM 13.677966102f

#define TOF_RD_GAIN      1.066537530f
#define TOF_RD_OFFSET_MM 7.796610169f

#define TOF_RF_GAIN      1.030702179f
#define TOF_RF_OFFSET_MM (-4.644067797f)

static uint16_t calibrate_distance(uint16_t raw, float gain, float offset)
{
    if (raw == 0xFFFFU)
        return raw;

    float corrected = ((float)raw - offset) / gain;

    if (corrected < 0.0f)
        corrected = 0.0f;
    if (corrected > 8191.0f)
        corrected = 8191.0f;

    return (uint16_t)(corrected + 0.5f);
}

static GPIO_TypeDef *ports[4] = {
    VL53_LEFT_XSHUT_PORT, VL53_FL_XSHUT_PORT,
    VL53_FR_XSHUT_PORT, VL53_RIGHT_XSHUT_PORT
};
static uint16_t pins[4] = {
    VL53_LEFT_XSHUT_PIN, VL53_FL_XSHUT_PIN,
    VL53_FR_XSHUT_PIN, VL53_RIGHT_XSHUT_PIN
};
static const uint8_t addresses[4] = {
    VL53_ADDR_LEFT, VL53_ADDR_FL, VL53_ADDR_FR, VL53_ADDR_RIGHT
};

bool tof_sensors_init(void)
{
    data.initialized=false;
    for (int i=0;i<4;i++) HAL_GPIO_WritePin(ports[i],pins[i],GPIO_PIN_RESET);
    HAL_Delay(10);

    for (int i=0;i<4;i++) {
        HAL_GPIO_WritePin(ports[i],pins[i],GPIO_PIN_SET);
        HAL_Delay(3);

        if (!vl53l0x_init(&s[i],&hi2c1,VL53_DEFAULT_ADDR))
            return false;
        if (!vl53l0x_set_address(&s[i],addresses[i]))
            return false;

        /* Confirm the address change really took effect before enabling
           the next sensor at the default 0x29 address. */
        uint8_t model = 0U;
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(s[i].address << 1),
                             0xC0, I2C_MEMADD_SIZE_8BIT, &model, 1, 100) != HAL_OK ||
            model != 0xEEU)
            return false;
        s[i].model_id=model;
    }
    data.left_address=s[0].address;
    data.front_left_address=s[1].address;
    data.front_right_address=s[2].address;
    data.right_address=s[3].address;
    data.left_model=s[0].model_id;
    data.front_left_model=s[1].model_id;
    data.front_right_model=s[2].model_id;
    data.right_model=s[3].model_id;
    data.left_status=255U;
    data.front_left_status=255U;
    data.front_right_status=255U;
    data.right_status=255U;
    data.initialized=true;
    return true;
}

void tof_sensors_update(void)
{
    static uint32_t last_ms = 0U;
    if (!data.initialized || !motion_is_complete()) return;
    uint32_t now = HAL_GetTick();
    if ((now - last_ms) < 300U) return;
    last_ms = now;

    data.left_raw=vl53l0x_read_range_mm(&s[0]);
    data.front_left_raw=vl53l0x_read_range_mm(&s[1]);
    data.front_right_raw=vl53l0x_read_range_mm(&s[2]);
    data.right_raw=vl53l0x_read_range_mm(&s[3]);

    data.left_status=s[0].last_status;
    data.front_left_status=s[1].last_status;
    data.front_right_status=s[2].last_status;
    data.right_status=s[3].last_status;

    /*
     * Only calibrated valid measurements are exposed to wall detection.
     * Invalid values remain invalid and are never turned into a plausible
     * distance by the calibration formula.
     */
    if (data.left_status == 0U)
        data.left = calibrate_distance(data.left_raw, TOF_LF_GAIN, TOF_LF_OFFSET_MM);
    else
        data.left = 0xFFFFU;

    if (data.front_left_status == 0U)
        data.front_left = calibrate_distance(data.front_left_raw, TOF_LD_GAIN, TOF_LD_OFFSET_MM);
    else
        data.front_left = 0xFFFFU;

    if (data.front_right_status == 0U)
        data.front_right = calibrate_distance(data.front_right_raw, TOF_RD_GAIN, TOF_RD_OFFSET_MM);
    else
        data.front_right = 0xFFFFU;

    if (data.right_status == 0U)
        data.right = calibrate_distance(data.right_raw, TOF_RF_GAIN, TOF_RF_OFFSET_MM);
    else
        data.right = 0xFFFFU;

    /*
     * Project the diagonal LD/RD beam onto the side-wall normal.
     *
     * The sensors are approximately 45 degrees from the forward axis, so
     * only cos(45 deg) of the calibrated beam is perpendicular to a side
     * wall. The sensor origins are approximately 5 mm outboard of the robot
     * centerline, so that lateral offset is added to obtain the distance
     * from the robot centerline to the wall.
     *
     * This is valid for a straight wall parallel to the robot's travel axis.
     * The longitudinal position of the diagonal sensor does not affect the
     * perpendicular distance to such a wall.
     */
    if (data.front_left_status == 0U) {
        float distance = VL53_SIDE_SENSOR_LATERAL_OFFSET_MM +
                         ((float)data.front_left * VL53_SIDE_SENSOR_COS_ANGLE);
        if (distance > 8191.0f) distance = 8191.0f;
        data.left_wall_distance = (uint16_t)(distance + 0.5f);
    } else {
        data.left_wall_distance = 0xFFFFU;
    }

    if (data.front_right_status == 0U) {
        float distance = VL53_SIDE_SENSOR_LATERAL_OFFSET_MM +
                         ((float)data.front_right * VL53_SIDE_SENSOR_COS_ANGLE);
        if (distance > 8191.0f) distance = 8191.0f;
        data.right_wall_distance = (uint16_t)(distance + 0.5f);
    } else {
        data.right_wall_distance = 0xFFFFU;
    }
}

const ToFSensors *tof_sensors_get(void) { return &data; }

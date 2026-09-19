#include "tof_sensors.h"
#include "config.h"
#include "main.h"
#include "motion.h"

extern I2C_HandleTypeDef hi2c1;

static VL53L0X_Device s[4];
static ToFSensors data = {0};

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

    data.left=vl53l0x_read_range_mm(&s[0]);
    data.front_left=vl53l0x_read_range_mm(&s[1]);
    data.front_right=vl53l0x_read_range_mm(&s[2]);
    data.right=vl53l0x_read_range_mm(&s[3]);

    data.left_status=s[0].last_status;
    data.front_left_status=s[1].last_status;
    data.front_right_status=s[2].last_status;
    data.right_status=s[3].last_status;
}

const ToFSensors *tof_sensors_get(void) { return &data; }

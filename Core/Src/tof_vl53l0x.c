#include "tof_vl53l0x.h"
#include <string.h>

#define REG_SYSRANGE_START                 0x00
#define REG_SYSTEM_SEQUENCE_CONFIG         0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO   0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR        0x0B
#define REG_RESULT_INTERRUPT_STATUS       0x13
#define REG_RESULT_RANGE_STATUS            0x14
#define REG_I2C_SLAVE_DEVICE_ADDRESS       0x8A
#define REG_MSRC_CONFIG_CONTROL             0x60
#define REG_MSRC_CONFIG_TIMEOUT_MACROP      0x46
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW  0x56
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH 0x57
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW 0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define REG_GLOBAL_CONFIG_VCSEL_WIDTH       0x32
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define REG_GPIO_HV_MUX_ACTIVE_HIGH         0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define REG_IDENTIFICATION_MODEL_ID         0xC0
#define REG_OSC_CALIBRATE_VAL               0xF8
#define REG_ALGO_PHASECAL_LIM               0x30
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT    0x30

static HAL_StatusTypeDef wr(VL53L0X_Device *d, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    return HAL_I2C_Master_Transmit(d->i2c, (uint16_t)(d->address << 1), b, 2, 100);
}

static HAL_StatusTypeDef rd(VL53L0X_Device *d, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(d->i2c, (uint16_t)(d->address << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}

static HAL_StatusTypeDef wr16(VL53L0X_Device *d, uint8_t reg, uint16_t val)
{
    uint8_t b[2] = {(uint8_t)(val >> 8), (uint8_t)val};
    return HAL_I2C_Mem_Write(d->i2c, (uint16_t)(d->address << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, b, 2, 100);
}

static HAL_StatusTypeDef rd16(VL53L0X_Device *d, uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(d->i2c, (uint16_t)(d->address << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, b, 2, 100);
    if (s == HAL_OK) *val = ((uint16_t)b[0] << 8) | b[1];
    return s;
}

static HAL_StatusTypeDef wr_multi(VL53L0X_Device *d, uint8_t reg,
                                  const uint8_t *p, uint16_t n)
{
    return HAL_I2C_Mem_Write(d->i2c, (uint16_t)(d->address << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)p, n, 100);
}

static HAL_StatusTypeDef rd_multi(VL53L0X_Device *d, uint8_t reg,
                                  uint8_t *p, uint16_t n)
{
    return HAL_I2C_Mem_Read(d->i2c, (uint16_t)(d->address << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, p, n, 100);
}

static bool wait_for_interrupt(VL53L0X_Device *d, uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    uint8_t v = 0;
    while ((HAL_GetTick() - start) < timeout) {
        if (rd(d, REG_RESULT_INTERRUPT_STATUS, &v) != HAL_OK) return false;
        if (v & 0x07U) return true;
    }
    return false;
}

static bool ref_calibration(VL53L0X_Device *d, uint8_t sequence_config,
                                  uint8_t init_byte)
{
    /*
     * VL53L0X reference calibration is two distinct single-reference
     * calibrations:
     *   sequence 0x01 + 0x40 = VHV calibration
     *   sequence 0x02 + 0x00 = phase calibration
     *
     * The previous implementation ran both calibrations while the normal
     * 0xE8 sequence was selected. That is not the ST/Pololu reference
     * sequence and can leave every sensor reporting RangeStatus 4
     * (phase fail) even though I2C addressing and model ID are correct.
     */
    if (wr(d, REG_SYSTEM_SEQUENCE_CONFIG, sequence_config) != HAL_OK)
        return false;

    if (wr(d, REG_SYSRANGE_START, (uint8_t)(0x01U | init_byte)) != HAL_OK)
        return false;

    if (!wait_for_interrupt(d, d->timeout_ms)) {
        (void)wr(d, REG_SYSRANGE_START, 0x00);
        return false;
    }

    if (wr(d, REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != HAL_OK) {
        (void)wr(d, REG_SYSRANGE_START, 0x00);
        return false;
    }

    if (wr(d, REG_SYSRANGE_START, 0x00) != HAL_OK)
        return false;

    return true;
}

static bool get_spad_info(VL53L0X_Device *d, uint8_t *count, bool *aperture)
{
    uint8_t v, tmp;
    if (wr(d,0x80,0x01)!=HAL_OK) return false;
    if (wr(d,0xFF,0x01)!=HAL_OK) return false;
    if (wr(d,0x00,0x00)!=HAL_OK) return false;
    if (wr(d,0xFF,0x06)!=HAL_OK) return false;
    if (rd(d,0x83,&v)!=HAL_OK) return false;
    if (wr(d,0x83,(uint8_t)(v|0x04))!=HAL_OK) return false;
    if (wr(d,0xFF,0x07)!=HAL_OK || wr(d,0x81,0x01)!=HAL_OK ||
        wr(d,0x80,0x01)!=HAL_OK || wr(d,0x94,0x6B)!=HAL_OK ||
        wr(d,0x83,0x00)!=HAL_OK) return false;

    uint32_t start=HAL_GetTick();
    do {
        if (rd(d,0x83,&v)!=HAL_OK) return false;
        if ((HAL_GetTick()-start)>d->timeout_ms) return false;
    } while (v==0);

    if (wr(d,0x83,0x01)!=HAL_OK || rd(d,0x92,&tmp)!=HAL_OK) return false;
    *count = tmp & 0x7F;
    *aperture = ((tmp >> 7) & 1U) != 0;

    (void)wr(d,0x81,0x00);
    (void)wr(d,0xFF,0x06);
    if (rd(d,0x83,&v)==HAL_OK) (void)wr(d,0x83,(uint8_t)(v & ~0x04U));
    (void)wr(d,0xFF,0x01);
    (void)wr(d,0x00,0x01);
    (void)wr(d,0xFF,0x00);
    (void)wr(d,0x80,0x00);
    return true;
}

static bool set_signal_rate_limit(VL53L0X_Device *d, float mcps)
{
    if (mcps < 0.0f || mcps > 511.99f) return false;
    return wr16(d, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (uint16_t)(mcps * 128.0f)) == HAL_OK;
}

bool vl53l0x_init(VL53L0X_Device *d, I2C_HandleTypeDef *i2c, uint8_t address)
{
    uint8_t model, v, map[6];
    uint8_t spad_count;
    bool aperture;

    memset(d,0,sizeof(*d));
    d->i2c=i2c;
    d->address=address;
    d->timeout_ms=100;
    d->last_status=255U;
    d->model_id=0U;

    if (rd(d, REG_IDENTIFICATION_MODEL_ID, &model) != HAL_OK || model != 0xEE)
        return false;
    d->model_id=model;

    if (rd(d,REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,&v)!=HAL_OK ||
        wr(d,REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,(uint8_t)(v|1))!=HAL_OK) return false;

    if (wr(d,0x88,0x00)!=HAL_OK || wr(d,0x80,0x01)!=HAL_OK ||
        wr(d,0xFF,0x01)!=HAL_OK || wr(d,0x00,0x00)!=HAL_OK ||
        rd(d,0x91,&d->stop_variable)!=HAL_OK ||
        wr(d,0x00,0x01)!=HAL_OK || wr(d,0xFF,0x00)!=HAL_OK ||
        wr(d,0x80,0x00)!=HAL_OK) return false;

    if (rd(d,REG_MSRC_CONFIG_CONTROL,&v)!=HAL_OK ||
        wr(d,REG_MSRC_CONFIG_CONTROL,(uint8_t)(v|0x12))!=HAL_OK ||
        !set_signal_rate_limit(d,0.25f) ||
        wr(d,REG_SYSTEM_SEQUENCE_CONFIG,0xFF)!=HAL_OK) return false;

    if (!get_spad_info(d,&spad_count,&aperture)) return false;
    if (rd_multi(d,REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,map,6)!=HAL_OK) return false;

    if (wr(d,0xFF,0x01)!=HAL_OK ||
        wr(d,REG_DYNAMIC_SPAD_REF_EN_START_OFFSET,0x00)!=HAL_OK ||
        wr(d,REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,0x2C)!=HAL_OK ||
        wr(d,0xFF,0x00)!=HAL_OK ||
        wr(d,REG_GLOBAL_CONFIG_REF_EN_START_SELECT,0xB4)!=HAL_OK) return false;

    uint8_t first = aperture ? 12 : 0, enabled=0;
    for (uint8_t i=0;i<48;i++) {
        if (i<first || enabled==spad_count) map[i/8] &= (uint8_t)~(1U<<(i%8));
        else if ((map[i/8]>>(i%8))&1U) enabled++;
    }
    if (wr_multi(d,REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,map,6)!=HAL_OK) return false;

    /* ST default tuning settings. */
    const uint8_t tune[][2]={
      {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
      {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
      {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
      {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
      {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
      {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
      {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
      {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
      {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
      {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
      {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
      {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
      {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
      {0xFF,0x00},{0x80,0x00}
    };
    for (uint32_t i=0;i<sizeof(tune)/sizeof(tune[0]);i++)
        if (wr(d,tune[i][0],tune[i][1])!=HAL_OK) return false;

    if (wr(d,REG_SYSTEM_INTERRUPT_CONFIG_GPIO,0x04)!=HAL_OK ||
        rd(d,REG_GPIO_HV_MUX_ACTIVE_HIGH,&v)!=HAL_OK ||
        wr(d,REG_GPIO_HV_MUX_ACTIVE_HIGH,(uint8_t)(v & ~0x10U))!=HAL_OK ||
        wr(d,REG_SYSTEM_INTERRUPT_CLEAR,0x01)!=HAL_OK ||
        wr(d,REG_SYSTEM_SEQUENCE_CONFIG,0xE8)!=HAL_OK) return false;

    /* Perform VHV calibration, then phase calibration, using the
       dedicated ST sequence-step configurations. */
    if (!ref_calibration(d,0x01,0x40)) return false;
    if (!ref_calibration(d,0x02,0x00)) return false;
    if (wr(d,REG_SYSTEM_SEQUENCE_CONFIG,0xE8)!=HAL_OK) return false;

    d->initialized=true;
    return true;
}

bool vl53l0x_set_address(VL53L0X_Device *d, uint8_t new_address)
{
    if (!d || new_address < 0x08 || new_address > 0x77) return false;
    if (wr(d,REG_I2C_SLAVE_DEVICE_ADDRESS,(uint8_t)(new_address & 0x7F))!=HAL_OK) return false;
    HAL_Delay(2);
    d->address=(uint8_t)(new_address & 0x7F);
    return true;
}

uint16_t vl53l0x_read_range_mm(VL53L0X_Device *d)
{
    uint8_t v;
    uint16_t range;

    /*
     * Any transport/timeout failure is an invalid measurement. Clear the
     * status here so wall detection cannot reuse the previous valid status
     * with a new but unusable distance value.
     */
    if (!d || !d->initialized) return 0xFFFF;
    d->last_status = 255U;

    if (wr(d,0x80,0x01)!=HAL_OK || wr(d,0xFF,0x01)!=HAL_OK ||
        wr(d,0x00,0x00)!=HAL_OK || wr(d,0x91,d->stop_variable)!=HAL_OK ||
        wr(d,0x00,0x01)!=HAL_OK || wr(d,0xFF,0x00)!=HAL_OK ||
        wr(d,0x80,0x00)!=HAL_OK || wr(d,REG_SYSRANGE_START,0x01)!=HAL_OK)
        return 0xFFFF;

    uint32_t start=HAL_GetTick();
    do {
        if (rd(d,REG_SYSRANGE_START,&v)!=HAL_OK) return 0xFFFF;
        if (HAL_GetTick()-start>d->timeout_ms) return 0xFFFF;
    } while (v&1U);

    if (!wait_for_interrupt(d,d->timeout_ms)) return 0xFFFF;

    /* RESULT_RANGE_STATUS (0x14), bits 6:3, contains the ST range status.
       0 = valid, 2 = signal fail. A reported 8191 mm with status 2 is
       an invalid/no-target result, not a real 8.191 m measurement. */
    if (rd(d,REG_RESULT_RANGE_STATUS,&v)!=HAL_OK) return 0xFFFF;

    /*
     * The bits in RESULT_RANGE_STATUS contain the device-internal status,
     * not the PAL/API RangeStatus shown by the ST VL53L0X API.
     *
     * In particular, internal status 11 means GOOD RANGING. The ST PAL
     * layer maps that condition to RangeStatus 0 (Range Valid). Reporting
     * the raw value as the public status made valid measurements appear
     * as "S11" on the OLED.
     */
    uint8_t device_status = (uint8_t)((v & 0x78U) >> 3);
    if (device_status == 0U || device_status == 5U ||
        device_status == 7U || device_status >= 12U) {
        d->last_status = 255U;              /* NONE / no valid update */
    } else if (device_status == 1U || device_status == 2U ||
               device_status == 3U) {
        d->last_status = 5U;                /* Hardware fail */
    } else if (device_status == 6U || device_status == 9U) {
        d->last_status = 4U;                /* Phase fail */
    } else if (device_status == 8U || device_status == 10U) {
        d->last_status = 3U;                /* Min range fail */
    } else if (device_status == 4U) {
        d->last_status = 2U;                /* Signal fail */
    } else {
        /* Includes device status 11: valid ranging. */
        d->last_status = 0U;
    }

    if (rd16(d,(uint8_t)(REG_RESULT_RANGE_STATUS+10),&range)!=HAL_OK) {
        d->last_status = 255U;
        return 0xFFFF;
    }
    (void)wr(d,REG_SYSTEM_INTERRUPT_CLEAR,0x01);
    return range;
}

bool vl53l0x_timeout_occurred(VL53L0X_Device *d)
{
    return d && d->initialized && d->last_status == 255U;
}

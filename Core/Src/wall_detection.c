#include "wall_detection.h"
#include "tof_sensors.h"

static WallState state = {0};

/*
 * Thresholds are based on the user's measured VL53L0X response.
 *
 * At a true 100 mm wall distance:
 *   LF = 111 mm, LD = 117 mm, RD = 117 mm, RF = 102 mm.
 *
 * LD/RD are diagonal, so their line-of-sight distance to a side wall is
 * greater than the perpendicular distance. The side threshold is therefore
 * deliberately higher than the front threshold.
 *
 * Hysteresis prevents rapid WALL/OPEN toggling near the threshold.
 */
#define FRONT_WALL_DETECT_MM 115U
#define FRONT_WALL_CLEAR_MM  135U
/* These thresholds are centerline-to-wall distances after diagonal projection.
 * They preserve approximately the previous 145/165 mm diagonal-beam thresholds:
 * 5 + 145*cos(45) ~= 108 mm and 5 + 165*cos(45) ~= 122 mm.
 */
#define SIDE_WALL_DETECT_MM  108U
#define SIDE_WALL_CLEAR_MM   122U

/*
 * An invalid/no-target result means we do not have a usable wall distance.
 * Do not keep an old WALL state forever just because the sensor temporarily
 * reports a non-valid range. Require two consecutive invalid samples before
 * declaring that wall OPEN, which avoids a single bad sample causing a
 * false opening.
 */
#define INVALID_SAMPLES_TO_CLEAR 2U

static bool update_hysteresis(bool previous, bool valid, uint16_t distance,
                              uint16_t detect_mm, uint16_t clear_mm,
                              uint8_t *invalid_count)
{
    if (!valid) {
        if (*invalid_count < INVALID_SAMPLES_TO_CLEAR)
            (*invalid_count)++;

        if (*invalid_count >= INVALID_SAMPLES_TO_CLEAR)
            return false;

        return previous;
    }

    *invalid_count = 0U;

    if (previous)
        return distance < clear_mm;

    return distance <= detect_mm;
}

void wall_detection_init(void)
{
    state.left = false;
    state.front = false;
    state.right = false;
    state.left_valid = false;
    state.front_valid = false;
    state.right_valid = false;
    state.front_error_mm = 0;
}

void wall_detection_update(void)
{
    const ToFSensors *tof = tof_sensors_get();

    static uint8_t left_invalid_count = 0U;
    static uint8_t right_invalid_count = 0U;
    static uint8_t front_invalid_count = 0U;

    if (tof == NULL || !tof->initialized)
        return;

    const bool lf_valid = (tof->left_status == 0U);
    const bool ld_valid = (tof->front_left_status == 0U);
    const bool rd_valid = (tof->front_right_status == 0U);
    const bool rf_valid = (tof->right_status == 0U);

    state.left_valid = ld_valid;
    state.right_valid = rd_valid;

    /* LD is diagonal; use the projected centerline-to-wall distance. */
    state.left = update_hysteresis(state.left, ld_valid, tof->left_wall_distance,
                                   SIDE_WALL_DETECT_MM, SIDE_WALL_CLEAR_MM,
                                   &left_invalid_count);

    /* RD is diagonal; use the projected centerline-to-wall distance. */
    state.right = update_hysteresis(state.right, rd_valid, tof->right_wall_distance,
                                    SIDE_WALL_DETECT_MM, SIDE_WALL_CLEAR_MM,
                                    &right_invalid_count);

    /*
     * LF and RF both point straight forward.
     *
     * With both readings valid, require both sensors to see the close wall.
     * If one temporarily becomes invalid, use the remaining valid forward
     * sensor so a single bad sample does not hide a real wall.
     *
     * If neither forward sensor is valid for two consecutive updates,
     * explicitly clear the old FRONT=WALL state.
     */
    const bool front_any_valid = lf_valid || rf_valid;
    state.front_valid = front_any_valid;

    if (front_any_valid) {
        front_invalid_count = 0U;

        bool front_detect;
        bool front_clear;

        if (lf_valid && rf_valid) {
            front_detect = (tof->left <= FRONT_WALL_DETECT_MM) &&
                           (tof->right <= FRONT_WALL_DETECT_MM);
            front_clear = (tof->left >= FRONT_WALL_CLEAR_MM) &&
                          (tof->right >= FRONT_WALL_CLEAR_MM);
        } else if (lf_valid) {
            front_detect = (tof->left <= FRONT_WALL_DETECT_MM);
            front_clear = (tof->left >= FRONT_WALL_CLEAR_MM);
        } else {
            front_detect = (tof->right <= FRONT_WALL_DETECT_MM);
            front_clear = (tof->right >= FRONT_WALL_CLEAR_MM);
        }

        if (!state.front && front_detect)
            state.front = true;
        else if (state.front && front_clear)
            state.front = false;
    } else {
        if (front_invalid_count < INVALID_SAMPLES_TO_CLEAR)
            front_invalid_count++;

        if (front_invalid_count >= INVALID_SAMPLES_TO_CLEAR)
            state.front = false;
    }

    /*
     * Front alignment error:
     *   > 0 : LF sees farther than RF
     *   < 0 : LF sees closer than RF
     */
    if (lf_valid && rf_valid) {
        int32_t error = (int32_t)tof->left - (int32_t)tof->right;
        if (error > 32767) error = 32767;
        if (error < -32768) error = -32768;
        state.front_error_mm = (int16_t)error;
    }
}

const WallState *wall_detection_get(void)
{
    return &state;
}

bool wall_detection_left(void)
{
    return state.left;
}

bool wall_detection_front(void)
{
    return state.front;
}

bool wall_detection_right(void)
{
    return state.right;
}

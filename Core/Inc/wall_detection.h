#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Physical ToF layout:
 *   LF = left-most, straight forward
 *   LD = left-inner, diagonal forward-left
 *   RD = right-inner, diagonal forward-right
 *   RF = right-most, straight forward
 *
 * Therefore:
 *   LF + RF -> front-wall detection
 *   LD      -> left-wall detection
 *   RD      -> right-wall detection
 */

typedef struct {
    bool left;
    bool front;
    bool right;

    bool left_valid;
    bool front_valid;
    bool right_valid;

    /* LF - RF, in mm. Positive means LF sees farther than RF. */
    int16_t front_error_mm;
} WallState;

void wall_detection_init(void);
void wall_detection_update(void);
const WallState *wall_detection_get(void);

bool wall_detection_left(void);
bool wall_detection_front(void);
bool wall_detection_right(void);

#endif

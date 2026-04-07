#ifndef __DRIVE_FEEDBACK_SERVICE_H__
#define __DRIVE_FEEDBACK_SERVICE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t left_wheel_angular_mrad_s;
    int32_t right_wheel_angular_mrad_s;
} drive_feedback_t;

void drive_feedback_snapshot(drive_feedback_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVE_FEEDBACK_SERVICE_H__ */

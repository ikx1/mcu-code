#ifndef __ROBOT_MOTION_CMD_H__
#define __ROBOT_MOTION_CMD_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROBOT_JOINT_CMD_NONE = 0,
    ROBOT_JOINT_CMD_ABSOLUTE_POS,
    ROBOT_JOINT_CMD_RELATIVE_POS,
} robot_joint_cmd_type_t;

typedef struct
{
    int32_t right_rpm;
    int32_t left_rpm;
    robot_joint_cmd_type_t joint_cmd_type;
    int32_t joint_cmd_value;
} robot_motion_cmd_t;

void robot_motion_cmd_snapshot(robot_motion_cmd_t *out);
void robot_motion_cmd_clear_motion(void);
void robot_motion_cmd_set_drive_rpm(int32_t right_rpm, int32_t left_rpm);
void robot_motion_cmd_set_joint_absolute(int32_t joint_abs_pos);
void robot_motion_cmd_set_joint_relative(int32_t joint_rel_pos);
void robot_motion_cmd_clear_joint(void);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_MOTION_CMD_H__ */

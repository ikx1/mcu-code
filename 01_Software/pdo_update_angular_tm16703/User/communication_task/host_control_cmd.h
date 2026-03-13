#ifndef __HOST_CONTROL_CMD_H__
#define __HOST_CONTROL_CMD_H__

#include <stdint.h>
#include "host_status_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HOST_MOTOR_POWER_CMD_NONE = 0,
    HOST_MOTOR_POWER_CMD_ENABLE = 1,
    HOST_MOTOR_POWER_CMD_DISABLE = 2,
} host_motor_power_cmd_t;

/* Communication layer owns this mailbox.
 * Other layers may snapshot it, but must not own its storage.
 */
typedef struct
{
    /* Protocol command semantics:
     * tag 0x01 + 0x02 -> one complete drive command frame (mrad/s)
     * tag 0x03 -> z_lift (mm)
     */
    int32_t left_wheel_angular_mrad_s;
    int32_t right_wheel_angular_mrad_s;
    /* Incremented once per accepted complete drive command frame and preserved across session reset. */
    uint32_t drive_cmd_seq;
    int32_t z_lift_mm;
    /* Incremented once per accepted lift command frame and preserved across session reset. */
    uint32_t lift_cmd_seq;
    uint8_t motor_reset_mask;
    host_motor_power_cmd_t right_motor_power_cmd;
    host_motor_power_cmd_t left_motor_power_cmd;
    host_motor_power_cmd_t joint_motor_power_cmd;
} host_control_cmd_t;

void host_control_cmd_snapshot(host_control_cmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __HOST_CONTROL_CMD_H__ */

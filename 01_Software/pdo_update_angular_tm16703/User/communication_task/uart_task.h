#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool uart_task_start(void);
/* Any valid UART protocol frame received within timeout. */
bool uart_session_is_alive(void);
/* Any host drive command received within timeout. */
bool uart_drive_cmd_is_fresh(void);
/* Any host lift command received within timeout. */
bool uart_lift_cmd_is_fresh(void);
void uart_task_diag_maintenance(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_TASK_H__ */

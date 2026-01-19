/********************************** Includes *********************************/
#include "control_task.h"
#include "bsp_motor_handler.h"
#include "bsp_ibus_handler.h"
#include "bsp_gpio_driver.h"
#include "uart_task.h"
#include "wdt_task.h"

#include "system_cfg.h"

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/********************************** Defines **********************************/
#define MOTOR_ENABLE    0x01
#define MOTOR_DISABLE   0x02

/********************************** Variables ********************************/
static const robot_speed_par_t robot_speed_par = {
	.high_speed = 0.8f,     
    .high_roat  = 2.0f,     
    .low_speed  = 0.4f,     
    .low_roat   = 0.8f      
};

#ifndef DEBUG
	static ROBOT_MODE_T robot_mode_current = {
#else
	ROBOT_MODE_T robot_mode_current = {
#endif
    .robot_mode_main = MODE_READY,
    .robot_mode_sub = SUB_STOP,
    .robot_speed_level = SPEED_LOW
};

#ifndef DEBUG
	static ROBOT_MODE_T robot_mode_last = {
#else
	ROBOT_MODE_T robot_mode_last = {
#endif
    .robot_mode_main = MODE_UNKNOWN,
    .robot_mode_sub = SUB_UNKNOWN,
};

#ifndef DEBUG
    static ROBOT_INFO robot_info = {0};
#else
    ROBOT_INFO robot_info = {0};
#endif /* DEBUG */

#ifndef DEBUG
	static uint32_t contronl_task_id;
#endif
	
/********************************** Functions ********************************/
void robot_mode_update_50ms(void);
uint8_t manu_set_lr_spd_low(void);
uint8_t manu_set_lr_spd_high(void);
uint8_t motor_speed(void);

void ContronlTask(void *pvParameters)
{ 
	uint32_t lastWakeTime = xTaskGetTickCount();
#ifndef DEBUG
   wdt_monitor_task_register("control", 300, &contronl_task_id);  // 400ms 超时时间
   wdt_monitor_task_enable(contronl_task_id);
#endif /* DEBUG */	

    while(1)
    {	
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50)); 

#ifndef DEBUG
		wdt_monitor_task_feed(contronl_task_id);
#endif /* DEBUG */	

		robot_mode_update_50ms();
	}
}

const ROBOT_MODE_T* get_robot_mode_p(void)
{
    return &robot_mode_current;
}


ROBOT_INFO* get_robot_p(void)
{
    return &robot_info;
}

void robot_info_snapshot(ROBOT_INFO *out)
{
	if (out == NULL)
	{
		return;
	}

	taskENTER_CRITICAL();
	*out = robot_info;
	taskEXIT_CRITICAL();
}

static inline void set_motor_speed(motor_driver_t* m, int32_t rs, int32_t ls)
{
	if (m[0].set_speed)
	{
		m[0].set_speed(&m[0], rs);
	}
	if (m[1].set_speed)
	{
		m[1].set_speed(&m[1], ls);
	}
}

static control_err_t motor_stop_all(void)
{
	motor_driver_t *motor_ptr = get_motor_drivers_p();
	if(NULL == motor_ptr)
    {
        return CONTROL_NULL_POINTER;
    }
    set_motor_speed(motor_ptr, 0, 0);

    return CONTROL_OK;
}

static control_err_t motorinfo_clear(void)
{
	motor_driver_t *motor_ptr = get_motor_drivers_p();
    ROBOT_INFO* p = get_robot_p();
	if(NULL == motor_ptr || NULL == p)
    {
        return CONTROL_NULL_POINTER;
    }

#ifdef DEBUG	
	if(MODE_READY == robot_mode_current.robot_mode_main)
	{
		taskENTER_CRITICAL();
		p->glob_line_speed = 0;
		p->glob_rota_speed = 0;
		p->glob_joint_pos = 0;
		p->status_word = 0x00;
		p->time_stamp = 0;
		taskEXIT_CRITICAL();
		
		set_motor_speed(motor_ptr, 0, 0);

		return CONTROL_OK;
	}
#endif
	
    if(SUB_SLAVE != robot_mode_last.robot_mode_sub || 
      SUB_SLAVE != robot_mode_current.robot_mode_sub)
    {
        return CONTROL_OK;
    }


	taskENTER_CRITICAL();
    p->glob_line_speed = 0;
    p->glob_rota_speed = 0;
    p->glob_joint_pos = 0;
    p->status_word = 0x00;
    p->time_stamp = 0;
	taskEXIT_CRITICAL();
	
	set_motor_speed(motor_ptr, 0, 0);

    return CONTROL_OK;
}

control_err_t robot_set_mode(ROBOT_MODE_MAIN main, ROBOT_MODE_SUB sub)
{
    if (main >= MODE_UNKNOWN || sub >= SUB_UNKNOWN)
        return CONTROL_ERR;

    robot_mode_current.robot_mode_main = main;
    robot_mode_current.robot_mode_sub  = sub;
    return CONTROL_OK;
}

/* 状态切换 */
static control_err_t enter_emergency(void) 
{
    robot_mode_current.robot_mode_main = MODE_EMERGENCY;

    /* enable/disable is arbitrated in Motor_Task to avoid conflicts */
    return motor_stop_all();
}

static control_err_t enter_ready(void) 
{
    robot_mode_current.robot_mode_main = MODE_READY;
    robot_mode_current.robot_mode_sub = SUB_STOP;
    return motor_stop_all();
}

static control_err_t enter_remote(void) 
{
    robot_mode_current.robot_mode_main = MODE_REMOTE;
    robot_mode_current.robot_mode_sub = SUB_STOP;

    if(CONTROL_OK != motorinfo_clear())
    {
        return CONTROL_ERR;
    }
    return motor_stop_all();
}

static void update_remote_sub_mode(int16_t ltr5, int16_t ltr6, int16_t ltr7, int16_t ltr8) 
{
    robot_mode_current.robot_speed_level = (ltr5 == 1) ? SPEED_HIGH : SPEED_LOW;

    if (ltr8 == 1 && ltr6 == 0 && ltr7 == 0) 
    {
        robot_mode_current.robot_mode_sub = SUB_CHARGE;
    } 
    else if (ltr6 == 0 && ltr7 == 0) 
    {
        robot_mode_current.robot_mode_sub = SUB_STOP;
    } 
    else if (ltr6 == 1 && ltr7 == 0) 
    {
        robot_mode_current.robot_mode_sub = SUB_MANUAL;
    }
    else if (ltr6 == 2 && ltr7 == 0) 
    {
        robot_mode_current.robot_mode_sub = SUB_SLAVE;
    } 
    else if (ltr6 == 0 && ltr7 == 1) 
    {
        robot_mode_current.robot_mode_sub = SUB_MOTOR_CTRL;
    } 
    else if (ltr6 == 0 && ltr7 == 2) 
    {
        robot_mode_current.robot_mode_sub = SUB_MOTOR_ERR_CLR;
    }
}

static control_err_t handle_remote_submode(void) 
{
	control_err_t ret = CONTROL_OK;

	switch (robot_mode_current.robot_mode_sub) 
	{
		case SUB_STOP:
			
			return CONTROL_OK;

		case SUB_MANUAL:
			ret = motorinfo_clear();
			if(CONTROL_OK != ret)
			{
				return ret;
			}	

			(robot_mode_current.robot_speed_level == SPEED_LOW) ? 
						manu_set_lr_spd_low() : manu_set_lr_spd_high();
			break;

		case SUB_SLAVE:
			motor_speed();
			break;
		
		case SUB_CHARGE:
           
			break;

		default:
			break;
	}

	return ret;
}

void robot_state_execute(void) 
{
    control_err_t err = CONTROL_OK;

    if (robot_mode_current.robot_mode_main != robot_mode_last.robot_mode_main)
    {
        switch (robot_mode_current.robot_mode_main)
        {
            case MODE_READY:     err = enter_ready(); break;
            case MODE_REMOTE:    err = enter_remote(); break;
            case MODE_EMERGENCY: err = enter_emergency(); break;
            default:             break;
        }
        if (err != CONTROL_OK)
		{
			// LOG_ERROR("Main mode switch failed: %d", err);
		}

        robot_mode_last.robot_mode_main = robot_mode_current.robot_mode_main;
        robot_mode_last.robot_mode_sub  = robot_mode_current.robot_mode_sub;
    }

    if (robot_mode_current.robot_mode_main == MODE_REMOTE)
	{
		if(robot_mode_current.robot_mode_sub != robot_mode_last.robot_mode_sub)
		{
			robot_mode_last.robot_mode_sub = robot_mode_current.robot_mode_sub;
		}

        err = handle_remote_submode();
        if (err != CONTROL_OK)
		{
            // LOG_ERROR("Sub mode handler failed: %d", err);
		}
	} 
	
#ifdef DEBUG
	if(MODE_READY == robot_mode_current.robot_mode_main)
	{
		motor_speed();		
	}
#endif
}

void robot_mode_update_50ms(void) 
{
    int16_t ltr5, ltr6, ltr7, ltr8;

	IBUS_Handler* ibus_data = get_ibus_data_p();
	const input_status_t* status = input_driver_get_status();
	if(NULL == ibus_data || NULL == status)
	{
		return;
	}

    if (status->emergency_flag) 
    {
        if(MODE_EMERGENCY != robot_mode_current.robot_mode_main)
        {
            robot_set_mode(MODE_EMERGENCY, SUB_STOP);
        }
        robot_state_execute();
        
        return;
    }

    if(ibus_handler_get_channel_value(ibus_data, IBUS_CH_CONN)) 
    {
        if (robot_mode_current.robot_mode_main != MODE_REMOTE) 
        {
            enter_remote();
        }

        ltr5 = ibus_handler_get_channel_value(ibus_data, IBUS_CH_SWD);
        ltr6 = ibus_handler_get_channel_value(ibus_data, IBUS_CH_SWC);
        ltr7 = ibus_handler_get_channel_value(ibus_data, IBUS_CH_SWB);
        ltr8 = ibus_handler_get_channel_value(ibus_data, IBUS_CH_SWA);

        update_remote_sub_mode(ltr5, ltr6, ltr7, ltr8);
    } 
    else 
    {
        if(MODE_READY != robot_mode_current.robot_mode_main)
        {
            robot_set_mode(MODE_READY, SUB_STOP);
        }
    }

    robot_state_execute();
}

uint8_t manu_set_lr_spd_high(void)
{
	uint8_t rtn = 0;
	
	IBUS_Handler* ibus_data = get_ibus_data_p();
	motor_driver_t *motor_ptr = get_motor_drivers_p();
	
	int16_t linear_input = ibus_handler_get_channel_value(ibus_data, 
                                                                   IBUS_CH_RY);
    int16_t angular_input = ibus_handler_get_channel_value(ibus_data, 
                                                                   IBUS_CH_RX);
	    // 死区处理
	if (abs(linear_input) < DEADZONE) linear_input = 0;
    if (abs(angular_input) < DEADZONE) angular_input = 0;
	
	float linear_ratio = (float)linear_input / MAX_INPUT_VALUE;
    float angular_ratio = (float)angular_input / MAX_INPUT_VALUE;	
	
	float line_speed = robot_speed_par.high_speed * linear_ratio;
	float rotation_speed = robot_speed_par.high_roat * angular_ratio;
		
	float differential = rotation_speed * WHEEL_WIDTH / 2.0f;
    float left_speed = line_speed + differential;
    float right_speed = line_speed - differential;
	
	float max_rpm = robot_speed_par.high_speed * TURN_M;
    float left_rpm = left_speed * TURN_M;
    float right_rpm = right_speed * TURN_M;
    
    left_rpm = fmaxf(fminf(left_rpm, max_rpm), -max_rpm);
    right_rpm = fmaxf(fminf(right_rpm, max_rpm), -max_rpm);
	
	set_motor_speed(motor_ptr, -(int32_t)right_rpm, (int32_t)left_rpm);

    return rtn;
}

uint8_t manu_set_lr_spd_low(void)
{
	uint8_t rtn = 0;
	
	IBUS_Handler* ibus_data = get_ibus_data_p();
	motor_driver_t *motor_ptr = get_motor_drivers_p();
	
	int16_t linear_input = ibus_handler_get_channel_value(ibus_data, 
                                                                   IBUS_CH_RY);
    int16_t angular_input = ibus_handler_get_channel_value(ibus_data, 
                                                                   IBUS_CH_RX);
	    // 死区处理
	if (abs(linear_input) < DEADZONE) linear_input = 0;
    if (abs(angular_input) < DEADZONE) angular_input = 0;
	
	float linear_ratio = (float)linear_input / MAX_INPUT_VALUE;
    float angular_ratio = (float)angular_input / MAX_INPUT_VALUE;	
	
	float line_speed = robot_speed_par.low_speed * linear_ratio;
	float rotation_speed = robot_speed_par.low_roat * angular_ratio;
		
	float differential = rotation_speed * WHEEL_WIDTH / 2.0f;
    float left_speed = line_speed + differential;
    float right_speed = line_speed - differential;
	
	float max_rpm = robot_speed_par.low_speed * TURN_M;
    float left_rpm = left_speed * TURN_M;
    float right_rpm = right_speed * TURN_M;
    
    left_rpm = fmaxf(fminf(left_rpm, max_rpm), -max_rpm);
    right_rpm = fmaxf(fminf(right_rpm, max_rpm), -max_rpm);
	
	set_motor_speed(motor_ptr, -(int32_t)right_rpm, (int32_t)left_rpm);

    return rtn;
}


static void status_word_analyze(uint16_t status_word, motor_driver_t *motor_ptr)
{
    /* status_word usage (current protocol):
     *  - Right motor cmd: bits[1:0]  (00 none, 01 enable, 10 disable)
     *  - Right reset: bit2
     *  - Left  motor cmd: bits[4:3]  (00 none, 01 enable, 10 disable)
     *  - Left  reset: bit5
     *  - Charge enable: bit14 (reserved)
     *
     * IMPORTANT: enable/disable must NOT be issued here to avoid conflicts.
     * Motor power is arbitrated and applied ONLY in Motor_Task().
     */
    (void)((status_word >> 14) & 0x01); /* charge_enable reserved */

    bool right_motor_reset = ((status_word >> 2) & 0x01) ? true : false;
    bool left_motor_reset  = ((status_word >> 5) & 0x01) ? true : false;

    if (motor_ptr == NULL) return;

    if ((MOTOR_NUM > 0) && right_motor_reset && motor_ptr[0].reset_motor) {
        motor_ptr[0].reset_motor(motor_ptr[0].info->id);
    }
    if ((MOTOR_NUM > 1) && left_motor_reset && motor_ptr[1].reset_motor) {
        motor_ptr[1].reset_motor(motor_ptr[1].info->id);
    }
}


uint8_t motor_speed(void)
{	
	ROBOT_INFO info = {0};
	motor_driver_t *motor_ptr = get_motor_drivers_p();
    if(NULL == motor_ptr)
	{
		return 1;
	}
	robot_info_snapshot(&info);
	if (!uart_link_is_alive())
	{
		set_motor_speed(motor_ptr, 0, 0);
		return 1;
	}
		
    /* parse host commands (enable/disable handled in Motor_Task) */
    status_word_analyze(info.status_word, motor_ptr);

    const input_status_t* status = input_driver_get_status();
    uint8_t right_cmd = (uint8_t)((info.status_word >> 0) & 0x03);
    uint8_t left_cmd  = (uint8_t)((info.status_word >> 3) & 0x03);
    bool any_disable = false;
    if (status && status->emergency_flag) any_disable = true;
    if (right_cmd == MOTOR_DISABLE || left_cmd == MOTOR_DISABLE) any_disable = true;
    if (any_disable) {
        set_motor_speed(motor_ptr, 0, 0);
        return 0;
    }

	uint8_t rtn = 0;
	int32_t cmdL = 0, cmdR = 0;

	float v = (float)(info.glob_line_speed / 1000.0f);
	float w = (float)(info.glob_rota_speed / 1000.0f);
	
	if(info.glob_rota_speed == 0)
	{
		cmdL = v * 60 / WHEEL_PERIMETER * 10;
		cmdR = cmdL;
	}
	else
	{
        float vL = v - w * (WHEEL_WIDTH * 0.5f);
        float vR = v + w * (WHEEL_WIDTH * 0.5f);

        float rpml = vL * 60.0f / WHEEL_PERIMETER;
        float rpmr = vR * 60.0f / WHEEL_PERIMETER;

        cmdL = (int32_t)(rpml * 10.0f);
        cmdR = (int32_t)(rpmr * 10.0f);
	}
	
	if(cmdL > TURN_M) cmdL = TURN_M;
	if(cmdR > TURN_M) cmdR  = TURN_M;
	if(cmdL < -TURN_M) cmdL = -TURN_M;
	if(cmdR < -TURN_M) cmdR  = -TURN_M;
	
	set_motor_speed(motor_ptr, -(int32_t)cmdR, (int32_t)cmdL);
	
	return rtn;
}

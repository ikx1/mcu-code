#include "bsp_handler_display.h"

#include <stddef.h>

#include "bsp_battery_handler.h"
#include "uart_legacy_bridge.h"
#include "irq_guard.h"
#include "user_function.h"

static DISPLAY_INFO display_info = {0};
static volatile uint8_t s_robot_mode_main = 0u;
static volatile uint8_t s_robot_mode_sub = 0u;
static volatile uint8_t s_read_data01_pending = 0u;

/**
 * @brief 锁定显示中断
 * 
 * @return uint32_t 中断状态
 */
static uint32_t display_irq_lock(void)
{
    return mcu_irq_guard_lock();
}

/**
 * @brief 解锁显示中断
 * 
 * @param primask 中断状态
 */
static void display_irq_unlock(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

/**
 * @brief 回答读取数据01请求
 * 
 * @param robot_mode_main 主模式
 * @param robot_mode_sub 子模式
 * @return uint8_t 0表示成功，1表示失败
 */
static uint8_t display_answer_read_data01(uint8_t robot_mode_main,
                                          uint8_t robot_mode_sub)
{
    BATTERY_INFO battery_info;
    uint8_t str[DISPLAY_FRAME] = {0};

    if (BatteryHandler_Snapshot(&battery_info) != 0u) {
        return 1u;
    }

    str[0] = DISPLAY_HEADER_1;
    str[1] = DISPLAY_HEADER_2;
    str[2] = 0x55u;
    str[3] = 0x01u;
    str[4] = robot_mode_main;
    str[5] = robot_mode_sub;
    str[6] = battery_info.voltage[0];
    str[7] = battery_info.voltage[1];
    str[8] = battery_info.soc[0];
    str[9] = battery_info.soc[1];
    str[10] = battery_info.charge_status;
    str[11] = battery_info.electricity[0];
    str[12] = battery_info.electricity[1];
    str[13] = Serial_checksum(str, DISPLAY_FRAME - 1u);

    (void)uart_legacy_display_write(str, (uint16_t)sizeof(str));
    return 0u;
}

/**
 * @brief 获取显示信息快照
 * 
 * @param out 输出显示信息指针
 * @return uint8_t 0表示成功，1表示失败
 */
uint8_t display_handler_snapshot(DISPLAY_INFO *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 1u;
    }

    primask = display_irq_lock();
    *out = display_info;
    display_irq_unlock(primask);

    return 0u;
}

/**
 * @brief 设置机器人模式
 * 
 * @param main_mode 主模式
 * @param sub_mode 子模式
 */
void display_handler_set_robot_mode(uint8_t main_mode, uint8_t sub_mode)
{
    uint32_t primask = display_irq_lock();
    s_robot_mode_main = main_mode;
    s_robot_mode_sub = sub_mode;
    display_irq_unlock(primask);
}

/**
 * @brief 轮询显示处理
 * 
 */
void display_handler_poll(void)
{
    uint8_t has_pending = 0u;
    uint8_t main_mode = 0u;
    uint8_t sub_mode = 0u;
    uint32_t primask = display_irq_lock();

    if (s_read_data01_pending != 0u)
    {
        s_read_data01_pending = 0u;
        has_pending = 1u;
    }
    main_mode = s_robot_mode_main;
    sub_mode = s_robot_mode_sub;

    display_irq_unlock(primask);

    if (has_pending != 0u)
    {
        (void)display_answer_read_data01(main_mode, sub_mode);
    }
}

/**
 * @brief 处理显示回调
 * 
 * @param data 显示数据指针
 * @param len 数据长度
 */
void display_callback(const uint8_t *data, uint8_t len)
{
    uint32_t primask;

    if ((data == NULL) || (len < DISPLAY_FRAME)) {
        return;
    }

    if ((data[0] != DISPLAY_HEADER_1) || (data[1] != DISPLAY_HEADER_2))
    {
        return;
    }

    if (data[2] == 0x55u) {
        primask = display_irq_lock();
        s_read_data01_pending = 1u;
        display_irq_unlock(primask);
        return;
    }

    if (data[2] != 0xaau || data[4] != 1u) {
        return;
    }

    primask = display_irq_lock();
    display_info.moniter_state = 1u;

    switch (data[5])
    {
        case 0x00u:
            display_info.moniter_mode = 0u;
            break;

        case 0x02u:
            display_info.moniter_mode = 2u;
            break;

        case 0x03u:
            display_info.moniter_mode = 3u;
            break;

        default:
            break;
    }
    display_irq_unlock(primask);
}

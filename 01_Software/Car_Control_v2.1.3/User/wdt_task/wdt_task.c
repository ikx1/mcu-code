/********************************** Includes *********************************/
#include "wdt_task.h"

/********************************** Defines **********************************/

/********************************** Variables ********************************/
typedef struct
{
    const char *name;
    uint32_t max_period_tick;
    uint32_t last_update;
    uint8_t enable;
} monitor_task_t;

typedef struct
{
    bool inited;
    SemaphoreHandle_t lock;
    TaskHandle_t thread;
    // st_drv_wdt_channel_id channel_id;
    uint32_t task_num;
    monitor_task_t tasks[WDT_MONITOR_TASK_MAX];
} wdt_monitor_data_t;

wdt_monitor_data_t wdt_data = {
    .inited = false,
    .lock = NULL,
    .thread = NULL,
    .task_num = 0,
};

/********************************** Functions ********************************/

/**
 * @brief 使系统进入挂起状态，等待独立看门狗 (IWDG) 触发复位。
 *
 * 此函数会打印当前函数名信息，然后进入一个无限循环，
 * 在循环中启用调试监视器并触发断点，以此让系统保持挂起状态，
 * 等待独立看门狗超时触发复位操作。
 */
static void system_hang_up(void)
{
    // // 打印当前函数名信息，提示系统已进入挂起状态
    // NRF_LOG_INFO("%s !!!\r\n", __FUNCTION__);

    /************* wait for IWDG trigge ******************/
    // 进入无限循环，使系统保持挂起状态，等待独立看门狗触发复位
    while (1)
    {
        // 启用调试监视器的跟踪功能，设置 DEMCR 寄存器的第 16 位
        CoreDebug->DEMCR |= 1 << 16U;
        // 触发断点，暂停程序执行
        __BKPT(1);
    }
}

/**
 * @brief check monited thread overflow
 * @param void NULL
 * @retval bool
 */
static bool monitor_task_timeout_check(void)
{
    ret_code_t ret = WDT_SUCCESS;
    bool timeout = false;

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    uint32_t current_time = WDT_TICK;

    /************* check monited thread overflow *********/
    for (uint32_t i = 0; i < wdt_data.task_num; i++)
    {
        if ((wdt_data.tasks[i].enable) &&
            ((WDT_TICK - wdt_data.tasks[i].last_update) >
             wdt_data.tasks[i].max_period_tick))
        {
            // NRF_LOG_INFO("Task %s timeout!!!", wdt_data.tasks[i].name);
            timeout = true;
        }
    }

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return timeout;
}

/**
 * @brief initialize the wdt module
 * @param void NULL
 * @retval return result
 */
ret_code_t wdt_monitor_init(void)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check reinit **************************/
    if (WDT_INITED == wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor reinit\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* create thread locker ******************/
    if (NULL == wdt_data.lock)
        wdt_data.lock = xSemaphoreCreateMutex();

    if (NULL == wdt_data.lock)
    {
        // NRF_LOG_INFO("[%s] create lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_NO_MEM;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* hardware initial **********************/
    hal_wdt_init();

    hal_wdt_enable();

    wdt_data.inited = true;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief registe thread monitor
 * @param name monited thread name
 * @param max_period max timeout term(ms)
 * @param out_id monited thread id (storage in monited thread)
 * @retval return result
 */
ret_code_t wdt_monitor_task_register(const char *name,
                                     uint32_t max_period,
                                     uint32_t *out_id)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check param  **************************/
    if ((NULL == name) || (NULL == out_id))
    {
        // NRF_LOG_INFO(" [%s] invalid param !!! \r\n", __FUNCTION__);
        return WDT_ERROR_INVALID_PARAM;
    }

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* register thread monitor ***************/
    wdt_data.tasks[wdt_data.task_num].name = name;
    wdt_data.tasks[wdt_data.task_num].max_period_tick = max_period;
    wdt_data.tasks[wdt_data.task_num].enable = false;
    wdt_data.tasks[wdt_data.task_num].last_update = WDT_TICK;

    *out_id = wdt_data.task_num;
    wdt_data.task_num++;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief enable thread monitor
 * @param id monited thread id (storage in monited thread)
 * @retval return result
 */
ret_code_t wdt_monitor_task_enable(uint32_t id)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check param **************************/
    if (id >= WDT_MONITOR_TASK_MAX)
    {
        // NRF_LOG_INFO(" [%s] invalid param !!! \r\n", __FUNCTION__);
        return WDT_ERROR_INVALID_PARAM;
    }

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* register monited thread ***************/
    wdt_data.tasks[id].enable = true;
    wdt_data.tasks[id].last_update = WDT_TICK;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief disable thread monitor
 * @param id monited thread id (storage in monited thread)
 * @retval return result
 */
ret_code_t wdt_monitor_task_disable(uint32_t id)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check param **************************/
    if (id >= WDT_MONITOR_TASK_MAX)
    {
        // NRF_LOG_INFO(" [%s] invalid param !!! \r\n", __FUNCTION__);
        return WDT_ERROR_INVALID_PARAM;
    }

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* register monited thread ***************/
    wdt_data.tasks[id].enable = false;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief disable thread monitor
 * @param id monited thread id (storage in monited thread)
 * @param max_period max timeout term(ms)
 * @retval return result
 */
ret_code_t wdt_monitor_task_period_set(uint32_t id, uint32_t max_period)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check param **************************/
    if (id >= WDT_MONITOR_TASK_MAX)
    {
        // NRF_LOG_INFO(" [%s] invalid param !!! \r\n", __FUNCTION__);
        return WDT_ERROR_INVALID_PARAM;
    }

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* register thread monitor ***************/
    wdt_data.tasks[id].max_period_tick = max_period;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief feed thread monitor
 * @param id monited thread id (storage in monited thread)
 * @retval return result
 */
ret_code_t wdt_monitor_task_feed(uint32_t id)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check param **************************/
    if (id >= WDT_MONITOR_TASK_MAX)
    {
        // NRF_LOG_INFO(" [%s] invalid param !!! \r\n", __FUNCTION__);
        return WDT_ERROR_INVALID_PARAM;
    }

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    /************* take thread lock **********************/
    if (pdTRUE != xSemaphoreTake(wdt_data.lock, portMAX_DELAY))
    {
        // NRF_LOG_INFO("[%s] wdt take lock fail !!! \r\n", __FUNCTION__);
        ret = WDT_ERROR_BUSY;
        return ret;
    }

    /************* register thread monitor ***************/
    wdt_data.tasks[id].last_update = WDT_TICK;

    /************* give thread lock **********************/
    xSemaphoreGive(wdt_data.lock);

    return ret;
}

/**
 * @brief notify monitor thread
 * @param void NULL
 * @retval return result
 */
ret_code_t wdt_monitor_notify(void)
{
    ret_code_t ret = WDT_SUCCESS;

    /************* check init state***********************/
    if (WDT_INITED != wdt_data.inited)
    {
        // NRF_LOG_INFO("[%s] wdt monitor not init\r\n", __FUNCTION__);
        ret = WDT_ERROR_INVALID_STATE;
        return ret;
    }

    xTaskNotifyGive(wdt_data.thread);

    return ret;
}

//********************* monitor thread **********************************//
/**
 * @brief thread loop function
 * @param arg thread input arguments
 * @retval return result
 */
void wdt_monitor_thread(void *arg)
{
    
    TickType_t xTicksToWait = pdMS_TO_TICKS(WDT_MONITOR_WAIT_MS);
    wdt_monitor_init();

    while (1)
    {
        if (monitor_task_timeout_check())
        {
            system_hang_up();
        }
        else
        {
            hal_wdt_feed();
        }

        (void)ulTaskNotifyTake(pdTRUE, xTicksToWait);
    }
}

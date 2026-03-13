/*** 
 * @file: Do not edit
 * @par: 
 * @author: ikk
 * @Date: 2024-11-07 11:19:33
 * @LastEditTime: 2024-11-07 11:25:50
 * @LastEditors: ikk
 * @description: 
 */
#ifndef __LOG_H__
#define __LOG_H__

#include "SEGGER_RTT.h"

#define LOG_DEBUG 1

#ifdef LOG_DEBUG


#define LOG_PROTO(type,color,format,...)            \
        SEGGER_RTT_printf(0,"  %s%s"format"\r\n%s", \
                          color,                    \
                          type,                     \
                          ##__VA_ARGS__,            \
                          RTT_CTRL_RESET)

/* 清屏*/
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  "RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format,...) LOG_PROTO("","",format,##__VA_ARGS__)

/* 有颜色格式日志输出 */
#define LOGI(format,...) LOG_PROTO("I: ", RTT_CTRL_TEXT_BRIGHT_GREEN , format, ##__VA_ARGS__)
#define LOGW(format,...) LOG_PROTO("W: ", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define LOGE(format,...) LOG_PROTO("E: ", RTT_CTRL_TEXT_BRIGHT_RED   , format, ##__VA_ARGS__)

#endif /* LOG_DEBUG end  */

#ifndef LOG_DEBUG

#define LOG_CLEAR()
#define LOG
#define LOGI
#define LOGW
#define LOGE
#endif 


#endif /* LOG_H END of */

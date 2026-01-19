
/*
 * Auto generated Run-Time-Environment Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'Car_Control' 
 * Target:  'Car_Control' 
 */

#ifndef PRE_INCLUDE_GLOBAL_H
#define PRE_INCLUDE_GLOBAL_H

/* GorgonMeducer::Utilities:perf_counter:Core:Source:2.4.0 */
#define __PERF_COUNTER_CFG_USE_SYSTICK_WRAPPER__ 1
/* GorgonMeducer::Utilities:perf_counter:FreeRTOS Patch:2.4.0 */
//! \brief Enable RTOS Patch for perf_counter
#define __PERF_CNT_USE_RTOS__ 1
            
#define traceTASK_SWITCHED_OUT_DISABLE  
#define traceTASK_SWITCHED_IN_DISABLE

extern void __freertos_evr_on_task_switched_out (void *ptTCB);
extern void __freertos_evr_on_task_switched_in(void *ptTCB, unsigned int uxTopPriority) ;

#   define traceTASK_SWITCHED_OUT()                                             \
        __freertos_evr_on_task_switched_out(pxCurrentTCB)
#   define traceTASK_SWITCHED_IN()                                              \
        __freertos_evr_on_task_switched_in(pxCurrentTCB, uxTopReadyPriority)
/* GorgonMeducer::Utilities:perf_counter:Porting:User Defined:1.0.3 */
#define __PERFC_USE_PORTING__                           1
#define __PERFC_USE_USER_CUSTOM_PORTING__               1
#define __PERFC_CFG_DISABLE_DEFAULT_SYSTICK_PORTING__   1
#define __PERFC_CFG_PORTING_INCLUDE__                   "perfc_port_user.h"


#endif /* PRE_INCLUDE_GLOBAL_H */

/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file event_manager.h
 * @brief Event manager module
 *
 * This module is responsible for the management of events during power supplies
 * operation, including data log on onboard memory. An event is generated by any
 * of the following situations:
 *
 *      1. Interlocks
 *      2. Alarms, in general caused by unusual operation values
 *      3. Commands received via communication interfaces, such as turn on/off,
 *         selection of operation mode, open/close control loop, changes on
 *         setpoint or other operation paramenters, etc. Usually is done by BSMP
 *         functions or HMI operation.
 *
 * Current version implements only interlocks management, including debouncing
 * logic.
 *
 * TODO: Events based on alarms and commands, and data log.
 * 
 * @author gabriel.brunheira
 * @date 14/08/2018
 *
 */

#ifndef EVENT_MANAGER_H_
#define EVENT_MANAGER_H_

#include <stdint.h>
#include "ps_modules/ps_modules.h"

/**
 *  This define globally enables/disables interlocks on ps_modules. When
 *  disabled, interlock variables still indicate which interlock has occured,
 *  but the power supply can still operate normally
 */
#define USE_ITLK

#define NUM_MAX_EVENT_COUNTER       32

#define NUM_MAX_HARD_INTERLOCKS     32
#define NUM_MAX_SOFT_INTERLOCKS     32

#define HARD_INTERLOCKS_DEBOUNCE_TIME   g_ipc_mtoc.interlocks.hard_itlks_debounce_time
#define HARD_INTERLOCKS_RESET_TIME      g_ipc_mtoc.interlocks.hard_itlks_reset_time
#define SOFT_INTERLOCKS_DEBOUNCE_TIME   g_ipc_mtoc.interlocks.soft_itlks_debounce_time
#define SOFT_INTERLOCKS_RESET_TIME      g_ipc_mtoc.interlocks.soft_itlks_reset_time

/**
 * This define must be included inside the time-base routine (for example, a
 * control ISR).
 */
#define SET_INTERLOCKS_TIMEBASE_FLAG(id)    g_event_manager[id].timebase_flag = 1;

typedef struct
{
    uint16_t flag;
    uint32_t counter;
    uint32_t debounce_count;
    uint32_t reset_count;
} debounce_counter_t;

typedef struct
{
    uint16_t num_events;
    debounce_counter_t event[NUM_MAX_EVENT_COUNTER];
} debounce_counters_t;

typedef struct
{
    uint16_t timebase_flag;
    float freq_timebase;
    debounce_counters_t hard_interlocks;
    debounce_counters_t soft_interlocks;
} event_manager_t;

extern volatile event_manager_t g_event_manager[NUM_MAX_PS_MODULES];

extern void init_event_manager(uint16_t id, float freq_timebase,
                               uint16_t num_hard_itlks, uint16_t num_soft_itlks,
                               uint32_t *hard_itlks_debounce_time_us,
                               uint32_t *hard_itlks_reset_time_us,
                               uint32_t *soft_itlks_debounce_time_us,
                               uint32_t *soft_itlks_reset_time_us);

extern void run_interlocks_debouncing(uint16_t id);

extern void set_hard_interlock(uint16_t id, uint32_t itlk);
extern void set_soft_interlock(uint16_t id, uint32_t itlk);
extern interrupt void isr_hard_interlock(void);
extern interrupt void isr_soft_interlock(void);
extern interrupt void isr_interlocks_timebase(void);

#endif /* EVENT_MANAGER_H_ */

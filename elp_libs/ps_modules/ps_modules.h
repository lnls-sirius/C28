/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file ps_modules.h
 * @brief Power supplies modules.
 * 
 * Main source file for power supply modules. It includes macros and enumerates
 * related to operation of power supplies from ELP group on Sirius Project.
 *
 * @author gabriel.brunheira
 * @date 25/10/2017
 *
 */

#ifndef PS_MODULES_H_
#define PS_MODULES_H_

#include <stdint.h>
//#include <string.h>

#define CLOSED_LOOP         0
#define OPEN_LOOP           1

#define INACTIVE            0
#define ACTIVE              1

#define LOCKED              0
#define UNLOCKED            1

#define NUM_MAX_PS_MODULES  4

typedef enum
{
    Off,
    Interlock,
    Initializing,
    SlowRef,
    SlowRefSync,
    Cycle,
    RmpWfm,
    MigWfm,
    FastRef
} ps_state_t;

typedef enum
{
    Remote,
    Local,
    PCHost
} ps_interface_t;

typedef enum
{
    Empty,
    FBP,
    FBP_DCLink,
    FAC_ACDC,
    FAC_DCDC,
    FAC_2S_ACDC,
    FAC_2S_DCDC,
    FAC_2P4S_ACDC,
    FAC_2P4S_DCDC,
    FAP,
    FAP_4P,
    FAC_DCDC_EMA,
    FAP_2P2S,
    FAP_IMAS,
    FAC_2P_ACDC_IMAS,
    FAC_2P_DCDC_IMAS,
    Uninitialized = 0x1F      // Empty EEPROM
} ps_model_t;

typedef struct
{
    ps_state_t      state      : 4;    // 3:0      Operation state
    uint16_t        openloop   : 1;    // 4        Control loop state
    ps_interface_t  interface  : 2;    // 6:5      Communication interface
    uint16_t        active     : 1;    // 7        Power supply active?
    ps_model_t      model      : 5;    // 12:8     Power supply model
    uint16_t        unlocked   : 1;    // 13       Unlocked?
    uint16_t        reserved   : 2;    // 15:14    Reserved for future use
} ps_status_bits_t;

typedef union
{
    uint16_t            all;
    ps_status_bits_t    bit;
} ps_status_t;

typedef struct
{
    ps_status_t     ps_status;
    float           ps_setpoint;
    float           ps_reference;
    uint32_t        ps_hard_interlock;
    uint32_t        ps_soft_interlock;
    uint32_t        ps_alarms;
    void            (*turn_on)(uint16_t);
    void            (*turn_off)(uint16_t);
    void            (*isr_soft_interlock)(void);
    void            (*isr_hard_interlock)(void);
    void            (*reset_interlocks)(uint16_t id);
} ps_module_t;

extern void init_ps_module(ps_module_t *p_ps_module, ps_model_t model,
                           void (*turn_on)(uint16_t), void (*turn_off)(uint16_t),
                           void (*isr_soft_interlock)(void),
                           void (*isr_hard_interlock)(void),
                           void (*reset_interlocks)(uint16_t id));
extern void cfg_ps_operation_mode(ps_module_t *p_ps_module, ps_state_t op_mode);
extern void open_loop(ps_module_t *p_ps_module);
extern void close_loop(ps_module_t *p_ps_module);
extern void cfg_ps_inteface(ps_module_t *p_ps_module, ps_interface_t interface);
extern void activate_ps_module(ps_module_t *p_ps_module);
extern void deactivate_ps_module(ps_module_t *p_ps_module);
extern void lock_ps_module(ps_module_t *p_ps_module);
extern void unlock_ps_module(ps_module_t *p_ps_module);
extern ps_model_t get_ps_model(ps_module_t *p_ps_module);

#endif /* PS_MODULES_H_ */

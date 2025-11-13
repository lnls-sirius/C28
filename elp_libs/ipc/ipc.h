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
 * @file ipc.h
 * @brief Interprocessor Communication module
 *
 * This module is responsible for definition of interprocessor communication
 * functionalities, between ARM and C28 cores.
 * 
 * @author gabriel.brunheira
 * @date 22/11/2017
 *
 */

#ifndef IPC_H_
#define IPC_H_

#include <stdint.h>
#include "boards/version.h"
#include "common/structs.h"
#include "ps_modules/ps_modules.h"
#include "siggen/siggen.h"
#include "wfmref/wfmref.h"
#include "control/control.h"
#include "parameters/parameters.h"
#include "scope/scope.h"

/**
 * Synchronization defines
 */
#define MIN_NUM_ISR_CONTROLLER_SYNC     1000
#define MAX_NUM_ISR_CONTROLLER_SYNC     4294967295  // 0xFFFFFFFF

/**
 * Shared resources defines
 */

#define SIZE_BUF_SAMPLES_CTOM   4096
#define SIZE_BUF_SAMPLES_MTOC   4096

#define SIGGEN_CTOM     g_ipc_ctom.siggen
#define SIGGEN_MTOC     g_ipc_mtoc.siggen

#define WFMREF_CTOM     g_ipc_ctom.wfmref
#define WFMREF_MTOC     g_ipc_mtoc.wfmref

#define SCOPE_CTOM      g_ipc_ctom.scope
#define SCOPE_MTOC      g_ipc_mtoc.scope

/**
 * IPC Message Defines
 */

#define IPC_MTOC_LOWPRIORITY_MSG    0x00000001  // IPC1
#define IPC_CTOM_LOWPRIORITY_MSG    0x00000001  // IPC1
#define SYNC_PULSE                  0x00000002  // IPC2
#define HARD_INTERLOCK              0x00000004  // IPC3
#define SOFT_INTERLOCK              0x00000008  // IPC4

typedef enum
{
    Turn_On = 1,
    Turn_Off,
    Open_Loop,
    Close_Loop,
    Operating_Mode,
    Reset_Interlocks,
    Unlock_UDC,
    Lock_UDC,
    Cfg_Source_Scope,
    Cfg_Freq_Scope,
    Cfg_Duration_Scope,
    Enable_Scope,
    Disable_Scope,
    Reset_Scope,
    Set_SlowRef,
    Set_SlowRef_All_PS,
    Cfg_WfmRef,
    Update_WfmRef,
    Reset_WfmRef,
    Cfg_SigGen,
    Set_SigGen,
    Enable_SigGen,
    Disable_SigGen,
    Reset_Counters,
    Set_Param,
    Set_DSP_Coeffs,
    Cfg_TimeSlicer,
    Set_Command_Interface,
    CtoM_Message_Error
} ipc_mtoc_lowpriority_msg_t;

typedef enum
{   Enable_HRADC_Boards,
    Disable_HRADC_Boards,
    MtoC_Message_Error
} ipc_ctom_lowpriority_msg_t;

#define GET_IPC_MTOC_LOWPRIORITY_MSG  (ipc_mtoc_lowpriority_msg_t) (g_ipc_ctom.msg_mtoc >> 4 ) & 0x0000FFFF

typedef enum
{
    No_Error_CtoM,
    Error1,
    Error2,
    Error3,
    Error4
} error_ctom_t;

typedef enum
{
    No_Error_MtoC,
    Invalid_Argument,
    Invalid_OpMode,
    IPC_LowPriority_Full,
    HRADC_Config_Error
} error_mtoc_t;

/**
 * IPC structures definitions
 */
typedef struct
{
    char            udc_c28_version[SIZE_VERSION]; // C28 char = 2 bytes
    uint32_t        msg_mtoc;
    uint16_t        msg_id;
    error_mtoc_t    error_mtoc;
    uint32_t        counter_set_slowref;
    uint32_t        counter_sync_pulse;
    uint32_t        period_sync_pulse;
    ps_module_t     ps_module[NUM_MAX_PS_MODULES];
    siggen_t        siggen[NUM_MAX_PS_MODULES];
    wfmref_t        wfmref[NUM_MAX_PS_MODULES];
    scope_t         scope[NUM_MAX_SCOPES];
} ipc_ctom_t;

typedef struct
{
    uint32_t                msg_ctom;
    uint16_t                msg_id;
    error_ctom_t            error_ctom;
    uint32_t                ps_name[SIZE_PS_NAME];
    ps_model_t              ps_model;
    uint16_t                num_ps_modules;
    ps_module_t             ps_module[NUM_MAX_PS_MODULES];
    siggen_t                siggen[NUM_MAX_PS_MODULES];
    wfmref_t                wfmref[NUM_MAX_PS_MODULES];
    scope_t                 scope[NUM_MAX_SCOPES];
    dsp_module_t            dsp_module;
    //param_control_t         control;
    //param_pwm_t             pwm;
    //param_hradc_t           hradc;
    //param_analog_vars_t     analog_vars;
    //param_communication_t   communication;
    //param_interlocks_t      interlocks;
} ipc_mtoc_t;

extern volatile float g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM];
extern volatile float g_buf_samples_mtoc[SIZE_BUF_SAMPLES_MTOC];

extern volatile ipc_ctom_t g_ipc_ctom;
extern volatile ipc_mtoc_t g_ipc_mtoc;

extern volatile uint32_t counter_sync_period;

extern void init_ipc(void);
extern void send_ipc_msg(uint16_t msg_id, uint32_t msg);
extern void send_ipc_lowpriority_msg(uint16_t msg_id,
                                     ipc_ctom_lowpriority_msg_t msg);

#endif /* IPC_H_ */

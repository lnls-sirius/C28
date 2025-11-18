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
 * @file parameters.h
 * @brief Power supply parameters bank module.
 *
 * This module implements a data structure for initialization and configuration
 * of parameters for operation of the power supplies applications.
 *
 * @author gabriel.brunheira
 * @date 23/02/2018
 *
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>
#include <math.h>
#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "event_manager/event_manager.h"
#include "ps_modules/ps_modules.h"
#include "scope/scope.h"
#include "siggen/siggen.h"

/// Different Size due to 32 bits definition
#define SIZE_PS_NAME            16

#define NUM_MAX_TIMESLICERS     4

#define NUM_MAX_ANALOG_VAR      64
#define NUM_MAX_DIGITAL_VAR     12
#define NUM_MAX_HRADC           4

#define NUM_MAX_HARD_INTERLOCKS     32
#define NUM_MAX_SOFT_INTERLOCKS     32

#define NUM_PARAMETERS          54
#define NUM_MAX_PARAMETERS      64
#define NUM_MAX_FLOATS          200

/**
 * General info
 */
#define PS_NAME                 g_param_bank.ps_name
#define PS_MODEL                g_param_bank.ps_model
#define NUM_PS_MODULES          g_param_bank.num_ps_modules

/**
 * Communication parameters
 */
#define RS485_BAUDRATE          g_param_bank.communication.rs485_baud
#define RS485_ADDRESS           g_param_bank.communication.rs485_address
#define RS485_TERMINATION       g_param_bank.communication.rs485_termination
#define UDCNET_ADDRESS          g_param_bank.communication.udcnet_address
#define ETHERNET_IP             g_param_bank.communication.ethernet_ip
#define ETHERNET_MASK           g_param_bank.communication.ethernet_mask
#define BUZZER_VOLUME           g_param_bank.communication.buzzer_volume
#define COMMAND_INTERFACE       g_param_bank.communication.command_interface

/**
 * Control parameters
 */
#define ISR_CONTROL_FREQ            g_param_bank.control.freq_isr_control
#define TIMESLICER_FREQ             g_param_bank.control.freq_timeslicer
#define LOOP_STATE                  g_param_bank.control.loop_state

#define MAX_REF                     g_param_bank.control.max_ref
#define MIN_REF                     g_param_bank.control.min_ref
#define MAX_REF_OL                  g_param_bank.control.max_ref_openloop
#define MIN_REF_OL                  g_param_bank.control.min_ref_openloop
#define MAX_CURRENT_STEP            g_param_bank.control.max_current_step
/*#define MAX_SLEWRATE_SLOWREF        g_param_bank.control.slewrate_slowref
#define MAX_SLEWRATE_SIGGEN_AMP     g_param_bank.control.slewrate_siggen_amp
#define MAX_SLEWRATE_SIGGEN_OFFSET  g_param_bank.control.slewrate_siggen_offset
#define MAX_SLEWRATE_WFMREF         g_param_bank.control.slewrate_wfmref*/

/**
 * PWM parameters
 */
#define PWM_FREQ                    g_param_bank.pwm.freq_pwm
#define PWM_DEAD_TIME               g_param_bank.pwm.dead_time
#define PWM_MAX_DUTY                g_param_bank.pwm.max_duty
#define PWM_MIN_DUTY                g_param_bank.pwm.min_duty
#define PWM_MAX_DUTY_OL             g_param_bank.pwm.max_duty_openloop
#define PWM_MIN_DUTY_OL             g_param_bank.pwm.min_duty_openloop
#define PWM_LIM_DUTY_SHARE          g_param_bank.pwm.lim_duty_share

/**
 * HRADC parameters
 */
#define HRADC_FREQ_SAMP             g_param_bank.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK               g_param_bank.hradc.freq_spiclk
#define NUM_HRADC_BOARDS            g_param_bank.hradc.num_hradc

#define HRADC_HEATER_ENABLE         g_param_bank.hradc.enable_heater
#define HRADC_MONITOR_ENABLE        g_param_bank.hradc.enable_monitor
#define TRANSDUCER_OUTPUT_TYPE      g_param_bank.hradc.type_transducer_output
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN     -g_p_bank.hradc.gain_transducer
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN         g_param_bank.hradc.gain_transducer
#endif

/**
 * SigGen parameters
 */
#define SIGGEN_TYPE_PARAM           g_param_bank.siggen.type
#define SIGGEN_NUM_CYCLES_PARAM     g_param_bank.siggen.num_cycles
#define SIGGEN_FREQ_PARAM           g_param_bank.siggen.freq
#define SIGGEN_AMP_PARAM            g_param_bank.siggen.amplitude
#define SIGGEN_OFFSET_PARAM         g_param_bank.siggen.offset
#define SIGGEN_AUX_PARAM            g_param_bank.siggen.aux_param

/**
 * WfmRef parameters
 */
#define WFMREF_SELECTED_PARAM       g_param_bank.wfmref.selected
#define WFMREF_SYNC_MODE_PARAM      g_param_bank.wfmref.sync_mode
#define WFMREF_FREQUENCY_PARAM      g_param_bank.wfmref.frequency
#define WFMREF_GAIN_PARAM           g_param_bank.wfmref.gain
#define WFMREF_OFFSET_PARAM         g_param_bank.wfmref.offset

/**
 * Analog Variables parameters
 */
#define ANALOG_VARS_MAX             g_param_bank.analog_vars.max
#define ANALOG_VARS_MIN             g_param_bank.analog_vars.min

/**
 * Interlocks parameters
 */
#define HARD_INTERLOCKS_DEBOUNCE_TIME   g_param_bank.interlocks.hard_itlks_debounce_time
#define HARD_INTERLOCKS_RESET_TIME      g_param_bank.interlocks.hard_itlks_reset_time
#define SOFT_INTERLOCKS_DEBOUNCE_TIME   g_param_bank.interlocks.soft_itlks_debounce_time
#define SOFT_INTERLOCKS_RESET_TIME      g_param_bank.interlocks.soft_itlks_reset_time

/**
 * Scope parameters
 */
#define SCOPE_FREQ_SAMPLING_PARAM   g_param_bank.scope.freq_sampling
#define SCOPE_SOURCE_PARAM          g_param_bank.scope.p_source

typedef enum
{
    PS_Name,
    PS_Model,
    Num_PS_Modules,

    Command_Interface,
    RS485_Baudrate,
    RS485_Address,
    RS485_Termination,
    UDCNet_Address,
    Ethernet_IP,
    Ethernet_Subnet_Mask,
    Buzzer_Volume,

    Freq_ISR_Controller,
    Freq_TimeSlicer,
    Control_Loop_State,
    Max_Ref,
    Min_Ref,
    Max_Ref_OpenLoop,
    Min_Ref_OpenLoop,
    Max_Current_Step,
    //Max_SlewRate_SlowRef,
    //Max_SlewRate_SigGen_Amp,
    //Max_SlewRate_SigGen_Offset,
    //Max_SlewRate_WfmRef,

    PWM_Freq,
    PWM_DeadTime,
    PWM_Max_Duty,
    PWM_Min_Duty,
    PWM_Max_Duty_OpenLoop,
    PWM_Min_Duty_OpenLoop,
    PWM_Lim_Duty_Share,

    HRADC_Num_Boards,
    HRADC_Freq_SPICLK,
    HRADC_Freq_Sampling,
    HRADC_Enable_Heater,
    HRADC_Enable_Monitor,
    HRADC_Type_Transducer,
    HRADC_Gain_Transducer,
    HRADC_Offset_Transducer,

    SigGen_Type,
    SigGen_Num_Cycles,
    SigGen_Freq,
    SigGen_Amplitude,
    SigGen_Offset,
    SigGen_Aux_Param,

    WfmRef_Selected,
    WfmRef_SyncMode,
    WfmRef_Frequency,
    WfmRef_Gain,
    WfmRef_Offset,

    Analog_Var_Max,
    Analog_Var_Min,

    Hard_Interlocks_Debounce_Time,
    Hard_Interlocks_Reset_Time,
    Soft_Interlocks_Debounce_Time,
    Soft_Interlocks_Reset_Time,

    Scope_Sampling_Frequency,
    Scope_Source
} param_id_t;

typedef enum
{
    is_uint16_t,
    is_uint32_t,
    is_float
} param_type_t;

typedef union
{
    uint16_t    *u16;
    uint32_t    *u32;
    float       *f;
} p_param_t;

typedef struct
{
    param_id_t      id;
    param_type_t    type;
    uint16_t        num_elements;
    p_param_t       p_val;
} param_t;

typedef struct
{
    float           rs485_baud;
    uint16_t        rs485_address[NUM_MAX_PS_MODULES];
    uint16_t        rs485_termination;
    uint16_t        udcnet_address;
    uint32_t        ethernet_ip;
    uint32_t        ethernet_mask;
    uint16_t        buzzer_volume;
    uint16_t        command_interface;
} param_communication_t;

typedef struct
{
    uint16_t loop_state;
    float   freq_isr_control;
    float   freq_timeslicer[NUM_MAX_TIMESLICERS];
    float   max_ref[NUM_MAX_PS_MODULES];
    float   min_ref[NUM_MAX_PS_MODULES];
    float   max_ref_openloop[NUM_MAX_PS_MODULES];
    float   min_ref_openloop[NUM_MAX_PS_MODULES];
    float   max_current_step[NUM_MAX_PS_MODULES];
    float   slewrate_slowref;
    float   slewrate_siggen_amp;
    float   slewrate_siggen_offset;
    float   slewrate_wfmref;
} param_control_t;

typedef struct
{
    float   freq_pwm;
    float   dead_time;
    float   max_duty;
    float   min_duty;
    float   max_duty_openloop;
    float   min_duty_openloop;
    float   lim_duty_share;
} param_pwm_t;

typedef struct
{
    uint16_t    num_hradc;
    uint16_t    freq_spiclk;
    float       freq_hradc_sampling;
    uint16_t    enable_heater[NUM_MAX_HRADC];
    uint16_t    enable_monitor[NUM_MAX_HRADC];
    uint16_t    type_transducer_output[NUM_MAX_HRADC];
    float       gain_transducer[NUM_MAX_HRADC];
    float       offset_transducer[NUM_MAX_HRADC];
} param_hradc_t;

typedef struct
{
    float   max[NUM_MAX_ANALOG_VAR];
    float   min[NUM_MAX_ANALOG_VAR];
} param_analog_vars_t;

typedef struct
{
    uint32_t    hard_itlks_debounce_time[NUM_MAX_HARD_INTERLOCKS];
    uint32_t    hard_itlks_reset_time[NUM_MAX_HARD_INTERLOCKS];
    uint32_t    soft_itlks_debounce_time[NUM_MAX_SOFT_INTERLOCKS];
    uint32_t    soft_itlks_reset_time[NUM_MAX_SOFT_INTERLOCKS];
} param_interlocks_t;

typedef struct
{
    uint16_t  type;
    uint16_t  num_cycles;
    float     freq;
    float     amplitude;
    float     offset;
    float     aux_param[NUM_SIGGEN_AUX_PARAM];
} param_siggen_t;

typedef struct
{
    uint16_t  selected[NUM_MAX_PS_MODULES];
    uint16_t  sync_mode[NUM_MAX_PS_MODULES];
    float     frequency[NUM_MAX_PS_MODULES];
    float     gain[NUM_MAX_PS_MODULES];
    float     offset[NUM_MAX_PS_MODULES];
} param_wfmref_t;


typedef struct
{
    float   freq_sampling[NUM_MAX_SCOPES];
    float   *p_source[NUM_MAX_SCOPES];
} param_scope_t;

typedef struct
{
    param_t                 param_info[NUM_MAX_PARAMETERS];
    uint32_t                ps_name[SIZE_PS_NAME];
    uint16_t                ps_model;
    uint16_t                num_ps_modules;
    param_communication_t   communication;
    param_control_t         control;
    param_pwm_t             pwm;
    param_hradc_t           hradc;
    param_siggen_t          siggen;
    param_wfmref_t          wfmref;
    param_analog_vars_t     analog_vars;
    param_interlocks_t      interlocks;
    param_scope_t           scope;
} param_bank_t;

//extern volatile param_t g_parameters[NUM_MAX_PARAMETERS];

extern volatile param_bank_t g_param_bank;

extern void init_param(param_id_t id, param_type_t type, uint16_t num_elements,
                       uint16_t *p_param);
extern uint16_t set_param(param_id_t id, uint16_t n, float val);
extern float get_param(param_id_t id, uint16_t n);

#endif /* PARAMETERS_H_ */

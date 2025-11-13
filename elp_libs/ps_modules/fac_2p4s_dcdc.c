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
 * @file fac_2p4s_dcdc.c
 * @brief FAC-2P4S DC/DC Stage module
 * 
 * Module for control of DC/DC module of FAC power supplies. It implements the
 * controller for load current.
 *
 * PWM signals are mapped as the following :
 *
 *      ePWM  =>  Signal   ( POF transmitter)
 *     channel     Name    (    on BCB      )
 *
 *     ePWM1A => Q1_MOD_1        (PWM1)
 *     ePWM1B => Q1_MOD_5        (PWM2)
 *     ePWM2A => Q2_MOD_1        (PWM3)
 *     ePWM2B => Q2_MOD_5        (PWM4)
 *     ePWM3A => Q1_MOD_2        (PWM5)
 *     ePWM3B => Q1_MOD_6        (PWM6)
 *     ePWM4A => Q2_MOD_2        (PWM7)
 *     ePWM4B => Q2_MOD_6        (PWM8)
 *     ePWM5A => Q1_MOD_3        (PWM9)
 *     ePWM5B => Q1_MOD_7        (PWM10)
 *     ePWM6A => Q2_MOD_3        (PWM11)
 *     ePWM6B => Q2_MOD_7        (PWM12)
 *     ePWM7A => Q1_MOD_4        (PWM13)
 *     ePWM7B => Q1_MOD_8        (PWM14)
 *     ePWM8A => Q2_MOD_4        (PWM15)
 *     ePWM8B => Q2_MOD_8        (PWM16)
 *
 *  TODO: Include reference filtering and feedforward, capacitor banks voltage
 *  feedforward and modules output voltage share control.
 *
 * @author gabriel.brunheira
 * @date 01/05/2018
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/structs.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"
#include "wfmref/wfmref.h"

#include "fac_2p4s_dcdc.h"

/**
 * Analog variables parameters
 */
#define MAX_ILOAD                               ANALOG_VARS_MAX[0]

#define MAX_V_CAPBANK                           ANALOG_VARS_MAX[1]
#define MIN_V_CAPBANK                           ANALOG_VARS_MIN[1]

#define MAX_DCCTS_DIFF                          ANALOG_VARS_MAX[2]

#define MAX_I_IDLE_DCCT                         ANALOG_VARS_MAX[3]
#define MIN_I_ACTIVE_DCCT                       ANALOG_VARS_MIN[3]
#define NUM_DCCTs                               ANALOG_VARS_MAX[4]

#define DELAY_TIME_INTERLOCK_IDB_US             ANALOG_VARS_MAX[5]

#define D_DUTY_MAX_POS                          ANALOG_VARS_MAX[6]
#define D_DUTY_MAX_NEG                          ANALOG_VARS_MAX[7]

#define MAX_I_ARM                               ANALOG_VARS_MAX[8]
#define MAX_I_ARMS_DIFF                         ANALOG_VARS_MAX[9]
#define I_ARMS_DIFF_MODE                        ANALOG_VARS_MAX[10]

#define ENABLE_COMPLEMENTARY_PS_INTERLOCK       ANALOG_VARS_MAX[11]

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                        g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                        g_controller_ctom.net_signals[1].f  // HRADC1
#define I_ARM_1                         g_controller_ctom.net_signals[2].f  // HRADC2
#define I_ARM_2                         g_controller_ctom.net_signals[3].f  // HRADC3

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[4].f
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[5].f
#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[6].f

#define I_ARMS_DIFF                     g_controller_ctom.net_signals[7].f
#define DUTY_DIFF                       g_controller_ctom.net_signals[8].f

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[9].f

#define DUTY_REF_FF                     g_controller_ctom.net_signals[10].f

#define V_CAPBANK_ARM_1_FILTERED        g_controller_ctom.net_signals[11].f
#define V_CAPBANK_ARM_2_FILTERED        g_controller_ctom.net_signals[12].f

#define IN_FF_V_CAPBANK_ARM_1           g_controller_ctom.net_signals[13].f
#define IN_FF_V_CAPBANK_ARM_2           g_controller_ctom.net_signals[14].f

#define WFMREF_IDX                      g_controller_ctom.net_signals[31].f

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1].f
#define DUTY_CYCLE_MOD_3                g_controller_ctom.output_signals[2].f
#define DUTY_CYCLE_MOD_4                g_controller_ctom.output_signals[3].f
#define DUTY_CYCLE_MOD_5                g_controller_ctom.output_signals[4].f
#define DUTY_CYCLE_MOD_6                g_controller_ctom.output_signals[5].f
#define DUTY_CYCLE_MOD_7                g_controller_ctom.output_signals[6].f
#define DUTY_CYCLE_MOD_8                g_controller_ctom.output_signals[7].f

/// ARM Net Signals
#define V_CAPBANK_MOD_1                 g_controller_mtoc.net_signals[0].f
#define V_CAPBANK_MOD_2                 g_controller_mtoc.net_signals[1].f
#define V_CAPBANK_MOD_3                 g_controller_mtoc.net_signals[2].f
#define V_CAPBANK_MOD_4                 g_controller_mtoc.net_signals[3].f
#define V_CAPBANK_MOD_5                 g_controller_mtoc.net_signals[4].f
#define V_CAPBANK_MOD_6                 g_controller_mtoc.net_signals[5].f
#define V_CAPBANK_MOD_7                 g_controller_mtoc.net_signals[6].f
#define V_CAPBANK_MOD_8                 g_controller_mtoc.net_signals[7].f

/// Reference
#define I_LOAD_SETPOINT                 g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE                g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE          &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define WFMREF                          g_ipc_ctom.wfmref[0]

#define SIGGEN                          SIGGEN_CTOM[0]
#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

/// Load current controller
#define ERROR_I_LOAD                    &g_controller_ctom.dsp_modules.dsp_error[0]

#define PI_CONTROLLER_I_LOAD            &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS     g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.ki

#define IIR_2P2Z_REFERENCE_FEEDFORWARD          &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS   g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

/// Arms current share controller
#define ERROR_I_SHARE                   &g_controller_ctom.dsp_modules.dsp_error[1]

#define PI_CONTROLLER_I_SHARE           &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_I_SHARE_COEFFS    g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.kp
#define KI_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.ki

/// Cap-bank voltage feedforward controllers
#define IIR_2P2Z_LPF_V_CAPBANK_ARM_1            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[1].coeffs.s

#define IIR_2P2Z_LPF_V_CAPBANK_ARM_2            &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS     g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s

#define FF_V_CAPBANK_ARM_1              &g_controller_ctom.dsp_modules.dsp_ff[0]
#define FF_V_CAPBANK_ARM_1_COEFFS       g_controller_mtoc.dsp_modules.dsp_ff[0].coeffs.s

#define FF_V_CAPBANK_ARM_2              &g_controller_ctom.dsp_modules.dsp_ff[1]
#define FF_V_CAPBANK_ARM_2_COEFFS       g_controller_mtoc.dsp_modules.dsp_ff[1].coeffs.s

/// PWM Modulators
#define PWM_MODULATOR_Q1_MOD_1_5        g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_Q2_MOD_1_5        g_pwm_modules.pwm_regs[1]
#define PWM_MODULATOR_Q1_MOD_2_6        g_pwm_modules.pwm_regs[2]
#define PWM_MODULATOR_Q2_MOD_2_6        g_pwm_modules.pwm_regs[3]
#define PWM_MODULATOR_Q1_MOD_3_7        g_pwm_modules.pwm_regs[4]
#define PWM_MODULATOR_Q2_MOD_3_7        g_pwm_modules.pwm_regs[5]
#define PWM_MODULATOR_Q1_MOD_4_8        g_pwm_modules.pwm_regs[6]
#define PWM_MODULATOR_Q2_MOD_4_8        g_pwm_modules.pwm_regs[7]

/// Scope
#define SCOPE                           SCOPE_CTOM[0]

/**
 * Digital I/O's status
 */
#define PIN_BYPASS_IDB_INTERLOCKS       SET_GPDO1;
#define PIN_ACTIVE_IDB_INTERLOCKS       CLEAR_GPDO1;

#define PIN_SET_IDB_INTERLOCK           CLEAR_GPDO2;
#define PIN_CLEAR_IDB_INTERLOCK         SET_GPDO2;

#define PIN_SET_UDC_INTERLOCK               CLEAR_EPWMSYNCO;
#define PIN_CLEAR_UDC_INTERLOCK             SET_EPWMSYNCO;

#define PIN_STATUS_COMPLEMENTARY_PS_INTERLOCK   (!GET_INT_ARM && ENABLE_COMPLEMENTARY_PS_INTERLOCK)

#define PIN_STATUS_DCCT_1_STATUS        GET_GPDI9
#define PIN_STATUS_DCCT_1_ACTIVE        GET_GPDI10
#define PIN_STATUS_DCCT_2_STATUS        GET_GPDI13
#define PIN_STATUS_DCCT_2_ACTIVE        GET_GPDI14

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_3_CapBank_Overvoltage,
    Module_4_CapBank_Overvoltage,
    Module_5_CapBank_Overvoltage,
    Module_6_CapBank_Overvoltage,
    Module_7_CapBank_Overvoltage,
    Module_8_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    Module_3_CapBank_Undervoltage,
    Module_4_CapBank_Undervoltage,
    Module_5_CapBank_Undervoltage,
    Module_6_CapBank_Undervoltage,
    Module_7_CapBank_Undervoltage,
    Module_8_CapBank_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
    IIB_Mod_3_Itlk,
    IIB_Mod_4_Itlk,
    IIB_Mod_5_Itlk,
    IIB_Mod_6_Itlk,
    IIB_Mod_7_Itlk,
    IIB_Mod_8_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    ARM_1_Overcurrent,
    ARM_2_Overcurrent,
    Arms_High_Difference,
    Complementary_PS_Itlk
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS     IIB_Mod_8_Itlk + 1
#define NUM_SOFT_INTERLOCKS     Complementary_PS_Itlk + 1

/**
 *  Private variables
 */
static uint16_t decimation_factor;
static float decimation_coeff;

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t dummy);
static void turn_off(uint16_t dummy);

static void reset_interlocks(uint16_t dummy);
static inline void check_interlocks(void);
static inline void check_capbank_undervoltage(void);
static inline void check_capbank_overvoltage(void);

static void cfg_pwm_module_h_brigde_q2(volatile struct EPWM_REGS *p_pwm_module);
static void set_pwm_duty_hbridge_chB(volatile struct EPWM_REGS *p_pwm_module, float duty_pu);
static float compensate_pwm_deadtime(float duty, float i_load);

/**
 * Main function for this power supply module
 */
void main_fac_2p4s_dcdc(void)
{
    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;


    /// Initial condition for set of boards to remove them from looped interlock
    PIN_CLEAR_IDB_INTERLOCK;
    DELAY_US(DELAY_TIME_INTERLOCK_IDB_US);
    PIN_BYPASS_IDB_INTERLOCKS;
    DELAY_US(DELAY_TIME_INTERLOCK_IDB_US);
    PIN_ACTIVE_IDB_INTERLOCKS;


    /// TODO: include condition for re-initialization
    while(1)
    {
        check_interlocks();
    }

    turn_off(0);

    disable_controller();
    term_interruptions();
    reset_controller();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    decimation_factor = (uint16_t) roundf(HRADC_FREQ_SAMP / ISR_CONTROL_FREQ);
    decimation_coeff = 1.0 / (float) decimation_factor;


    HRADCs_Info.enable_Sampling = 0;
    HRADCs_Info.n_HRADC_boards = NUM_HRADC_BOARDS;

    Init_DMA_McBSP_nBuffers(NUM_HRADC_BOARDS, decimation_factor, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_HRADC_BOARDS; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, decimation_factor,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /**
     *
     * Initialization of PWM modules. PWM signals are mapped as the following:
     *
     *      ePWM  =>  Signal    POF transmitter
     *     channel     Name        on BCB
     *
     *     ePWM1A => Q1_MOD_1       PWM1
     *     ePWM1B => Q1_MOD_5       PWM2
     *     ePWM2A => Q2_MOD_1       PWM3
     *     ePWM2B => Q2_MOD_5       PWM4
     *     ePWM3A => Q1_MOD_2       PWM5
     *     ePWM3B => Q1_MOD_6       PWM6
     *     ePWM4A => Q2_MOD_2       PWM7
     *     ePWM4B => Q2_MOD_6       PWM8
     *     ePWM5A => Q1_MOD_3       PWM9
     *     ePWM5B => Q1_MOD_7       PWM10
     *     ePWM6A => Q2_MOD_3       PWM11
     *     ePWM6B => Q2_MOD_7       PWM12
     *     ePWM7A => Q1_MOD_4       PWM13
     *     ePWM7B => Q1_MOD_8       PWM14
     *     ePWM8A => Q2_MOD_4       PWM15
     *     ePWM8B => Q2_MOD_8       PWM16
     *
     */

    g_pwm_modules.num_modules = 8;

    PWM_MODULATOR_Q1_MOD_1_5 = &EPwm1Regs;
    PWM_MODULATOR_Q2_MOD_1_5 = &EPwm2Regs;
    PWM_MODULATOR_Q1_MOD_2_6 = &EPwm3Regs;
    PWM_MODULATOR_Q2_MOD_2_6 = &EPwm4Regs;
    PWM_MODULATOR_Q1_MOD_3_7 = &EPwm5Regs;
    PWM_MODULATOR_Q2_MOD_3_7 = &EPwm6Regs;
    PWM_MODULATOR_Q1_MOD_4_8 = &EPwm7Regs;
    PWM_MODULATOR_Q2_MOD_4_8 = &EPwm8Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    init_pwm_module(PWM_MODULATOR_Q1_MOD_1_5, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_Q2_MOD_1_5, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    cfg_pwm_module_h_brigde_q2(PWM_MODULATOR_Q2_MOD_1_5);

    init_pwm_module(PWM_MODULATOR_Q1_MOD_2_6, PWM_FREQ, 0, PWM_Sync_Slave, 45,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_Q2_MOD_2_6, PWM_FREQ, 3, PWM_Sync_Slave, 225,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    cfg_pwm_module_h_brigde_q2(PWM_MODULATOR_Q2_MOD_2_6);

    init_pwm_module(PWM_MODULATOR_Q1_MOD_3_7, PWM_FREQ, 0, PWM_Sync_Slave, 90,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_Q2_MOD_3_7, PWM_FREQ, 5, PWM_Sync_Slave, 270,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    cfg_pwm_module_h_brigde_q2(PWM_MODULATOR_Q2_MOD_3_7);

    init_pwm_module(PWM_MODULATOR_Q1_MOD_4_8, PWM_FREQ, 0, PWM_Sync_Slave, 135,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_Q2_MOD_4_8, PWM_FREQ, 7, PWM_Sync_Slave, 315,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    cfg_pwm_module_h_brigde_q2(PWM_MODULATOR_Q2_MOD_4_8);

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();

    /// Initialization of timers
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;

    /// Configure EPWMSYNCO as GPDO for complementary PS interlock
    PIN_CLEAR_UDC_INTERLOCK;
    cfg_epwmsynco_gpdo();
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    g_ipc_ctom.ps_module[1].ps_status.all = 0;
    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_event_manager(0, ISR_CONTROL_FREQ,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_control_framework(&g_controller_ctom);

    init_ipc();

    init_wfmref(&WFMREF, WFMREF_SELECTED_PARAM[0], WFMREF_SYNC_MODE_PARAM[0],
                ISR_CONTROL_FREQ, WFMREF_FREQUENCY_PARAM[0], WFMREF_GAIN_PARAM[0],
                WFMREF_OFFSET_PARAM[0], &g_wfmref_data.data, SIZE_WFMREF,
                &I_LOAD_REFERENCE);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ, &I_LOAD_REFERENCE);

    cfg_siggen(&SIGGEN, SIGGEN_TYPE_PARAM, SIGGEN_NUM_CYCLES_PARAM,
               SIGGEN_FREQ_PARAM, SIGGEN_AMP_PARAM,
               SIGGEN_OFFSET_PARAM, SIGGEN_AUX_PARAM);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].amplitude
     *         out:     SIGGEN_CTOM[0].amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SLEWRATE_SIGGEN_AMP, ISR_CONTROL_FREQ,
                   &SIGGEN_MTOC[0].amplitude, &SIGGEN.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].offset
     *         out:     SIGGEN_CTOM[0].offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SLEWRATE_SIGGEN_OFFSET,
                   ISR_CONTROL_FREQ, &SIGGEN_MTOC[0].offset,
                   &SIGGEN_CTOM[0].offset);

    /*************************************************/
    /** INITIALIZATION OF LOAD CURRENT CONTROL LOOP **/
    /*************************************************/

    /**
     *        name:     SRLIM_I_LOAD_REFERENCE
     * description:     Load current slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     I_LOAD_SETPOINT
     *         out:     I_LOAD_REFERENCE
     */

    init_dsp_srlim(SRLIM_I_LOAD_REFERENCE, MAX_SLEWRATE_SLOWREF, ISR_CONTROL_FREQ,
                   &I_LOAD_SETPOINT, &I_LOAD_REFERENCE);

    /**
     *        name:     ERROR_I_LOAD
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     I_LOAD_REFERENCE
     *           -:     I_LOAD_MEAN
     *         out:     I_LOAD_ERROR
     */

    init_dsp_error(ERROR_I_LOAD, &I_LOAD_REFERENCE, &I_LOAD_MEAN, &I_LOAD_ERROR);

    /**
     *        name:     PI_CONTROLLER_I_LOAD
     * description:     Capacitor bank voltage PI controller
     *  dsp module:     DSP_PI
     *          in:     I_LOAD_ERROR
     *         out:     DUTY_I_LOAD_PI
     */

    init_dsp_pi(PI_CONTROLLER_I_LOAD, KP_I_LOAD, KI_I_LOAD, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_ERROR, &DUTY_I_LOAD_PI);

    /**
     *        name:     IIR_2P2Z_REFERENCE_FEEDFORWARD
     * description:     Load current IIR 2P2Z controller
     *  dsp module:     DSP_IIR_2P2Z
     *          in:     I_LOAD_REFERENCE
     *         out:     DUTY_FF
     */

    init_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b0,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b1,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.b2,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.a1,
                      IIR_2P2Z_REFERENCE_FEEDFORWARD_COEFFS.a2,
                      PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_REFERENCE,
                      &DUTY_REF_FF);

    /*******************************************************/
    /** INITIALIZATION OF ARMS CURRENT SHARE CONTROL LOOP **/
    /*******************************************************/

    /**
     *        name:     PI_CONTROLLER_I_SHARE
     * description:     Arms current share PI controller
     *  dsp module:     DSP_PI
     *          in:     I_ARMS_DIFF
     *         out:     DUTY_DIFF
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE, KP_I_SHARE, KI_I_SHARE, ISR_CONTROL_FREQ,
                PWM_LIM_DUTY_SHARE, -PWM_LIM_DUTY_SHARE, &I_ARMS_DIFF, &DUTY_DIFF);

    /**********************************************************/
    /** INITIALIZATION OF CAPACITOR BANK VOLTAGE FEEDFORWARD **/
    /**********************************************************/

    /**
     *        name:     IIR_2P2Z_LPF_V_CAPBANK_ARM_1
     * description:     Module 1 capacitor bank voltage low-pass filter
     *    DP class:     ELP_IIR_2P2Z
     *          in:     V_CAPBANK_MOD_4
     *         out:     V_CAPBANK_ARM_1_FILTERED
     */

    init_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_1,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS.b0,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS.b1,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS.b2,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS.a1,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_1_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_MOD_4, &V_CAPBANK_ARM_1_FILTERED);

    /**
     *        name:     FF_V_CAPBANK_ARM_1
     * description:     Module 1 capacitor bank voltage feed-forward
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     V_CAPBANK_ARM_1_FILTERED
     *          in:     IN_FF_V_CAPBANK_ARM_1
     *         out:     DUTY_CYCLE_MOD_1
     */

    init_dsp_vdclink_ff(FF_V_CAPBANK_ARM_1, FF_V_CAPBANK_ARM_1_COEFFS.vdc_nom,
                        FF_V_CAPBANK_ARM_1_COEFFS.vdc_min,
                        &V_CAPBANK_ARM_1_FILTERED, &IN_FF_V_CAPBANK_ARM_1,
                        &DUTY_CYCLE_MOD_1);

    /**
     *        name:     IIR_2P2Z_LPF_V_CAPBANK_ARM_2
     * description:     Module 2 capacitor bank voltage low-pass filter
     *    DP class:     ELP_IIR_2P2Z
     *          in:     V_CAPBANK_MOD_5
     *         out:     V_CAPBANK_ARM_2_FILTERED
     */

    init_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_2,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS.b0,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS.b1,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS.b2,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS.a1,
                      IIR_2P2Z_LPF_V_CAPBANK_ARM_2_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_MOD_5, &V_CAPBANK_ARM_2_FILTERED);

    /**
     *        name:     FF_V_CAPBANK_ARM_2
     * description:     Module 2 capacitor bank voltage feed-forward
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     V_CAPBANK_ARM_2_FILTERED
     *          in:     IN_FF_V_CAPBANK_ARM_2
     *         out:     DUTY_CYCLE_MOD_5
     */

    init_dsp_vdclink_ff(FF_V_CAPBANK_ARM_2, FF_V_CAPBANK_ARM_2_COEFFS.vdc_nom,
                        FF_V_CAPBANK_ARM_2_COEFFS.vdc_min,
                        &V_CAPBANK_ARM_2_FILTERED, &IN_FF_V_CAPBANK_ARM_2,
                        &DUTY_CYCLE_MOD_5);

    /******************************/
    /** INITIALIZATION OF SCOPES **/
    /******************************/

    init_scope(&SCOPE, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[0],
               &g_buf_samples_ctom[0], SIZE_BUF_SAMPLES_CTOM,
               SCOPE_SOURCE_PARAM[0], &run_scope_shared_ram);

    /**
     * Reset all internal variables
     */
    reset_controller();
}

/**
 * Reset all internal variables from controller
 */
static void reset_controller(void)
{
    set_pwm_duty_chA(PWM_MODULATOR_Q1_MOD_1_5, 50.0);
    set_pwm_duty_chB(PWM_MODULATOR_Q1_MOD_1_5, 50.0);

    set_pwm_duty_chA(PWM_MODULATOR_Q1_MOD_2_6, 50.0);
    set_pwm_duty_chB(PWM_MODULATOR_Q1_MOD_2_6, 50.0);

    set_pwm_duty_chA(PWM_MODULATOR_Q1_MOD_3_7, 50.0);
    set_pwm_duty_chB(PWM_MODULATOR_Q1_MOD_3_7, 50.0);

    set_pwm_duty_chA(PWM_MODULATOR_Q1_MOD_4_8, 50.0);
    set_pwm_duty_chB(PWM_MODULATOR_Q1_MOD_4_8, 50.0);

    g_ipc_ctom.ps_module[0].ps_status.bit.openloop = LOOP_STATE;

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);
    reset_dsp_error(ERROR_I_LOAD);
    reset_dsp_pi(PI_CONTROLLER_I_LOAD);

    reset_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD);

    reset_dsp_pi(PI_CONTROLLER_I_SHARE);

    reset_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_1);
    reset_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_2);

    reset_dsp_vdclink_ff(FF_V_CAPBANK_ARM_1);
    reset_dsp_vdclink_ff(FF_V_CAPBANK_ARM_2);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);

    reset_wfmref(&WFMREF);
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    PieVectTable.EPWM2_INT =  &isr_controller;
    PieVectTable.EPWM5_INT =  &isr_controller;
    PieVectTable.EPWM6_INT =  &isr_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 1;

    enable_pwm_interrupt(PWM_MODULATOR_Q1_MOD_1_5);
    enable_pwm_interrupt(PWM_MODULATOR_Q2_MOD_1_5);
    enable_pwm_interrupt(PWM_MODULATOR_Q1_MOD_3_7);
    enable_pwm_interrupt(PWM_MODULATOR_Q2_MOD_3_7);

    IER |= M_INT1;
    IER |= M_INT3;
    IER |= M_INT11;

    /// Enable global interrupts (EINT)
    EINT;
    ERTM;
}

/**
 * Termination of interruptions.
 */
static void term_interruptions(void)
{
    /// Disable global interrupts (EINT)
    DINT;
    DRTM;

    /// Clear enables
    IER = 0;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;  /// ePWM1
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;  /// ePWM2
    PieCtrlRegs.PIEIER3.bit.INTx5 = 0;  /// ePWM5
    PieCtrlRegs.PIEIER3.bit.INTx6 = 0;  /// ePWM6

    disable_pwm_interrupt(PWM_MODULATOR_Q1_MOD_1_5);
    disable_pwm_interrupt(PWM_MODULATOR_Q2_MOD_1_5);
    disable_pwm_interrupt(PWM_MODULATOR_Q1_MOD_3_7);
    disable_pwm_interrupt(PWM_MODULATOR_Q2_MOD_3_7);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    PWM_MODULATOR_Q1_MOD_1_5->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q1_MOD_1_5->ETCLR.bit.INT = 1;

    PWM_MODULATOR_Q2_MOD_1_5->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q2_MOD_1_5->ETCLR.bit.INT = 1;

    PWM_MODULATOR_Q1_MOD_3_7->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q1_MOD_3_7->ETCLR.bit.INT = 1;

    PWM_MODULATOR_Q2_MOD_3_7->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_Q2_MOD_3_7->ETCLR.bit.INT = 1;

    /**
     *  Enable XINT2 (external interrupt 2) interrupt used for sync pulses for
     *  the first time
     *
     *  TODO: include here mechanism described in section 1.5.4.3 from F28M36
     *  Technical Reference Manual (SPRUHE8E) to clear flag before enabling, to
     *  avoid false alarms that may occur when sync pulses are received during
     *  firmware initialization.
     */
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    /// Clear interrupt flag for PWM interrupts group
    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

    //CLEAR_DEBUG_GPIO1;
    //SET_DEBUG_GPIO0;
    SET_DEBUG_GPIO1;

    temp[0] = 0.0;
    temp[1] = 0.0;
    temp[2] = 0.0;
    temp[3] = 0.0;

    /// Get HRADC samples
    for(i = 0; i < decimation_factor; i++)
    {
        temp[0] += (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer++);
        temp[1] += (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer++);
        temp[2] += (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer++);
        temp[3] += (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer++);
    }

    //CLEAR_DEBUG_GPIO1;

    HRADCs_Info.HRADC_boards[0].SamplesBuffer = buffers_HRADC[0];
    HRADCs_Info.HRADC_boards[1].SamplesBuffer = buffers_HRADC[1];
    HRADCs_Info.HRADC_boards[2].SamplesBuffer = buffers_HRADC[2];
    HRADCs_Info.HRADC_boards[3].SamplesBuffer = buffers_HRADC[3];

    temp[0] *= HRADCs_Info.HRADC_boards[0].gain * decimation_coeff;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain * decimation_coeff;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain * decimation_coeff;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain * decimation_coeff;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    if(NUM_DCCTs)
    {
        I_LOAD_1 = temp[0];
        I_LOAD_2 = temp[1];
        I_ARM_1 = temp[2];
        I_ARM_2 = temp[3];

        I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
        I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;
    }
    else
    {
        I_LOAD_1 = temp[0];
        I_ARM_1 = temp[1];
        I_ARM_2 = temp[2];

        I_LOAD_MEAN = I_LOAD_1;
        I_LOAD_DIFF = 0;
    }

    run_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_1);
    run_dsp_iir_2p2z(IIR_2P2Z_LPF_V_CAPBANK_ARM_2);

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        /// Calculate reference according to operation mode
        switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
        {
            case SlowRef:
            case SlowRefSync:
            {
                run_dsp_srlim(SRLIM_I_LOAD_REFERENCE, USE_MODULE);
                break;
            }
            case Cycle:
            {
                run_dsp_srlim(SRLIM_SIGGEN_AMP, USE_MODULE);
                run_dsp_srlim(SRLIM_SIGGEN_OFFSET, USE_MODULE);
                SIGGEN.p_run_siggen(&SIGGEN);
                break;
            }
            case RmpWfm:
            case MigWfm:
            {
                run_wfmref(&WFMREF);
                break;
            }
            default:
            {
                break;
            }
        }

        /// Open-loop
        if(g_ipc_ctom.ps_module[0].ps_status.bit.openloop)
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0]);
            DUTY_CYCLE_MOD_1 = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE_MOD_1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);

            DUTY_CYCLE_MOD_5 = DUTY_CYCLE_MOD_1;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF[0], MIN_REF[0]);

            /// Load current controller
            run_dsp_error(ERROR_I_LOAD);
            run_dsp_pi(PI_CONTROLLER_I_LOAD);
            run_dsp_iir_2p2z(IIR_2P2Z_REFERENCE_FEEDFORWARD);

            /// Arms current share controller
            if(I_ARMS_DIFF_MODE)
            {
                I_ARMS_DIFF = I_ARM_1 - 0.5*I_LOAD_MEAN;
            }
            else
            {
                I_ARMS_DIFF = I_ARM_1 - I_ARM_2;
            }
            run_dsp_pi(PI_CONTROLLER_I_SHARE);

            /// Cap-bank voltage feedforward controller
            IN_FF_V_CAPBANK_ARM_1 = DUTY_I_LOAD_PI + DUTY_REF_FF - DUTY_DIFF;
            IN_FF_V_CAPBANK_ARM_2 = DUTY_I_LOAD_PI + DUTY_REF_FF + DUTY_DIFF;

            run_dsp_vdclink_ff(FF_V_CAPBANK_ARM_1);
            run_dsp_vdclink_ff(FF_V_CAPBANK_ARM_2);

            DUTY_CYCLE_MOD_1 = compensate_pwm_deadtime(DUTY_CYCLE_MOD_1, I_LOAD_MEAN);
            DUTY_CYCLE_MOD_5 = compensate_pwm_deadtime(DUTY_CYCLE_MOD_5, I_LOAD_MEAN);

            SATURATE(DUTY_CYCLE_MOD_1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_MOD_5, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        DUTY_CYCLE_MOD_2 = DUTY_CYCLE_MOD_1;
        DUTY_CYCLE_MOD_3 = DUTY_CYCLE_MOD_1;
        DUTY_CYCLE_MOD_4 = DUTY_CYCLE_MOD_1;

        DUTY_CYCLE_MOD_6 = DUTY_CYCLE_MOD_5;
        DUTY_CYCLE_MOD_7 = DUTY_CYCLE_MOD_5;
        DUTY_CYCLE_MOD_8 = DUTY_CYCLE_MOD_5;

        set_pwm_duty_hbridge(PWM_MODULATOR_Q1_MOD_1_5, DUTY_CYCLE_MOD_1);
        set_pwm_duty_hbridge(PWM_MODULATOR_Q1_MOD_2_6, DUTY_CYCLE_MOD_2);
        set_pwm_duty_hbridge(PWM_MODULATOR_Q1_MOD_3_7, DUTY_CYCLE_MOD_3);
        set_pwm_duty_hbridge(PWM_MODULATOR_Q1_MOD_4_8, DUTY_CYCLE_MOD_4);

        set_pwm_duty_hbridge_chB(PWM_MODULATOR_Q1_MOD_1_5, DUTY_CYCLE_MOD_5);
        set_pwm_duty_hbridge_chB(PWM_MODULATOR_Q1_MOD_2_6, DUTY_CYCLE_MOD_6);
        set_pwm_duty_hbridge_chB(PWM_MODULATOR_Q1_MOD_3_7, DUTY_CYCLE_MOD_7);
        set_pwm_duty_hbridge_chB(PWM_MODULATOR_Q1_MOD_4_8, DUTY_CYCLE_MOD_8);
    }

    WFMREF_IDX = (float) (WFMREF.wfmref_data[WFMREF.wfmref_selected].p_buf_idx -
                          WFMREF.wfmref_data[WFMREF.wfmref_selected].p_buf_start);

    RUN_SCOPE(SCOPE);
    //CLEAR_DEBUG_GPIO1;

    SET_INTERLOCKS_TIMEBASE_FLAG(0);

    /**
     * Re-enable external interrupt 2 (XINT2) interrupts to allow sync pulses to
     * be handled once per isr_controller
     */
    if(PieCtrlRegs.PIEIER1.bit.INTx5 == 0)
    {
        /// Set alarm if counter is below limit when receiving new sync pulse
        if(counter_sync_period < MIN_NUM_ISR_CONTROLLER_SYNC)
        {
            g_ipc_ctom.ps_module[0].ps_alarms = High_Sync_Input_Frequency;
        }

        /// Store counter value on BSMP variable
        g_ipc_ctom.period_sync_pulse = counter_sync_period;
        counter_sync_period = 0;
    }

    counter_sync_period++;

    /**
     * Reset counter to threshold to avoid false alarms during its overflow
     */
    if(counter_sync_period == MAX_NUM_ISR_CONTROLLER_SYNC)
    {
        counter_sync_period = MIN_NUM_ISR_CONTROLLER_SYNC;
    }

    /// Re-enable XINT2 (external interrupt 2) interrupt used for sync pulses
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;

    /// Clear interrupt flags for PWM interrupts
    PWM_MODULATOR_Q1_MOD_1_5->ETCLR.bit.INT = 1;
    PWM_MODULATOR_Q2_MOD_1_5->ETCLR.bit.INT = 1;
    PWM_MODULATOR_Q1_MOD_3_7->ETCLR.bit.INT = 1;
    PWM_MODULATOR_Q2_MOD_3_7->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;

    //CLEAR_DEBUG_GPIO0;
    CLEAR_DEBUG_GPIO1;
}

/**
 * Enable control ISR
 */
static void enable_controller()
{
    stop_DMA();
    DELAY_US(5);
    start_DMA();
    HRADCs_Info.enable_Sampling = 1;
    enable_pwm_tbclk();
}

/**
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();

    reset_controller();
}

/**
 * Turn power supply on.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_on(uint16_t dummy)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Initializing)
    #endif
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Initializing;

        if(PIN_STATUS_COMPLEMENTARY_PS_INTERLOCK)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Complementary_PS_Itlk);
            set_soft_interlock(0, Complementary_PS_Itlk);
        }

        #ifdef USE_ITLK
        else
        {
        #endif

            check_capbank_undervoltage();

            #ifdef USE_ITLK
            if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Initializing)
            {
            #endif

                g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;

                enable_pwm_output(0);
                enable_pwm_output(1);
                enable_pwm_output(2);
                enable_pwm_output(3);
                enable_pwm_output(4);
                enable_pwm_output(5);
                enable_pwm_output(6);
                enable_pwm_output(7);

            #ifdef USE_ITLK
            }
        }
        #endif
    }
}

/**
 * Turn off specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_off(uint16_t dummy)
{
    disable_pwm_output(0);
    disable_pwm_output(1);
    disable_pwm_output(2);
    disable_pwm_output(3);
    disable_pwm_output(4);
    disable_pwm_output(5);
    disable_pwm_output(6);
    disable_pwm_output(7);

    reset_controller();

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void reset_interlocks(uint16_t dummy)
{
    g_ipc_ctom.ps_module[0].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_soft_interlock = 0;
    g_ipc_ctom.ps_module[0].ps_alarms = 0;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;

        PIN_CLEAR_IDB_INTERLOCK;
        DELAY_US(DELAY_TIME_INTERLOCK_IDB_US);
        PIN_BYPASS_IDB_INTERLOCKS;
        DELAY_US(DELAY_TIME_INTERLOCK_IDB_US);
        PIN_ACTIVE_IDB_INTERLOCKS;

        PIN_CLEAR_UDC_INTERLOCK;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(I_LOAD_MEAN) > MAX_ILOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(0, DCCT_High_Difference);
    }

    if(fabs(I_ARM_1) > MAX_I_ARM)
    {
        set_soft_interlock(0, ARM_1_Overcurrent);
    }

    if(fabs(I_ARM_2) > MAX_I_ARM)
    {
        set_soft_interlock(0, ARM_2_Overcurrent);
    }

    if(fabs(I_ARMS_DIFF) > MAX_I_ARMS_DIFF)
    {
        set_soft_interlock(0, Arms_High_Difference);
    }

    if(!PIN_STATUS_DCCT_1_STATUS)
    {
        set_soft_interlock(0, DCCT_1_Fault);
    }

    if( NUM_DCCTs && !PIN_STATUS_DCCT_2_STATUS )
    {
        set_soft_interlock(0, DCCT_2_Fault);
    }

    if(PIN_STATUS_DCCT_1_ACTIVE)
    {
        if(fabs(I_LOAD_1) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }
    else
    {
        if(fabs(I_LOAD_1) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(0, Load_Feedback_1_Fault);
        }
    }

    if(NUM_DCCTs)
    {
        if(PIN_STATUS_DCCT_2_ACTIVE)
        {
            if(fabs(I_LOAD_2) < MIN_I_ACTIVE_DCCT)
            {
                set_soft_interlock(0, Load_Feedback_2_Fault);
            }
        }
        else
        {
            if(fabs(I_LOAD_2) > MAX_I_IDLE_DCCT)
            {
                set_soft_interlock(0, Load_Feedback_2_Fault);
            }
        }
    }

    check_capbank_overvoltage();

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
    {
        check_capbank_undervoltage();

        if(PIN_STATUS_COMPLEMENTARY_PS_INTERLOCK)
        {
            set_soft_interlock(0, Complementary_PS_Itlk);
        }
    }

    EINT;

    //SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    //CLEAR_DEBUG_GPIO1;

    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Interlock)
    #else
    if(g_ipc_ctom.ps_module[0].ps_hard_interlock || g_ipc_ctom.ps_module[0].ps_soft_interlock)
    #endif
    {
        PIN_SET_IDB_INTERLOCK;

        if(GET_EPWMSYNCO)
        {
            PIN_SET_UDC_INTERLOCK;
        }
        else
        {
            PIN_CLEAR_UDC_INTERLOCK;
        }
    }
}

static inline void check_capbank_undervoltage(void)
{
    if(V_CAPBANK_MOD_1 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_1_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_2 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_2_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_3 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_3_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_4 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_4_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_5 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_5_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_6 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_6_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_7 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_7_CapBank_Undervoltage);
    }

    if(V_CAPBANK_MOD_8 < MIN_V_CAPBANK)
    {
        set_hard_interlock(0, Module_8_CapBank_Undervoltage);
    }
}

static inline void check_capbank_overvoltage(void)
{
    if(V_CAPBANK_MOD_1 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_1_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_2 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_2_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_3 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_3_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_4 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_4_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_5 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_5_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_6 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_6_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_7 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_7_CapBank_Overvoltage);
    }

    if(V_CAPBANK_MOD_8 > MAX_V_CAPBANK)
    {
        set_hard_interlock(0, Module_8_CapBank_Overvoltage);
    }
}

/**
 * Configure specified PWM module to generate inverted PWM pulses (active on
 * LOW). This is used to generate 8x Q2 signals for the 8 DC/DC modules.
 *
 * @param p_pwm_module specified PWM module
 */
static void cfg_pwm_module_h_brigde_q2(volatile struct EPWM_REGS *p_pwm_module)
{
    p_pwm_module->AQCTLA.bit.ZRO = AQ_CLEAR;
    p_pwm_module->AQCTLA.bit.PRD = AQ_NO_ACTION;
    p_pwm_module->AQCTLA.bit.CAU = AQ_SET;
    p_pwm_module->AQCTLA.bit.CAD = AQ_NO_ACTION;
    p_pwm_module->AQCTLA.bit.CBU = AQ_NO_ACTION;
    p_pwm_module->AQCTLA.bit.CBD = AQ_NO_ACTION;

    p_pwm_module->AQCTLB.bit.ZRO = AQ_CLEAR;
    p_pwm_module->AQCTLB.bit.PRD = AQ_NO_ACTION;
    p_pwm_module->AQCTLB.bit.CAU = AQ_NO_ACTION;
    p_pwm_module->AQCTLB.bit.CAD = AQ_NO_ACTION;
    p_pwm_module->AQCTLB.bit.CBU = AQ_SET;
    p_pwm_module->AQCTLB.bit.CBD = AQ_NO_ACTION;
}

/**
 * Set duty-cycle (-1.0 to 1.0) for specified PWM module working with a H-bridge
 * with unipolar switching scheme using only channels B for Q1 and Q2,
 * independent from channels A.
 *
 * @param p_pwm_module specified PWM module
 * @param duty specified duty cycle [p.u.]
 */
static void set_pwm_duty_hbridge_chB(volatile struct EPWM_REGS *p_pwm_module, float duty_pu)
{
    uint16_t duty_int;
    uint16_t duty_frac;
    float duty;

    duty = (0.5 * duty_pu + 0.5) * (float)p_pwm_module->TBPRD;

    duty_int  = (uint16_t) duty;
    duty_frac = ((uint16_t) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    p_pwm_module->CMPBM.half.CMPB    = duty_int;
    p_pwm_module->CMPBM.half.CMPBHR  = duty_frac;
}

static float compensate_pwm_deadtime(float duty, float i_load)
{
    static float duty_offset;

    if(i_load < 0.0)
    {
        duty_offset = D_DUTY_MAX_NEG;
    }
    else
    {
        duty_offset = D_DUTY_MAX_POS;
    }

    return duty + duty_offset;
}

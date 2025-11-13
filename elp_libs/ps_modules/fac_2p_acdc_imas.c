/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_2p_acdc_imas.h
 * @brief FAC-2P AC/DC Stage module for IMAS
 * 
 * Module for control of two AC/DC modules of FAC power supplies used by IMAS
 * group on magnets characterization tests. It implements the individual
 * controllers for capacitor bank voltage of each AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 20/02/2020
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

#include "fac_2p_acdc_imas.h"

/**
 * Control parameters
 */
#define TIMESLICER_CONTROLLER_IDX       2
#define TIMESLICER_CONTROLLER           g_controller_ctom.timeslicer[TIMESLICER_CONTROLLER_IDX]
#define CONTROLLER_FREQ_SAMP            TIMESLICER_FREQ[TIMESLICER_CONTROLLER_IDX]

/**
 * Analog variables parameters
 */
#define MAX_V_CAPBANK                           ANALOG_VARS_MAX[0]

#define MAX_IOUT_RECT                           ANALOG_VARS_MAX[1]

#define MAX_IOUT_RECT_REF                       ANALOG_VARS_MAX[2]
#define MIN_IOUT_RECT_REF                       ANALOG_VARS_MIN[2]

#define TIMEOUT_AC_MAINS_CONTACTOR_CLOSED_MS    ANALOG_VARS_MAX[3]
#define TIMEOUT_AC_MAINS_CONTACTOR_OPENED_MS    ANALOG_VARS_MAX[4]

#define HRADC_R_BURDEN_1                        ANALOG_VARS_MAX[6]
#define HRADC_R_BURDEN_3                        ANALOG_VARS_MAX[7]

/**
 * Controller defines
 */

/// Shared defines
#define NF_ALPHA                        0.99

/// Reference
#define V_CAPBANK_SETPOINT              g_ipc_ctom.ps_module[0].ps_setpoint
#define V_CAPBANK_REFERENCE             g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_V_CAPBANK_REFERENCE       &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define SIGGEN                          SIGGEN_CTOM[0]
#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

/**
 * Defines for AC/DC Module A
 */
#define MOD_A_ID        0x0

/// DSP Signals
#define V_CAPBANK_MOD_A                     g_controller_ctom.net_signals[0].f  // HRADC0
#define IOUT_RECT_MOD_A                     g_controller_ctom.net_signals[1].f  // HRADC1

#define V_CAPBANK_FILTERED_2HZ_MOD_A        g_controller_ctom.net_signals[2].f
#define V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A    g_controller_ctom.net_signals[3].f
#define V_CAPBANK_ERROR_MOD_A               g_controller_ctom.net_signals[4].f

#define IOUT_RECT_REF_MOD_A                 g_controller_ctom.net_signals[5].f
#define IOUT_RECT_ERROR_MOD_A               g_controller_ctom.net_signals[6].f

#define DUTY_CYCLE_MOD_A                    g_controller_ctom.output_signals[0].f

/// Capbank voltage controller for module A
#define ERROR_V_CAPBANK_MOD_A               &g_controller_ctom.dsp_modules.dsp_error[0]

#define PI_CONTROLLER_V_CAPBANK_MOD_A           &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_V_CAPBANK_MOD_A_COEFFS    g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_V_CAPBANK_MOD_A                      PI_CONTROLLER_V_CAPBANK_MOD_A_COEFFS.kp
#define KI_V_CAPBANK_MOD_A                      PI_CONTROLLER_V_CAPBANK_MOD_A_COEFFS.ki

#define NOTCH_FILT_2HZ_V_CAPBANK_MOD_A                  &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS           g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s
#define NOTCH_FILT_4HZ_V_CAPBANK_MOD_A                  &g_controller_ctom.dsp_modules.dsp_iir_2p2z[1]
#define NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS           g_controller_mtoc.dsp_modules.dsp_iir_2p2z[1].coeffs.s

/// Rectifier output current controller for module A
#define ERROR_IOUT_RECT_MOD_A                           &g_controller_ctom.dsp_modules.dsp_error[1]

#define PI_CONTROLLER_IOUT_RECT_MOD_A                   &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_IOUT_RECT_MOD_A_COEFFS            g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_IOUT_RECT_MOD_A                              PI_CONTROLLER_IOUT_RECT_MOD_A_COEFFS.kp
#define KI_IOUT_RECT_MOD_A                              PI_CONTROLLER_IOUT_RECT_MOD_A_COEFFS.ki

/// PWM modulator for module A
#define PWM_MODULATOR_MOD_A         g_pwm_modules.pwm_regs[0]

/// Scope for module A
#define SCOPE_MOD_A                 SCOPE_CTOM[0]

/**
 * Defines for AC/DC Module B
 */
#define MOD_B_ID        0x1

/// DSP Signals
#define V_CAPBANK_MOD_B                     g_controller_ctom.net_signals[7].f  // HRADC2
#define IOUT_RECT_MOD_B                     g_controller_ctom.net_signals[8].f  // HRADC3

#define V_CAPBANK_FILTERED_2HZ_MOD_B        g_controller_ctom.net_signals[9].f
#define V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B    g_controller_ctom.net_signals[10].f
#define V_CAPBANK_ERROR_MOD_B               g_controller_ctom.net_signals[11].f

#define IOUT_RECT_REF_MOD_B                 g_controller_ctom.net_signals[12].f
#define IOUT_RECT_ERROR_MOD_B               g_controller_ctom.net_signals[13].f

#define DUTY_CYCLE_MOD_B                    g_controller_ctom.output_signals[1].f

/// Capbank voltage controller for module B
#define ERROR_V_CAPBANK_MOD_B               &g_controller_ctom.dsp_modules.dsp_error[2]

#define PI_CONTROLLER_V_CAPBANK_MOD_B           &g_controller_ctom.dsp_modules.dsp_pi[2]
#define PI_CONTROLLER_V_CAPBANK_MOD_B_COEFFS    g_controller_mtoc.dsp_modules.dsp_pi[2].coeffs.s
#define KP_V_CAPBANK_MOD_B                      PI_CONTROLLER_V_CAPBANK_MOD_B_COEFFS.kp
#define KI_V_CAPBANK_MOD_B                      PI_CONTROLLER_V_CAPBANK_MOD_B_COEFFS.ki

#define NOTCH_FILT_2HZ_V_CAPBANK_MOD_B                  &g_controller_ctom.dsp_modules.dsp_iir_2p2z[2]
#define NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS           g_controller_mtoc.dsp_modules.dsp_iir_2p2z[2].coeffs.s
#define NOTCH_FILT_4HZ_V_CAPBANK_MOD_B                  &g_controller_ctom.dsp_modules.dsp_iir_2p2z[3]
#define NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS           g_controller_mtoc.dsp_modules.dsp_iir_2p2z[3].coeffs.s

/// Rectifier output current controller for module B
#define ERROR_IOUT_RECT_MOD_B                           &g_controller_ctom.dsp_modules.dsp_error[3]

#define PI_CONTROLLER_IOUT_RECT_MOD_B                   &g_controller_ctom.dsp_modules.dsp_pi[3]
#define PI_CONTROLLER_IOUT_RECT_MOD_B_COEFFS            g_controller_mtoc.dsp_modules.dsp_pi[3].coeffs.s
#define KP_IOUT_RECT_MOD_B                              PI_CONTROLLER_IOUT_RECT_MOD_B_COEFFS.kp
#define KI_IOUT_RECT_MOD_B                              PI_CONTROLLER_IOUT_RECT_MOD_B_COEFFS.ki

/// PWM modulator for module B
#define PWM_MODULATOR_MOD_B         g_pwm_modules.pwm_regs[1]

/// Scope for module B
#define SCOPE_MOD_B                 SCOPE_CTOM[1]

/**
 * Digital I/O's status
 */
#define PIN_STATUS_AC_MAINS_CONTACTOR       GET_GPDI1
#define PIN_STATUS_DCDC_INTERLOCK           !GET_GPDI2

#define PIN_OPEN_AC_MAINS_CONTACTOR         CLEAR_GPDO1
#define PIN_CLOSE_AC_MAINS_CONTACTOR        SET_GPDO1

#define PIN_SET_ACDC_INTERLOCK              CLEAR_GPDO2
#define PIN_CLEAR_ACDC_INTERLOCK            SET_GPDO2

/**
 * Interlocks defines
 */
typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overcurrent,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    Module_A_Interlock,
    Module_B_Interlock,
    DCDC_Interlock
} hard_interlocks_t;

#define NUM_HARD_INTERLOCKS     DCDC_Interlock + 1
#define NUM_SOFT_INTERLOCKS     0

/**
 *  Private variables
 */
static float decimation_factor;
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
static void init_controller_module_A(void);
static void init_controller_module_B(void);
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

/**
 * Main function for this power supply module
 */
void main_fac_2p_acdc_imas(void)
{
    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

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

    /// Clear AC/DC interlock signal
    PIN_CLEAR_ACDC_INTERLOCK;

    /// Initialization of HRADC boards
    stop_DMA();

    decimation_factor = HRADC_FREQ_SAMP / ISR_CONTROL_FREQ;
    decimation_coeff = 1.0 / decimation_factor;


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

    // Manually configure gains for Iin_bipolar input on HRADC v2.0 boards
    #if HRADC_v2_0
        HRADCs_Info.HRADC_boards[1].gain =
                TRANSDUCER_GAIN[1] * (1.0/(HRADC_R_BURDEN_1 * HRADC_BI_OFFSET));
        HRADCs_Info.HRADC_boards[1].offset =
                -(HRADCs_Info.HRADC_boards[1].gain * HRADC_BI_OFFSET);

        HRADCs_Info.HRADC_boards[3].gain =
                TRANSDUCER_GAIN[3] * (1.0/(HRADC_R_BURDEN_3 * HRADC_BI_OFFSET));
        HRADCs_Info.HRADC_boards[3].offset =
                -(HRADCs_Info.HRADC_boards[3].gain * HRADC_BI_OFFSET);
    #endif

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /**
     * Initialization of PWM modules. PWM signals are mapped as the following:
     *
     *      ePWM  =>    Signal     POF transmitter
     *     channel       Name         on BCB
     *
     *     ePWM1A =>    PWM_MOD_A      PWM1
     *     ePWM2A =>    PWM_MOD_B      PWM3
     */
    g_pwm_modules.num_modules = 2;

    PWM_MODULATOR_MOD_A = &EPwm1Regs;
    PWM_MODULATOR_MOD_B = &EPwm2Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PWM initialization
    init_pwm_module(PWM_MODULATOR_MOD_A, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);

    init_pwm_module(PWM_MODULATOR_MOD_B, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);

    InitEPwm1Gpio();
    InitEPwm2Gpio();

    /// Initialization of timers
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ, 1000000);
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    init_ps_module(&g_ipc_ctom.ps_module[0],
                   g_ipc_mtoc.ps_module[0].ps_status.bit.model,
                   &turn_on, &turn_off, &isr_soft_interlock,
                   &isr_hard_interlock, &reset_interlocks);

    init_ps_module(&g_ipc_ctom.ps_module[1],
                       g_ipc_mtoc.ps_module[1].ps_status.bit.model,
                       &turn_on, &turn_off, &isr_soft_interlock,
                       &isr_hard_interlock, &reset_interlocks);

    g_ipc_ctom.ps_module[2].ps_status.all = 0;
    g_ipc_ctom.ps_module[3].ps_status.all = 0;

    init_event_manager(0, ISR_CONTROL_FREQ,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_event_manager(1, ISR_CONTROL_FREQ,
                       NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                       &HARD_INTERLOCKS_DEBOUNCE_TIME,
                       &HARD_INTERLOCKS_RESET_TIME,
                       &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                       &SOFT_INTERLOCKS_RESET_TIME);

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /*************************************/
    /** INITIALIZATION OF DSP FRAMEWORK **/
    /*************************************/

    /**
     *        name:     SRLIM_V_CAPBANK_REFERENCE
     * description:     Capacitor bank voltage reference slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     V_CAPBANK_SETPOINT
     *         out:     V_CAPBANK_REFERENCE
     */

    init_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE, MAX_SLEWRATE_SLOWREF,
                   CONTROLLER_FREQ_SAMP, &V_CAPBANK_SETPOINT,
                   &V_CAPBANK_REFERENCE);

    init_controller_module_A();
    init_controller_module_B();

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, CONTROLLER_FREQ_SAMP, &V_CAPBANK_REFERENCE);

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

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SLEWRATE_SIGGEN_AMP,
                   CONTROLLER_FREQ_SAMP, &SIGGEN_MTOC[0].amplitude,
                   &SIGGEN.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     SIGGEN_MTOC[0].offset
     *         out:     SIGGEN_CTOM[0].offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SLEWRATE_SIGGEN_OFFSET,
                   CONTROLLER_FREQ_SAMP, &SIGGEN_MTOC[0].offset,
                   &SIGGEN_CTOM[0].offset);

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for controller
     */
    init_timeslicer(&TIMESLICER_CONTROLLER, ISR_CONTROL_FREQ);
    cfg_timeslicer(&TIMESLICER_CONTROLLER, CONTROLLER_FREQ_SAMP);

    /******************************/
    /** INITIALIZATION OF SCOPES **/
    /******************************/

    init_scope(&SCOPE_MOD_A, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[0],
               &g_buf_samples_ctom[0], SIZE_BUF_SAMPLES_CTOM/2,
               SCOPE_SOURCE_PARAM[0], &run_scope_shared_ram);

    init_scope(&SCOPE_MOD_B, ISR_CONTROL_FREQ, SCOPE_FREQ_SAMPLING_PARAM[1],
               &g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM/2], SIZE_BUF_SAMPLES_CTOM/2,
               SCOPE_SOURCE_PARAM[1], &run_scope_shared_ram);

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
    set_pwm_duty_chA(PWM_MODULATOR_MOD_A, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_MOD_B, 0.0);

    V_CAPBANK_SETPOINT = 0.0;
    V_CAPBANK_REFERENCE = 0.0;

    reset_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE);

    /// Reset capacitor bank voltage controller for module A
    reset_dsp_error(ERROR_V_CAPBANK_MOD_A);
    reset_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_A);
    reset_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_A);
    reset_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_A);

    /// Reset rectifier output current controller for module A
    reset_dsp_error(ERROR_IOUT_RECT_MOD_A);
    reset_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_A);

    /// Reset capacitor bank voltage controller for module B
    reset_dsp_error(ERROR_V_CAPBANK_MOD_B);
    reset_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_B);
    reset_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_B);
    reset_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_B);

    /// Reset rectifier output current controller for module B
    reset_dsp_error(ERROR_IOUT_RECT_MOD_B);
    reset_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_B);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);
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
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    PWM_MODULATOR_MOD_A->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_MOD_A->ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
static interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

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

    V_CAPBANK_MOD_A = temp[0];
    IOUT_RECT_MOD_A = temp[1];
    V_CAPBANK_MOD_B = temp[2];
    IOUT_RECT_MOD_B = temp[3];

    /******** Timeslicer for controllers *********/
    RUN_TIMESLICER(TIMESLICER_CONTROLLER)
    /*********************************************/

        /// Run notch filters for capacitor bank voltage feedback
        run_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_A);
        run_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_A);

        run_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_B);
        run_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_B);

        /// Check whether power supply is ON
        if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
        {
            /// Calculate reference according to operation mode
            switch(g_ipc_ctom.ps_module[0].ps_status.bit.state)
            {
                case SlowRef:
                case SlowRefSync:
                {
                    run_dsp_srlim(SRLIM_V_CAPBANK_REFERENCE, USE_MODULE);
                    break;
                }
                case Cycle:
                {
                    break;
                }
                case RmpWfm:
                {
                    break;
                }
                case MigWfm:
                {
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
                SATURATE(V_CAPBANK_REFERENCE, MAX_REF_OL[0], MIN_REF_OL[0]);
                DUTY_CYCLE_MOD_A = 0.01 * V_CAPBANK_REFERENCE;
                SATURATE(DUTY_CYCLE_MOD_A, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
                DUTY_CYCLE_MOD_B = DUTY_CYCLE_MOD_A;
            }
            /// Closed-loop
            else
            {
                /// Run capacitor banks voltages control laws
                SATURATE(g_ipc_ctom.ps_module[0].ps_reference, MAX_REF[0], MIN_REF[0]);

                run_dsp_error(ERROR_V_CAPBANK_MOD_A);
                run_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_A);

                run_dsp_error(ERROR_V_CAPBANK_MOD_B);
                run_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_B);

                /// Run rectifiers output currents control laws
                run_dsp_error(ERROR_IOUT_RECT_MOD_A);
                run_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_A);
                SATURATE(DUTY_CYCLE_MOD_A, PWM_MAX_DUTY, PWM_MIN_DUTY);

                run_dsp_error(ERROR_IOUT_RECT_MOD_B);
                run_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_B);
                SATURATE(DUTY_CYCLE_MOD_B, PWM_MAX_DUTY, PWM_MIN_DUTY);
            }

            set_pwm_duty_chA(PWM_MODULATOR_MOD_A, DUTY_CYCLE_MOD_A);
            set_pwm_duty_chA(PWM_MODULATOR_MOD_B, DUTY_CYCLE_MOD_B);
        }

    /*********************************************/
    END_TIMESLICER(TIMESLICER_CONTROLLER)
    /*********************************************/

    RUN_SCOPE(SCOPE_MOD_A);
    RUN_SCOPE(SCOPE_MOD_B);

    SET_INTERLOCKS_TIMEBASE_FLAG(0);
    SET_INTERLOCKS_TIMEBASE_FLAG(1);

    PWM_MODULATOR_MOD_A->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;

    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT =  &isr_init_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    enable_pwm_interrupt(PWM_MODULATOR_MOD_A);

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
    disable_pwm_interrupt(PWM_MODULATOR_MOD_A);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * Turn power supply on.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_on(uint16_t dummy)
{
    #ifdef USE_ITLK
    if(g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state <= Interlock)
    #endif
    {
        reset_controller();

        g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state = Initializing;
        g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state = Initializing;

        PIN_CLOSE_AC_MAINS_CONTACTOR;

        DELAY_US(TIMEOUT_AC_MAINS_CONTACTOR_CLOSED_MS*1000);

        if(!PIN_STATUS_AC_MAINS_CONTACTOR)
        {
            PIN_SET_ACDC_INTERLOCK;

            BYPASS_HARD_INTERLOCK_DEBOUNCE(MOD_A_ID, Opened_Contactor_Fault);
            set_hard_interlock(MOD_A_ID, Opened_Contactor_Fault);

            BYPASS_HARD_INTERLOCK_DEBOUNCE(MOD_B_ID, Opened_Contactor_Fault);
            set_hard_interlock(MOD_B_ID, Opened_Contactor_Fault);
        }

        if(g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state == Initializing)
        {
            g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state = SlowRef;
            g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.openloop = OPEN_LOOP;
            g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state = SlowRef;
            enable_pwm_output(MOD_A_ID);
            enable_pwm_output(MOD_B_ID);
        }
    }
}

/**
 * Turn off specified power supply.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void turn_off(uint16_t dummy)
{
    disable_pwm_output(MOD_A_ID);
    disable_pwm_output(MOD_B_ID);

    PIN_OPEN_AC_MAINS_CONTACTOR;

    DELAY_US(TIMEOUT_AC_MAINS_CONTACTOR_OPENED_MS*1000);

    reset_controller();

    if(g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state != Interlock)
    {
        g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state = Off;
        g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state = Off;
    }
}

/**
 * Reset interlocks for specified power supply. Variable state from ps_module[0]
 * is shared between both modules.
 *
 * @param dummy dummy argument due to ps_module pointer
 */
static void reset_interlocks(uint16_t dummy)
{
    g_ipc_ctom.ps_module[MOD_A_ID].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[MOD_A_ID].ps_soft_interlock = 0;

    g_ipc_ctom.ps_module[MOD_B_ID].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[MOD_B_ID].ps_soft_interlock = 0;

    if(g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state = Off;
        g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state = Off;

        PIN_CLEAR_ACDC_INTERLOCK;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(V_CAPBANK_MOD_A) > MAX_V_CAPBANK)
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_A_ID, CapBank_Overvoltage);
    }

    if(fabs(V_CAPBANK_MOD_B) > MAX_V_CAPBANK)
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_B_ID, CapBank_Overvoltage);
    }

    if(fabs(IOUT_RECT_MOD_A) > MAX_IOUT_RECT)
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_A_ID, Rectifier_Overcurrent);
    }

    if(fabs(IOUT_RECT_MOD_B) > MAX_IOUT_RECT)
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_B_ID, Rectifier_Overcurrent);
    }

    if(PIN_STATUS_DCDC_INTERLOCK)
    {
        set_hard_interlock(MOD_A_ID, DCDC_Interlock);
        set_hard_interlock(MOD_B_ID, DCDC_Interlock);
    }

    DINT;

    if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
         (PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_A_ID, Welded_Contactor_Fault);
        set_hard_interlock(MOD_B_ID, Welded_Contactor_Fault);
    }

    else if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
              && (!PIN_STATUS_AC_MAINS_CONTACTOR) )
    {
        PIN_SET_ACDC_INTERLOCK;
        set_hard_interlock(MOD_A_ID, Opened_Contactor_Fault);
        set_hard_interlock(MOD_B_ID, Opened_Contactor_Fault);
    }

    EINT;

    if( g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state == Interlock &&
        g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state != Interlock )
    {
        set_hard_interlock(MOD_A_ID, Module_B_Interlock);
    }

    if( g_ipc_ctom.ps_module[MOD_A_ID].ps_status.bit.state == Interlock &&
        g_ipc_ctom.ps_module[MOD_B_ID].ps_status.bit.state != Interlock )
    {
        set_hard_interlock(MOD_B_ID, Module_A_Interlock);
    }

    //SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    run_interlocks_debouncing(1);
    //CLEAR_DEBUG_GPIO1;
}

static void init_controller_module_A(void)
{
    /************************************************************************/
    /** INITIALIZATION OF CAPACITOR BANK VOLTAGE CONTROL LOOP FOR MODULE A **/
    /************************************************************************/

    /**
     *        name:     ERROR_V_CAPBANK_MOD_A
     * description:     Capacitor bank voltage reference error for module A
     *  dsp module:     DSP_Error
     *           +:     V_CAPBANK_REFERENCE
     *           -:     V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A
     *         out:     V_CAPBANK_ERROR_MOD_A
     */

    init_dsp_error(ERROR_V_CAPBANK_MOD_A, &V_CAPBANK_REFERENCE,
                   &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A, &V_CAPBANK_ERROR_MOD_A);

    /**
     *        name:     PI_CONTROLLER_V_CAPBANK_MOD_A
     * description:     Capacitor bank voltage PI controller for module A
     *  dsp module:     DSP_PI
     *          in:     V_CAPBANK_ERROR_MOD_A
     *         out:     IOUT_RECT_REF_MOD_A
     */

    init_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_A, KP_V_CAPBANK_MOD_A,
                KI_V_CAPBANK_MOD_A, CONTROLLER_FREQ_SAMP, MAX_IOUT_RECT_REF,
                MIN_IOUT_RECT_REF, &V_CAPBANK_ERROR_MOD_A, &IOUT_RECT_REF_MOD_A);

    /**
     *        name:     NOTCH_FILT_2HZ_V_CAPBANK_MOD_A
     * description:     Cap bank voltage notch filter (fcut = 2 Hz) for module A
     *    DP class:     DSP_IIR_2P2Z
     *          in:     V_CAPBANK_MOD_A
     *         out:     V_CAPBANK_FILTERED_2HZ_MOD_A
     */

    init_dsp_notch_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_A, NF_ALPHA, 2.0,
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
                        &V_CAPBANK_MOD_A, &V_CAPBANK_FILTERED_2HZ_MOD_A);

    /*init_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_A,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS.b0,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS.b1,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS.b2,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS.a1,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_MOD_A, &V_CAPBANK_FILTERED_2HZ_MOD_A);*/

    /**
     *        name:     NOTCH_FILT_4HZ_V_CAPBANK_MOD_A
     * description:     Cap bank voltage notch filter (fcut = 4 Hz) for module A
     *    DP class:     DSP_IIR_2P2Z
     *          in:     V_CAPBANK_FILTERED_2HZ_MOD_A
     *         out:     V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A
     */

    init_dsp_notch_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_A, NF_ALPHA, 4.0,
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
                        &V_CAPBANK_FILTERED_2HZ_MOD_A,
                        &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A);

    /*init_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_A,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS.b0,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS.b1,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS.b2,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS.a1,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_A_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_FILTERED_2HZ_MOD_A,
                      &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A);*/

    /**************************************************************************/
    /** INITIALIZATION OF RECTIFIER OUTPUT CURRENT CONTROL LOOP FOR MODULE A **/
    /**************************************************************************/

    /**
     *        name:     ERROR_IOUT_RECT_MOD_A
     * description:     Rectifier output current reference error for module A
     *    DP class:     DSP_Error
     *           +:     IOUT_RECT_REF_MOD_A
     *           -:     IOUT_RECT_MOD_A
     *         out:     IOUT_RECT_ERROR_MOD_A
     */

    init_dsp_error(ERROR_IOUT_RECT_MOD_A, &IOUT_RECT_REF_MOD_A,
                   &IOUT_RECT_MOD_A, &IOUT_RECT_ERROR_MOD_A);

    /**
     *        name:     PI_CONTROLLER_IOUT_RECT_MOD_A
     * description:     Rectifier output current PI controller for module A
     *    DP class:     DSP_PI
     *          in:     IOUT_RECT_ERROR_MOD_A
     *         out:     DUTY_CYCLE_MOD_A
     */
    init_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_A, KP_IOUT_RECT_MOD_A,
                KI_IOUT_RECT_MOD_A, CONTROLLER_FREQ_SAMP, PWM_MAX_DUTY,
                PWM_MIN_DUTY, &IOUT_RECT_ERROR_MOD_A, &DUTY_CYCLE_MOD_A);
}

static void init_controller_module_B(void)
{
    /************************************************************************/
    /** INITIALIZATION OF CAPACITOR BANK VOLTAGE CONTROL LOOP FOR MODULE B **/
    /************************************************************************/

    /**
     *        name:     ERROR_V_CAPBANK_MOD_B
     * description:     Capacitor bank voltage reference error for module B
     *  dsp module:     DSP_Error
     *           +:     V_CAPBANK_REFERENCE
     *           -:     V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B
     *         out:     V_CAPBANK_ERROR_MOD_B
     */

    init_dsp_error(ERROR_V_CAPBANK_MOD_B, &V_CAPBANK_REFERENCE,
                   &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B, &V_CAPBANK_ERROR_MOD_B);

    /**
     *        name:     PI_CONTROLLER_V_CAPBANK_MOD_B
     * description:     Capacitor bank voltage PI controller for module B
     *  dsp module:     DSP_PI
     *          in:     V_CAPBANK_ERROR_MOD_B
     *         out:     IOUT_RECT_REF_MOD_B
     */

    init_dsp_pi(PI_CONTROLLER_V_CAPBANK_MOD_B, KP_V_CAPBANK_MOD_B,
                KI_V_CAPBANK_MOD_B, CONTROLLER_FREQ_SAMP, MAX_IOUT_RECT_REF,
                MIN_IOUT_RECT_REF, &V_CAPBANK_ERROR_MOD_B, &IOUT_RECT_REF_MOD_B);

    /**
     *        name:     NOTCH_FILT_2HZ_V_CAPBANK_MOD_B
     * description:     Cap bank voltage notch filter (fcut = 2 Hz) for module B
     *    DP class:     DSP_IIR_2P2Z
     *          in:     V_CAPBANK_MOD_B
     *         out:     V_CAPBANK_FILTERED_2HZ_MOD_B
     */

    init_dsp_notch_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_B, NF_ALPHA, 2.0,
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
                        &V_CAPBANK_MOD_B, &V_CAPBANK_FILTERED_2HZ_MOD_B);

    /*init_dsp_iir_2p2z(NOTCH_FILT_2HZ_V_CAPBANK_MOD_B,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS.b0,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS.b1,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS.b2,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS.a1,
                      NOTCH_FILT_2HZ_V_CAPBANK_MOD_B_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_MOD_B, &V_CAPBANK_FILTERED_2HZ_MOD_B);*/

    /**
     *        name:     NOTCH_FILT_4HZ_V_CAPBANK_MOD_B
     * description:     Cap bank voltage notch filter (fcut = 4 Hz) for module B
     *    DP class:     DSP_IIR_2P2Z
     *          in:     V_CAPBANK_FILTERED_2HZ_MOD_B
     *         out:     V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B
     */

    init_dsp_notch_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_B, NF_ALPHA, 4.0,
                        CONTROLLER_FREQ_SAMP, FLT_MAX, -FLT_MAX,
                        &V_CAPBANK_FILTERED_2HZ_MOD_B,
                        &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B);

    /*init_dsp_iir_2p2z(NOTCH_FILT_4HZ_V_CAPBANK_MOD_B,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS.b0,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS.b1,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS.b2,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS.a1,
                      NOTCH_FILT_4HZ_V_CAPBANK_MOD_B_COEFFS.a2,
                      FLT_MAX, -FLT_MAX,
                      &V_CAPBANK_FILTERED_2HZ_MOD_B,
                      &V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B);*/

    /**************************************************************************/
    /** INITIALIZATION OF RECTIFIER OUTPUT CURRENT CONTROL LOOP FOR MODULE B **/
    /**************************************************************************/

    /**
     *        name:     ERROR_IOUT_RECT_MOD_B
     * description:     Rectifier output current reference error for module B
     *    DP class:     DSP_Error
     *           +:     IOUT_RECT_REF_MOD_B
     *           -:     IOUT_RECT_MOD_B
     *         out:     IOUT_RECT_ERROR_MOD_B
     */

    init_dsp_error(ERROR_IOUT_RECT_MOD_B, &IOUT_RECT_REF_MOD_B,
                   &IOUT_RECT_MOD_B, &IOUT_RECT_ERROR_MOD_B);

    /**
     *        name:     PI_CONTROLLER_IOUT_RECT_MOD_B
     * description:     Rectifier output current PI controller for module B
     *    DP class:     DSP_PI
     *          in:     IOUT_RECT_ERROR_MOD_B
     *         out:     DUTY_CYCLE_MOD_B
     */
    init_dsp_pi(PI_CONTROLLER_IOUT_RECT_MOD_B, KP_IOUT_RECT_MOD_B,
                KI_IOUT_RECT_MOD_B, CONTROLLER_FREQ_SAMP, PWM_MAX_DUTY,
                PWM_MIN_DUTY, &IOUT_RECT_ERROR_MOD_B, &DUTY_CYCLE_MOD_B);
}

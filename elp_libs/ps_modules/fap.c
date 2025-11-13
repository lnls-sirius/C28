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
 * @file fap.c
 * @brief FAP module
 * 
 * Module for control of FAP power supplies. It implements the controller for
 * load current.
 *
 * @author gabriel.brunheira
 * @date 08/08/2018
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

#include "fap.h"

/**
 * Control parameters
 */
#define TIMESLICER_I_SHARE_CONTROLLER_IDX   0
#define TIMESLICER_I_SHARE_CONTROLLER       g_controller_ctom.timeslicer[TIMESLICER_I_SHARE_CONTROLLER_IDX]
#define I_SHARE_CONTROLLER_FREQ_SAMP        TIMESLICER_FREQ[TIMESLICER_I_SHARE_CONTROLLER_IDX]

/**
 * Analog variables parameters
 */
#define MAX_ILOAD               ANALOG_VARS_MAX[0]
#define MAX_VLOAD               ANALOG_VARS_MAX[1]

#define MAX_I_IGBT              ANALOG_VARS_MAX[2]
#define MAX_IGBT_DIFF           ANALOG_VARS_MAX[3]

#define MAX_V_DCLINK            ANALOG_VARS_MAX[4]
#define MIN_V_DCLINK            ANALOG_VARS_MIN[4]

#define MAX_DCCTS_DIFF          ANALOG_VARS_MAX[5]

#define MAX_I_IDLE_DCCT         ANALOG_VARS_MAX[6]
#define MIN_I_ACTIVE_DCCT       ANALOG_VARS_MIN[6]

#define I_IGBT_DIFF_MODE        ANALOG_VARS_MAX[7]

#define TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS      ANALOG_VARS_MAX[8]
#define TIMEOUT_DCLINK_CONTACTOR_OPENED_MS      ANALOG_VARS_MAX[9]

#define NUM_DCCTs                               ANALOG_VARS_MAX[10]

#define RESET_PULSE_TIME_DCLINK_CONTACTOR_MS    ANALOG_VARS_MAX[11]

#define MAX_V_DCLINK_TURN_ON    ANALOG_VARS_MAX[12]
/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1].f  // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2].f  // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3].f
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4].f
#define I_LOAD_DIFF             g_controller_ctom.net_signals[16].f

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[6].f

#define DUTY_MEAN               g_controller_ctom.net_signals[5].f
#define DUTY_DIFF               g_controller_ctom.net_signals[7].f

#define V_DCLINK_FILTERED       g_controller_ctom.net_signals[10].f

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1].f

/// ARM Net Signals
#define I_IGBT_1                g_controller_mtoc.net_signals[0].f  // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1].f  // ANI1

#define V_LOAD                  g_controller_mtoc.net_signals[2]

/// Reference
#define I_LOAD_SETPOINT             g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE            g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE      &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define WFMREF                      g_ipc_ctom.wfmref[0]

#define SIGGEN                      SIGGEN_CTOM[0]
#define SRLIM_SIGGEN_AMP            &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET         &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define MAX_SLEWRATE_SLOWREF            g_controller_mtoc.dsp_modules.dsp_srlim[0].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_AMP         g_controller_mtoc.dsp_modules.dsp_srlim[1].coeffs.s.max_slewrate
#define MAX_SLEWRATE_SIGGEN_OFFSET      g_controller_mtoc.dsp_modules.dsp_srlim[2].coeffs.s.max_slewrate

/// Load current controller
#define ERROR_I_LOAD                    &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_CONTROLLER_I_LOAD            &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS     g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.ki

/// IGBTs current share controller
#define ERROR_I_SHARE                   &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_I_SHARE           &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_I_SHARE_COEFFS    g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.kp
#define KI_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.ki

/// DC-link voltage feedforward controller
#define IIR_2P2Z_LPF_V_DCLINK           &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define IIR_2P2Z_LPF_V_DCLINK_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

#define FF_V_DCLINK_IGBT_1              &g_controller_ctom.dsp_modules.dsp_ff[0]
#define FF_V_DCLINK_IGBT_1_COEFFS       g_controller_mtoc.dsp_modules.dsp_ff[0].coeffs.s
#define NOM_V_DCLINK_FF_IGBT_1          FF_V_DCLINK_IGBT_1_COEFFS.vdc_nom
#define MIN_V_DCLINK_FF_IGBT_1          FF_V_DCLINK_IGBT_1_COEFFS.vdc_min

#define FF_V_DCLINK_IGBT_2              &g_controller_ctom.dsp_modules.dsp_ff[1]
#define FF_V_DCLINK_IGBT_2_COEFFS       g_controller_mtoc.dsp_modules.dsp_ff[1].coeffs.s
#define NOM_V_DCLINK_FF_IGBT_2          FF_V_DCLINK_IGBT_2_COEFFS.vdc_nom
#define MIN_V_DCLINK_FF_IGBT_2          FF_V_DCLINK_IGBT_2_COEFFS.vdc_min

/// PWM modulators
#define PWM_MODULATOR_IGBT_1            g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_IGBT_2            g_pwm_modules.pwm_regs[1]

/// Scope
#define SCOPE                           SCOPE_CTOM[0]

/**
 * Digital I/O's status
 */
#define PIN_OPEN_DCLINK_CONTACTOR       CLEAR_GPDO1;
#define PIN_CLOSE_DCLINK_CONTACTOR      SET_GPDO1;

#define PIN_STATUS_DCLINK_CONTACTOR     GET_GPDI5

#define PIN_STATUS_DCCT_1_STATUS        GET_GPDI9
#define PIN_STATUS_DCCT_1_ACTIVE        GET_GPDI10
#define PIN_STATUS_DCCT_2_STATUS        GET_GPDI11
#define PIN_STATUS_DCCT_2_ACTIVE        GET_GPDI12

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    IGBT_1_Overcurrent,
    IGBT_2_Overcurrent,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    IGBTs_Current_High_Difference,
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS             IIB_Itlk + 1
#define NUM_SOFT_INTERLOCKS             IGBTs_Current_High_Difference + 1

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

/**
 * Main function for this power supply module
 */
void main_fap(void)
{
    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    reset_controller();
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

    /// Initialization of PWM modules
    g_pwm_modules.num_modules = 2;

    PWM_MODULATOR_IGBT_1 = &EPwm1Regs;
    PWM_MODULATOR_IGBT_2 = &EPwm2Regs;

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PWM initialization
    init_pwm_module(PWM_MODULATOR_IGBT_1, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Independent, PWM_DEAD_TIME);
    init_pwm_module(PWM_MODULATOR_IGBT_2, PWM_FREQ, 1, PWM_Sync_Slave, 180,
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

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ,
                &g_ipc_ctom.ps_module[0].ps_reference);

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
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     I_LOAD_ERROR
     *         out:     DUTY_MEAN
     */

    init_dsp_pi(PI_CONTROLLER_I_LOAD, KP_I_LOAD, KI_I_LOAD, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &I_LOAD_ERROR, &DUTY_MEAN);

    /*******************************************************/
    /** INITIALIZATION OF IGBT CURRENT SHARE CONTROL LOOP **/
    /*******************************************************/

    /**
     *        name:     ERROR_I_SHARE
     * description:     IGBT current share error
     *  dsp module:     DSP_Error
     *           +:     I_IGBT_1
     *           -:     I_IGBT_2
     *         out:     I_IGBTS_DIFF
     */

    init_dsp_error(ERROR_I_SHARE, &I_IGBT_1, &I_IGBT_2, &I_IGBTS_DIFF);

    /**
     *        name:     PI_CONTROLLER_I_SHARE
     * description:     IGBT current share PI controller
     *  dsp module:     DSP_PI
     *          in:     I_IGBTS_DIFF
     *         out:     DUTY_DIFF
     */

    init_dsp_pi(PI_CONTROLLER_I_SHARE, KP_I_SHARE, KI_I_SHARE,
                I_SHARE_CONTROLLER_FREQ_SAMP, PWM_LIM_DUTY_SHARE,
                -PWM_LIM_DUTY_SHARE, &I_IGBTS_DIFF, &DUTY_DIFF);

    /***************************************************/
    /** INITIALIZATION OF DC-LINK VOLTAGE FEEDFORWARD **/
    /***************************************************/

    /**
     *        name:     IIR_2P2Z_LPF_V_DCLINK
     * description:     DC-Link voltage low-pass filter
     *    DP class:     ELP_IIR_2P2Z
     *          in:     V_DCLINK
     *         out:     V_DCLINK_FILTERED
     */

    init_dsp_iir_2p2z(IIR_2P2Z_LPF_V_DCLINK,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b0,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b1,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b2,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.a1,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.a2,
                      FLT_MAX, -FLT_MAX, &V_DCLINK, &V_DCLINK_FILTERED);

    /**
     *        name:     FF_V_DCLINK_IGBT_1
     * description:     DC-Link voltage feed-forward for IGBT 1
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     V_DCLINK_FILTERED
     *          in:     net_signals[8]
     *         out:     output_signals[0]
     */

    init_dsp_vdclink_ff(FF_V_DCLINK_IGBT_1, NOM_V_DCLINK_FF_IGBT_1,
                        MIN_V_DCLINK_FF_IGBT_1,
                        &V_DCLINK_FILTERED, &g_controller_ctom.net_signals[8].f,
                        &g_controller_ctom.output_signals[0].f);

    /**
     *        name:     FF_V_DCLINK_IGBT_2
     * description:     DC-Link voltage feed-forward for IGBT 2
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     V_DCLINK_FILTERED
     *          in:     net_signals[9]
     *         out:     output_signals[1]
     */

    init_dsp_vdclink_ff(FF_V_DCLINK_IGBT_2, NOM_V_DCLINK_FF_IGBT_2,
                        MIN_V_DCLINK_FF_IGBT_2,
                        &V_DCLINK_FILTERED, &g_controller_ctom.net_signals[9].f,
                        &g_controller_ctom.output_signals[1].f);

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for IGBT current share controller
     */
    init_timeslicer(&TIMESLICER_I_SHARE_CONTROLLER, ISR_CONTROL_FREQ);
    cfg_timeslicer(&TIMESLICER_I_SHARE_CONTROLLER, I_SHARE_CONTROLLER_FREQ_SAMP);

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
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_1, 0.0);
    set_pwm_duty_chA(PWM_MODULATOR_IGBT_2, 0.0);

    g_ipc_ctom.ps_module[0].ps_status.bit.openloop = LOOP_STATE;

    I_LOAD_SETPOINT = 0.0;
    I_LOAD_REFERENCE = 0.0;

    reset_dsp_srlim(SRLIM_I_LOAD_REFERENCE);
    reset_dsp_error(ERROR_I_LOAD);
    reset_dsp_pi(PI_CONTROLLER_I_LOAD);

    reset_dsp_error(ERROR_I_SHARE);
    reset_dsp_pi(PI_CONTROLLER_I_SHARE);

    reset_dsp_vdclink_ff(FF_V_DCLINK_IGBT_1);
    reset_dsp_vdclink_ff(FF_V_DCLINK_IGBT_2);

    reset_dsp_srlim(SRLIM_SIGGEN_AMP);
    reset_dsp_srlim(SRLIM_SIGGEN_OFFSET);
    disable_siggen(&SIGGEN);

    reset_wfmref(&WFMREF);
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

    PWM_MODULATOR_IGBT_1->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_IGBT_1->ETCLR.bit.INT = 1;

    PWM_MODULATOR_IGBT_2->ETSEL.bit.INTSEL = ET_CTR_ZERO;
    PWM_MODULATOR_IGBT_2->ETCLR.bit.INT = 1;

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
        V_DCLINK = temp[2];
        g_controller_ctom.net_signals[20].f = temp[2];

        I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
        I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;
    }
    else
    {
        I_LOAD_1 = temp[0];
        V_DCLINK = temp[1];
        g_controller_ctom.net_signals[20].f = temp[2];
        g_controller_ctom.net_signals[21].f = temp[3];

        I_LOAD_MEAN = I_LOAD_1;
        I_LOAD_DIFF = 0;
    }

    I_IGBTS_DIFF = I_IGBT_1 - I_IGBT_2;

    /// Run low-pass filter for DC-Link voltage
    run_dsp_iir_2p2z(IIR_2P2Z_LPF_V_DCLINK);

    /// Check whether power supply is ON
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state >= SlowRef)
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
            DUTY_CYCLE_IGBT_1 = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE_IGBT_1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);

            DUTY_CYCLE_IGBT_2 = DUTY_CYCLE_IGBT_1;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF[0], MIN_REF[0]);
            run_dsp_error(ERROR_I_LOAD);
            run_dsp_pi(PI_CONTROLLER_I_LOAD);

            /*********************************************/
            RUN_TIMESLICER(TIMESLICER_I_SHARE_CONTROLLER)
            /*********************************************/

                if(I_IGBT_DIFF_MODE)
                {
                    I_IGBTS_DIFF = I_IGBT_1 - 0.5*I_LOAD_MEAN;
                }
                else
                {
                    run_dsp_error(ERROR_I_SHARE);
                }

                run_dsp_pi(PI_CONTROLLER_I_SHARE);

            /*********************************************/
            END_TIMESLICER(TIMESLICER_I_SHARE_CONTROLLER)
            /*********************************************/

            g_controller_ctom.net_signals[8].f = DUTY_MEAN - DUTY_DIFF;
            g_controller_ctom.net_signals[9].f = DUTY_MEAN + DUTY_DIFF;

            run_dsp_vdclink_ff(FF_V_DCLINK_IGBT_1);
            run_dsp_vdclink_ff(FF_V_DCLINK_IGBT_2);

            SATURATE(DUTY_CYCLE_IGBT_1, PWM_MAX_DUTY, PWM_MIN_DUTY);
            SATURATE(DUTY_CYCLE_IGBT_2, PWM_MAX_DUTY, PWM_MIN_DUTY);
        }

        set_pwm_duty_chA(PWM_MODULATOR_IGBT_1, DUTY_CYCLE_IGBT_1);
        set_pwm_duty_chA(PWM_MODULATOR_IGBT_2, DUTY_CYCLE_IGBT_2);
    }

    RUN_SCOPE(SCOPE);

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
    PWM_MODULATOR_IGBT_1->ETCLR.bit.INT = 1;
    PWM_MODULATOR_IGBT_2->ETCLR.bit.INT = 1;
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
    PieVectTable.EPWM2_INT =  &isr_controller;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    enable_pwm_interrupt(PWM_MODULATOR_IGBT_1);
    enable_pwm_interrupt(PWM_MODULATOR_IGBT_2);

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
    disable_pwm_interrupt(PWM_MODULATOR_IGBT_1);
    disable_pwm_interrupt(PWM_MODULATOR_IGBT_2);

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
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Off)
    #else
    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    #endif
    {
        if(V_DCLINK > MAX_V_DCLINK_TURN_ON)
        {
            BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Overvoltage);
            set_hard_interlock(0, DCLink_Overvoltage);
        }

        #ifdef USE_ITLK
        else
        {
        #endif

            PIN_CLOSE_DCLINK_CONTACTOR;
            DELAY_US(TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS*1000);

            if(!PIN_STATUS_DCLINK_CONTACTOR)
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(0, Opened_Contactor_Fault);
                set_hard_interlock(0, Opened_Contactor_Fault);
            }

            #ifdef USE_ITLK
            else
            {
            #endif
                g_ipc_ctom.ps_module[0].ps_status.bit.state = Initializing;
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

    PIN_OPEN_DCLINK_CONTACTOR;
    DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);

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
        if(PIN_STATUS_DCLINK_CONTACTOR)
        {
            PIN_CLOSE_DCLINK_CONTACTOR;
            DELAY_US(RESET_PULSE_TIME_DCLINK_CONTACTOR_MS*1000);
            PIN_OPEN_DCLINK_CONTACTOR;
            DELAY_US(TIMEOUT_DCLINK_CONTACTOR_OPENED_MS*1000);
        }

        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    //SET_DEBUG_GPIO1;

    if(fabs(I_LOAD_MEAN) > MAX_ILOAD)
    {
        set_hard_interlock(0, Load_Overcurrent);
    }

    if(fabs(I_IGBT_1) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_1_Overcurrent);
    }

    if(fabs(I_IGBT_2) > MAX_I_IGBT)
    {
        set_hard_interlock(0, IGBT_2_Overcurrent);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(0, DCCT_High_Difference);
    }

    if(fabs(I_IGBTS_DIFF) > MAX_IGBT_DIFF)
    {
        set_soft_interlock(0, IGBTs_Current_High_Difference);
    }

    if(fabs(V_DCLINK) > MAX_V_DCLINK)
    {
        set_hard_interlock(0, DCLink_Overvoltage);
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

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(0, Welded_Contactor_Fault);
        }
    }
    else
    {
        if(!PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(0, Opened_Contactor_Fault);
        }

        if(g_ipc_ctom.ps_module[0].ps_status.bit.state == Initializing)
        {
            if(V_DCLINK > MIN_V_DCLINK)
            {
                g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
                enable_pwm_output(0);
                enable_pwm_output(1);
            }
        }
        else if(g_ipc_ctom.ps_module[0].ps_status.bit.state > Initializing) /// Power supply ON
        {
            if(V_DCLINK < MIN_V_DCLINK)
            {
                set_hard_interlock(0, DCLink_Undervoltage);
            }
        }
    }

    EINT;

    //CLEAR_DEBUG_GPIO1;
    //SET_DEBUG_GPIO1;
    run_interlocks_debouncing(0);
    //CLEAR_DEBUG_GPIO1;
}

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
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fap.h"

//#define USE_ITLK

/**
 * PWM parameters
 */
#define PWM_FREQ                g_ipc_mtoc.pwm.freq_pwm
#define PWM_DEAD_TIME           g_ipc_mtoc.pwm.dead_time
#define PWM_MAX_DUTY            g_ipc_mtoc.pwm.max_duty
#define PWM_MIN_DUTY            g_ipc_mtoc.pwm.min_duty
#define PWM_MAX_DUTY_OL         g_ipc_mtoc.pwm.max_duty_openloop
#define PWM_MIN_DUTY_OL         g_ipc_mtoc.pwm.min_duty_openloop
#define PWM_LIM_DUTY_SHARE      g_ipc_mtoc.pwm.lim_duty_share

/**
 * Control parameters
 */
#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MIN_REF                 g_ipc_mtoc.control.min_ref
#define MAX_REF                 g_ipc_mtoc.control.max_ref
#define MAX_REF_OL              g_ipc_mtoc.control.max_ref_openloop
#define MIN_REF_OL              g_ipc_mtoc.control.min_ref_openloop
#define MAX_REF_SLEWRATE        g_ipc_mtoc.control.slewrate_slowref
#define MAX_SR_SIGGEN_OFFSET    g_ipc_mtoc.control.slewrate_siggen_offset
#define MAX_SR_SIGGEN_AMP       g_ipc_mtoc.control.slewrate_siggen_amp

#define ISR_CONTROL_FREQ        g_ipc_mtoc.control.freq_isr_control

#define HRADC_FREQ_SAMP         g_ipc_mtoc.hradc.freq_hradc_sampling
#define HRADC_SPI_CLK           g_ipc_mtoc.hradc.freq_spiclk
#define NUM_HRADC_BOARDS        g_ipc_mtoc.hradc.num_hradc

#define TIMESLICER_BUFFER       1
#define BUFFER_FREQ             g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_BUFFER]
#define BUFFER_DECIMATION       (uint16_t) roundf(ISR_CONTROL_FREQ / BUFFER_FREQ)


#define TIMESLICER_I_SHARE_CONTROLLER   2
#define I_SHARE_CONTROLLER_FREQ_SAMP    g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_I_SHARE_CONTROLLER]
#define I_SHARE_CONTROLLER_DECIMATION   (uint16_t) roundf(ISR_CONTROL_FREQ / I_SHARE_CONTROLLER_FREQ_SAMP)

#define SIGGEN                  g_ipc_ctom.siggen

/**
 * HRADC parameters
 */
#define HRADC_HEATER_ENABLE     g_ipc_mtoc.hradc.enable_heater
#define HRADC_MONITOR_ENABLE    g_ipc_mtoc.hradc.enable_monitor
#define TRANSDUCER_OUTPUT_TYPE  g_ipc_mtoc.hradc.type_transducer_output
#if (HRADC_v2_0)
    #define TRANSDUCER_GAIN     -g_ipc_mtoc.hradc.gain_transducer
#endif
#if (HRADC_v2_1)
    #define TRANSDUCER_GAIN     g_ipc_mtoc.hradc.gain_transducer
#endif

/**
 * Analog variables parameters
 */
#define MAX_ILOAD               g_ipc_mtoc.analog_vars.max[0]
#define MAX_VLOAD               g_ipc_mtoc.analog_vars.max[1]

#define MAX_I_IGBT              g_ipc_mtoc.analog_vars.max[2]
#define MAX_IGBT_DIFF           g_ipc_mtoc.analog_vars.max[3]

#define MAX_V_DCLINK            g_ipc_mtoc.analog_vars.max[4]
#define MIN_V_DCLINK            g_ipc_mtoc.analog_vars.min[4]

#define NOM_V_DCLINK_FF         g_ipc_mtoc.analog_vars.max[5]
#define MIN_V_DCLINK_FF         g_ipc_mtoc.analog_vars.min[5]

#define MAX_DCCTS_DIFF          g_ipc_mtoc.analog_vars.max[6]

#define MAX_I_IDLE_DCCT         g_ipc_mtoc.analog_vars.max[7]
#define MIN_I_ACTIVE_DCCT       g_ipc_mtoc.analog_vars.min[7]

#define I_IGBT_DIFF_MODE        g_ipc_mtoc.analog_vars.max[8]

#define TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS      g_ipc_mtoc.analog_vars.max[9]
#define TIMEOUT_DCLINK_CONTACTOR_OPENED_MS      g_ipc_mtoc.analog_vars.max[10]

#define NETSIGNAL_ELEM_CTOM_BUF     g_ipc_mtoc.analog_vars.max[11]
#define NETSIGNAL_ELEM_MTOC_BUF     g_ipc_mtoc.analog_vars.min[11]

#define NETSIGNAL_CTOM_BUF      g_controller_ctom.net_signals[(uint16_t) NETSIGNAL_ELEM_CTOM_BUF].f
#define NETSIGNAL_MTOC_BUF      g_controller_mtoc.net_signals[(uint16_t) NETSIGNAL_ELEM_MTOC_BUF].f

/**
 * Controller defines
 */
#define I_LOAD_1                g_controller_ctom.net_signals[0].f  // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1].f  // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2].f  // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3].f
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4].f
#define I_LOAD_DIFF             g_controller_ctom.net_signals[16].f

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[6].f
#define I_IGBT_1                g_controller_mtoc.net_signals[0].f  // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1].f  // ANI1

#define DUTY_MEAN               g_controller_ctom.net_signals[5].f
#define DUTY_DIFF               g_controller_ctom.net_signals[7].f

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0].f
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1].f

#define I_LOAD_SETPOINT         g_ipc_ctom.ps_module[0].ps_setpoint
#define I_LOAD_REFERENCE        g_ipc_ctom.ps_module[0].ps_reference

#define SRLIM_I_LOAD_REFERENCE          &g_controller_ctom.dsp_modules.dsp_srlim[0]

#define ERROR_I_LOAD                    &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_CONTROLLER_I_LOAD            &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_I_LOAD_COEFFS     g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s
#define KP_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.kp
#define KI_I_LOAD                       PI_CONTROLLER_I_LOAD_COEFFS.ki

#define ERROR_I_SHARE                   &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_I_SHARE           &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_I_SHARE_COEFFS    g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s
#define KP_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.kp
#define KI_I_SHARE                      PI_CONTROLLER_I_SHARE_COEFFS.ki

#define IIR_2P2Z_LPF_V_DCLINK           &g_controller_ctom.dsp_modules.dsp_iir_2p2z[0]
#define IIR_2P2Z_LPF_V_DCLINK_COEFFS    g_controller_mtoc.dsp_modules.dsp_iir_2p2z[0].coeffs.s

#define FF_V_DCLINK_IGBT_1              &g_controller_ctom.dsp_modules.dsp_ff[0]
#define FF_V_DCLINK_IGBT_2              &g_controller_ctom.dsp_modules.dsp_ff[1]

#define PWM_MODULATOR_IGBT_1            g_pwm_modules.pwm_regs[0]
#define PWM_MODULATOR_IGBT_2            g_pwm_modules.pwm_regs[1]

#define SRLIM_SIGGEN_AMP                &g_controller_ctom.dsp_modules.dsp_srlim[1]
#define SRLIM_SIGGEN_OFFSET             &g_controller_ctom.dsp_modules.dsp_srlim[2]

#define BUF_SAMPLES                     &g_ipc_ctom.buf_samples[0]

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
 * Interlock defines
 */
#define LOAD_OVERCURRENT        0x00000001
#define LOAD_OVERVOLTAGE        0x00000002
#define V_DCLINK_OVERVOLTAGE    0x00000004
#define V_DCLINK_UNDERVOLTAGE   0x00000008
#define DCLINK_CONTACTOR_FAIL   0x00000010
#define IGBT_1_OVERCURRENT      0x00000020
#define IGBT_2_OVERCURRENT      0x00000040

#define DCCT_1_FAULT            0x00000001
#define DCCT_2_FAULT            0x00000002
#define DCCTS_HIGH_DIFFERENCE   0x00000004
#define I_LOAD_1_FEEDBACK_FAULT 0x00000008
#define I_LOAD_2_FEEDBACK_FAULT 0x00000010
#define IGBTS_HIGH_DIFFERENCE   0x00000020

static uint16_t decimation_factor;
static float decimation_coeff;

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(set_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(set_soft_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_hard_interlock, "ramfuncs");
#pragma CODE_SECTION(isr_soft_interlock, "ramfuncs");

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
static void set_hard_interlock(uint32_t itlk);
static void set_soft_interlock(uint32_t itlk);
static interrupt void isr_hard_interlock(void);
static interrupt void isr_soft_interlock(void);

static inline void check_interlocks(void);

/**
 * Main function for this power supply module
 */
void main_fap(void)
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

    init_ipc();
    init_control_framework(&g_controller_ctom);

    /***********************************************/
    /** INITIALIZATION OF SIGNAL GENERATOR MODULE **/
    /***********************************************/

    disable_siggen(&SIGGEN);

    init_siggen(&SIGGEN, ISR_CONTROL_FREQ,
                &g_ipc_ctom.ps_module[0].ps_reference);

    cfg_siggen(&SIGGEN, g_ipc_mtoc.siggen.type, g_ipc_mtoc.siggen.num_cycles,
               g_ipc_mtoc.siggen.freq, g_ipc_mtoc.siggen.amplitude,
               g_ipc_mtoc.siggen.offset, g_ipc_mtoc.siggen.aux_param);

    /**
     *        name:     SRLIM_SIGGEN_AMP
     * description:     Signal generator amplitude slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.amplitude
     *         out:     g_ipc_ctom.siggen.amplitude
     */

    init_dsp_srlim(SRLIM_SIGGEN_AMP, MAX_SR_SIGGEN_AMP, ISR_CONTROL_FREQ,
                   &g_ipc_mtoc.siggen.amplitude, &g_ipc_ctom.siggen.amplitude);

    /**
     *        name:     SRLIM_SIGGEN_OFFSET
     * description:     Signal generator offset slew-rate limiter
     *    DP class:     DSP_SRLim
     *          in:     g_ipc_mtoc.siggen.offset
     *         out:     g_ipc_ctom.siggen.offset
     */

    init_dsp_srlim(SRLIM_SIGGEN_OFFSET, MAX_SR_SIGGEN_OFFSET,
                   ISR_CONTROL_FREQ, &g_ipc_mtoc.siggen.offset,
                   &g_ipc_ctom.siggen.offset);

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

    init_dsp_srlim(SRLIM_I_LOAD_REFERENCE, MAX_REF_SLEWRATE, ISR_CONTROL_FREQ,
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
     *          in:     net_signals[2]
     *         out:     net_signals[10]
     */

    init_dsp_iir_2p2z(IIR_2P2Z_LPF_V_DCLINK,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b0,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b1,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.b2,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.a1,
                      IIR_2P2Z_LPF_V_DCLINK_COEFFS.a2,
                      FLT_MAX, -FLT_MAX, &g_controller_ctom.net_signals[2].f,
                      &g_controller_ctom.net_signals[10].f);

    /**
     *        name:     FF_V_DCLINK_IGBT_1
     * description:     DC-Link voltage feed-forward for IGBT 1
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     net_signals[10]
     *          in:     net_signals[8]
     *         out:     output_signals[0]
     */

    init_dsp_vdclink_ff(FF_V_DCLINK_IGBT_1, NOM_V_DCLINK_FF, MIN_V_DCLINK_FF,
                        &g_controller_ctom.net_signals[10].f,
                        &g_controller_ctom.net_signals[8].f,
                        &g_controller_ctom.output_signals[0].f);

    /**
     *        name:     FF_V_DCLINK_IGBT_2
     * description:     DC-Link voltage feed-forward for IGBT 2
     *    DP class:     DSP_VdcLink_FeedForward
     *    vdc_meas:     net_signals[10]
     *          in:     net_signals[9]
     *         out:     output_signals[1]
     */

    init_dsp_vdclink_ff(FF_V_DCLINK_IGBT_2, NOM_V_DCLINK_FF, MIN_V_DCLINK_FF,
                        &g_controller_ctom.net_signals[10].f,
                        &g_controller_ctom.net_signals[9].f,
                        &g_controller_ctom.output_signals[1].f);

    /************************************/
    /** INITIALIZATION OF TIME SLICERS **/
    /************************************/

    /**
     * Time-slicer for WfmRef sweep decimation
     */
    cfg_timeslicer(TIMESLICER_WFMREF, WFMREF_DECIMATION);

    /**
     * Time-slicer for SamplesBuffer
     */
    cfg_timeslicer(TIMESLICER_BUFFER, BUFFER_DECIMATION);

    /**
     * Time-slicer for IGBT current share controller
     */
    cfg_timeslicer(TIMESLICER_I_SHARE_CONTROLLER, I_SHARE_CONTROLLER_DECIMATION);

    /**
     * Samples buffer initialization
     */
    init_buffer(BUF_SAMPLES, &g_buf_samples_ctom, SIZE_BUF_SAMPLES_CTOM);
    enable_buffer(BUF_SAMPLES);

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

    reset_timeslicers();
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

    PieCtrlRegs.PIEACK.all |= M_INT3;
}

/**
 * Control ISR
 */
interrupt void isr_controller(void)
{
    static float temp[4];
    static uint16_t i;

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

    I_LOAD_1 = temp[0];
    I_LOAD_2 = temp[1];
    V_DCLINK = temp[2];

    I_LOAD_MEAN = 0.5*(I_LOAD_1 + I_LOAD_2);
    I_LOAD_DIFF = I_LOAD_1 - I_LOAD_2;
    I_IGBTS_DIFF = I_IGBT_1 - I_IGBT_2;

    /// Run low-pass filter for DC-Link voltage
    run_dsp_iir_2p2z(IIR_2P2Z_LPF_V_DCLINK);

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
            {
                switch(WFMREF.sync_mode)
                {
                    case OneShot:
                    {   /*********************************************/
                        RUN_TIMESLICER(TIMESLICER_WFMREF)
                            if( WFMREF.wfmref_data.p_buf_idx <=
                                WFMREF.wfmref_data.p_buf_end)
                            {
                                I_LOAD_REFERENCE =
                                        *(WFMREF.wfmref_data.p_buf_idx++) *
                                        (WFMREF.gain) + WFMREF.offset;
                            }
                        END_TIMESLICER(TIMESLICER_WFMREF)
                        /*********************************************/
                        break;
                    }

                    case SampleBySample:
                    case SampleBySample_OneCycle:
                    {
                        if(WFMREF.wfmref_data.p_buf_idx <= WFMREF.wfmref_data.p_buf_end)
                        {
                            I_LOAD_REFERENCE =  *(WFMREF.wfmref_data.p_buf_idx) *
                                                 (WFMREF.gain) + WFMREF.offset;
                        }
                        break;
                    }
                }
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
            SATURATE(I_LOAD_REFERENCE, MAX_REF_OL, MIN_REF_OL);
            DUTY_CYCLE_IGBT_1 = 0.01 * I_LOAD_REFERENCE;
            SATURATE(DUTY_CYCLE_IGBT_1, PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);

            DUTY_CYCLE_IGBT_2 = DUTY_CYCLE_IGBT_1;
        }
        /// Closed-loop
        else
        {
            SATURATE(I_LOAD_REFERENCE, MAX_REF, MIN_REF);
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

    /*********************************************/
    RUN_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/
        insert_buffer(BUF_SAMPLES, NETSIGNAL_CTOM_BUF);
        insert_buffer(BUF_SAMPLES, NETSIGNAL_MTOC_BUF);
    /*********************************************/
    END_TIMESLICER(TIMESLICER_BUFFER)
    /*********************************************/

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
        if(V_DCLINK < MIN_V_DCLINK)
        {
            set_hard_interlock(V_DCLINK_UNDERVOLTAGE);
        }

        #ifdef USE_ITLK
        else
        {
        #endif

            PIN_CLOSE_DCLINK_CONTACTOR;
            DELAY_US(TIMEOUT_DCLINK_CONTACTOR_CLOSED_MS*1000);

            if(!PIN_STATUS_DCLINK_CONTACTOR)
            {
                set_hard_interlock(DCLINK_CONTACTOR_FAIL);
            }

            #ifdef USE_ITLK
            else
            {
            #endif

                reset_controller();

                g_ipc_ctom.ps_module[0].ps_status.bit.openloop = OPEN_LOOP;
                g_ipc_ctom.ps_module[0].ps_status.bit.state = SlowRef;
                enable_pwm_output(0);
                enable_pwm_output(1);

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

    PIN_OPEN_DCLINK_CONTACTOR;;
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

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Off;
    }
}

/**
 * Set specified hard interlock for specified power supply.
 *
 * @param itlk specified hard interlock
 */
static void set_hard_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |= itlk;
    }
}

/**
 * Set specified soft interlock for specified power supply.
 *
 * @param itlk specified soft interlock
 */
static void set_soft_interlock(uint32_t itlk)
{
    if(!(g_ipc_ctom.ps_module[0].ps_soft_interlock & itlk))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_soft_interlock |= itlk;
    }
}

/**
 * ISR for MtoC hard interlock request.
 */
static interrupt void isr_hard_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[0].ps_hard_interlock &
         g_ipc_mtoc.ps_module[0].ps_hard_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_hard_interlock |=
        g_ipc_mtoc.ps_module[0].ps_hard_interlock;
    }
}

/**
 * ISR for MtoC soft interlock request.
 */
static interrupt void isr_soft_interlock(void)
{
    if(!(g_ipc_ctom.ps_module[0].ps_soft_interlock &
         g_ipc_mtoc.ps_module[0].ps_soft_interlock))
    {
        #ifdef USE_ITLK
        turn_off(0);
        g_ipc_ctom.ps_module[0].ps_status.bit.state = Interlock;
        #endif

        g_ipc_ctom.ps_module[0].ps_soft_interlock |=
        g_ipc_mtoc.ps_module[0].ps_soft_interlock;
    }
}

/**
 * Check interlocks of this specific power supply topology
 */
static inline void check_interlocks(void)
{
    if(fabs(I_LOAD_MEAN) > MAX_ILOAD)
    {
        set_hard_interlock(LOAD_OVERCURRENT);
    }

    if(fabs(I_IGBT_1) > MAX_I_IGBT)
    {
        set_hard_interlock(IGBT_1_OVERCURRENT);
    }

    if(fabs(I_IGBT_2) > MAX_I_IGBT)
    {
        set_hard_interlock(IGBT_2_OVERCURRENT);
    }

    if(fabs(I_LOAD_DIFF) > MAX_DCCTS_DIFF)
    {
        set_soft_interlock(DCCTS_HIGH_DIFFERENCE);
    }

    if(fabs(I_IGBTS_DIFF) > MAX_IGBT_DIFF)
    {
        set_soft_interlock(IGBTS_HIGH_DIFFERENCE);
    }

    if(fabs(V_DCLINK) > MAX_V_DCLINK)
    {
        set_hard_interlock(V_DCLINK_OVERVOLTAGE);
    }

    if(!PIN_STATUS_DCCT_1_STATUS)
    {
        set_soft_interlock(DCCT_1_FAULT);
    }

    if(!PIN_STATUS_DCCT_2_STATUS)
    {
        set_soft_interlock(DCCT_2_FAULT);
    }

    if(PIN_STATUS_DCCT_1_ACTIVE)
    {
        if(fabs(I_LOAD_1) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(I_LOAD_1_FEEDBACK_FAULT);
        }
    }
    else
    {
        if(fabs(I_LOAD_1) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(I_LOAD_1_FEEDBACK_FAULT);
        }
    }

    if(PIN_STATUS_DCCT_2_ACTIVE)
    {
        if(fabs(I_LOAD_2) < MIN_I_ACTIVE_DCCT)
        {
            set_soft_interlock(I_LOAD_2_FEEDBACK_FAULT);
        }
    }
    else
    {
        if(fabs(I_LOAD_2) > MAX_I_IDLE_DCCT)
        {
            set_soft_interlock(I_LOAD_2_FEEDBACK_FAULT);
        }
    }

    DINT;

    if(g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock)
    {
        if(PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(DCLINK_CONTACTOR_FAIL);
        }
    }
    else
    {
        if(!PIN_STATUS_DCLINK_CONTACTOR)
        {
            set_hard_interlock(DCLINK_CONTACTOR_FAIL);
        }

        if(V_DCLINK < MIN_V_DCLINK)
        {
            set_hard_interlock(V_DCLINK_UNDERVOLTAGE);
        }
    }

    EINT;
}

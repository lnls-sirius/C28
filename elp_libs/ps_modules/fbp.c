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
 * @file fbp.c
 * @brief FBP v4.0 module
 *
 * Module for control of FBP v4.0 power supplies (Low-Power Power Supply).
 *
 * @author gabriel.brunheira
 * @date 23/11/2017
 *
 */

#include <float.h>

#include "boards/udc_c28.h"
#include "common/timeslicer.h"
#include "control/control.h"
#include "event_manager/event_manager.h"
#include "HRADC_board/HRADC_Boards.h"
#include "ipc/ipc.h"
#include "parameters/parameters.h"
#include "pwm/pwm.h"

#include "fbp.h"

/**
 * Analog variables parameters
 */
#define MAX_ILOAD(id)           ANALOG_VARS_MAX[0+id]
#define MAX_VLOAD(id)           ANALOG_VARS_MAX[4+id]
#define MIN_DCLINK(id)          ANALOG_VARS_MIN[8+id]
#define MAX_DCLINK(id)          ANALOG_VARS_MAX[8+id]
#define MAX_TEMP(id)            ANALOG_VARS_MAX[12+id]
#define MAX_OUTPUT_CHANGE(id)   ANALOG_VARS_MAX[16+id]

/**
 * All power supplies defines
 */
#define DECIMATION_FACTOR       1//(HRADC_FREQ_SAMP/ISR_CONTROL_FREQ)

#define SIGGEN                  SIGGEN_CTOM
#define SIGGEN_OUTPUT           g_controller_ctom.net_signals[12].f

#define WFMREF                  g_ipc_ctom.wfmref
#define WFMREF_OUTPUT           g_controller_ctom.net_signals[13].f

#define PS_SETPOINT(i)          g_ipc_ctom.ps_module[i].ps_setpoint
#define PS_REFERENCE(i)         g_ipc_ctom.ps_module[i].ps_reference

/**
 * Power supply 1 defines
 */
#define PS1_ID                          0x0000

#define PIN_OPEN_PS1_DCLINK_RELAY       CLEAR_GPDO4;
#define PIN_CLOSE_PS1_DCLINK_RELAY      SET_GPDO4;

#define PIN_STATUS_PS1_DCLINK_RELAY     GET_GPDI4
#define PIN_STATUS_PS1_DRIVER_ERROR     GET_GPDI5
#define PIN_STATUS_PS1_FUSE             GET_GPDI14

#define PS1_LOAD_CURRENT                g_controller_ctom.net_signals[0].f  // HRADC0
#define PS1_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[0].f  // ANI2
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[4].f  // ANI6
#define PS1_TEMPERATURE                 g_controller_mtoc.net_signals[8].f  // I2C Add 0x48

#define PS1_SETPOINT                    g_ipc_ctom.ps_module[0].ps_setpoint
#define PS1_REFERENCE                   g_ipc_ctom.ps_module[0].ps_reference

#define ERROR_CALCULATOR_PS1            &g_controller_ctom.dsp_modules.dsp_error[0]
#define PI_CONTROLLER_ILOAD_PS1         &g_controller_ctom.dsp_modules.dsp_pi[0]
#define PI_CONTROLLER_ILOAD_PS1_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[0].coeffs.s

#define PS1_KP                          PI_CONTROLLER_ILOAD_PS1_COEFFS.kp
#define PS1_KI                          PI_CONTROLLER_ILOAD_PS1_COEFFS.ki

#define PS1_PWM_MODULATOR               g_pwm_modules.pwm_regs[0]
#define PS1_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[1]

#define PS1_SCOPE                       SCOPE_CTOM[0]

/**
 * Power supply 2 defines
 */
#define PS2_ID                          0x0001

#define PIN_OPEN_PS2_DCLINK_RELAY       CLEAR_GPDO3;
#define PIN_CLOSE_PS2_DCLINK_RELAY      SET_GPDO3;

#define PIN_STATUS_PS2_DCLINK_RELAY     GET_GPDI11
#define PIN_STATUS_PS2_DRIVER_ERROR     GET_GPDI9
#define PIN_STATUS_PS2_FUSE             GET_GPDI16

#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[1].f  // HRADC1
#define PS2_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[1].f  // ANI1
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[5].f  // ANI7
#define PS2_TEMPERATURE                 g_controller_mtoc.net_signals[9].f  // I2C Add 0x49

#define PS2_SETPOINT                    g_ipc_ctom.ps_module[1].ps_setpoint
#define PS2_REFERENCE                   g_ipc_ctom.ps_module[1].ps_reference

#define ERROR_CALCULATOR_PS2            &g_controller_ctom.dsp_modules.dsp_error[1]
#define PI_CONTROLLER_ILOAD_PS2         &g_controller_ctom.dsp_modules.dsp_pi[1]
#define PI_CONTROLLER_ILOAD_PS2_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[1].coeffs.s

#define PS2_KP                          PI_CONTROLLER_ILOAD_PS2_COEFFS.kp
#define PS2_KI                          PI_CONTROLLER_ILOAD_PS2_COEFFS.ki

#define PS2_PWM_MODULATOR               g_pwm_modules.pwm_regs[2]
#define PS2_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[3]

#define PS2_SCOPE                       SCOPE_CTOM[1]

/**
 * Power supply 3 defines
 */
#define PS3_ID                          0x0002

#define PIN_OPEN_PS3_DCLINK_RELAY       CLEAR_GPDO1;
#define PIN_CLOSE_PS3_DCLINK_RELAY      SET_GPDO1;

#define PIN_STATUS_PS3_DCLINK_RELAY     GET_GPDI8
#define PIN_STATUS_PS3_DRIVER_ERROR     GET_GPDI1
#define PIN_STATUS_PS3_FUSE             GET_GPDI13

#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[2].f  // HRADC2
#define PS3_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[2].f  // ANI4
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[6].f  // ANI3
#define PS3_TEMPERATURE                 g_controller_mtoc.net_signals[10].f // I2C Add 0x4A

#define PS3_SETPOINT                    g_ipc_ctom.ps_module[2].ps_setpoint
#define PS3_REFERENCE                   g_ipc_ctom.ps_module[2].ps_reference

#define ERROR_CALCULATOR_PS3            &g_controller_ctom.dsp_modules.dsp_error[2]
#define PI_CONTROLLER_ILOAD_PS3         &g_controller_ctom.dsp_modules.dsp_pi[2]
#define PI_CONTROLLER_ILOAD_PS3_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[2].coeffs.s

#define PS3_KP                          PI_CONTROLLER_ILOAD_PS3_COEFFS.kp
#define PS3_KI                          PI_CONTROLLER_ILOAD_PS3_COEFFS.ki

#define PS3_PWM_MODULATOR               g_pwm_modules.pwm_regs[4]
#define PS3_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[5]

#define PS3_SCOPE                       SCOPE_CTOM[2]

/**
 * Power supply 4 defines
 */
#define PS4_ID                          0x0003

#define PIN_OPEN_PS4_DCLINK_RELAY       CLEAR_GPDO2;
#define PIN_CLOSE_PS4_DCLINK_RELAY      SET_GPDO2;

#define PIN_STATUS_PS4_DCLINK_RELAY     GET_GPDI2
#define PIN_STATUS_PS4_DRIVER_ERROR     GET_GPDI3
#define PIN_STATUS_PS4_FUSE             GET_GPDI15

#define PS4_LOAD_CURRENT                g_controller_ctom.net_signals[3].f  // HRADC3
#define PS4_DCLINK_VOLTAGE              g_controller_mtoc.net_signals[3].f  // ANI0
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[7].f  // ANI5
#define PS4_TEMPERATURE                 g_controller_mtoc.net_signals[11].f // I2C Add 0x4C

#define PS4_SETPOINT                    g_ipc_ctom.ps_module[3].ps_setpoint
#define PS4_REFERENCE                   g_ipc_ctom.ps_module[3].ps_reference

#define ERROR_CALCULATOR_PS4            &g_controller_ctom.dsp_modules.dsp_error[3]
#define PI_CONTROLLER_ILOAD_PS4         &g_controller_ctom.dsp_modules.dsp_pi[3]
#define PI_CONTROLLER_ILOAD_PS4_COEFFS  g_controller_mtoc.dsp_modules.dsp_pi[3].coeffs.s

#define PS4_KP                          PI_CONTROLLER_ILOAD_PS4_COEFFS.kp
#define PS4_KI                          PI_CONTROLLER_ILOAD_PS4_COEFFS.ki

#define PS4_PWM_MODULATOR               g_pwm_modules.pwm_regs[6]
#define PS4_PWM_MODULATOR_NEG           g_pwm_modules.pwm_regs[7]

#define PS4_SCOPE                       SCOPE_CTOM[3]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    Opened_Relay_Fault,
    DCLink_Fuse_Fault,
    MOSFETs_Driver_Fault,
    Welded_Relay_Fault
} hard_interlocks_t;

typedef enum
{
    Heatsink_Overtemperature
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

#define NUM_HARD_INTERLOCKS             MOSFETs_Driver_Fault + 1
#define NUM_SOFT_INTERLOCKS             Heatsink_Overtemperature + 1

#define ISR_FREQ_INTERLOCK_TIMEBASE     5000.0

/**
 * Private functions
 */
#pragma CODE_SECTION(isr_init_controller, "ramfuncs");
#pragma CODE_SECTION(isr_controller, "ramfuncs");
#pragma CODE_SECTION(turn_off, "ramfuncs");
#pragma CODE_SECTION(open_relay, "ramfuncs");

static void init_peripherals_drivers(void);
static void term_peripherals_drivers(void);

static void init_controller(void);
static void reset_controller(uint16_t id);
static void reset_controllers(void);
static void enable_controller();
static void disable_controller();
static interrupt void isr_init_controller(void);
static interrupt void isr_controller(void);

static void init_interruptions(void);
static void term_interruptions(void);

static void turn_on(uint16_t id);
static void turn_off(uint16_t id);

static void open_relay(uint16_t id);
static void close_relay(uint16_t id);

static void reset_interlocks(uint16_t id);
static void check_interlocks_ps_module(uint16_t id);

static inline void run_dsp_pi_inline(dsp_pi_t *p_pi);
static inline void set_pwm_duty_hbridge_inline(volatile struct EPWM_REGS
                                               *p_pwm_module, float duty_pu);
static inline uint16_t insert_buffer_inline(buf_t *p_buf, float data);

/**
 * Main function for this power supply module
 */
void main_fbp(void)
{
    uint16_t i;

    init_controller();
    init_peripherals_drivers();
    init_interruptions();
    enable_controller();

    /// TODO: check why first sync_pulse occurs
    g_ipc_ctom.counter_sync_pulse = 0;

    /// TODO: include condition for re-initialization
    while(1)
    {
        for(i = 0; i < NUM_MAX_PS_MODULES; i++)
        {
            if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
            {
                check_interlocks_ps_module(i);
            }
        }
    }

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        turn_off(i);
    }

    disable_controller();
    term_interruptions();
    reset_controllers();
    term_peripherals_drivers();
}

static void init_peripherals_drivers(void)
{
    uint16_t i;

    /// Initialization of HRADC boards
    stop_DMA();

    HRADCs_Info.enable_Sampling = 0;

    Init_DMA_McBSP_nBuffers(NUM_PS_MODULES, DECIMATION_FACTOR, HRADC_SPI_CLK);

    Init_SPIMaster_McBSP(HRADC_SPI_CLK);
    Init_SPIMaster_Gpio();
    InitMcbspa20bit();

    DELAY_US(500000);
    send_ipc_lowpriority_msg(0,Enable_HRADC_Boards);
    DELAY_US(2000000);

    for(i = 0; i < NUM_PS_MODULES; i++)
    {
        Init_HRADC_Info(&HRADCs_Info.HRADC_boards[i], i, DECIMATION_FACTOR,
                        buffers_HRADC[i], TRANSDUCER_GAIN[i]);
        Config_HRADC_board(&HRADCs_Info.HRADC_boards[i], TRANSDUCER_OUTPUT_TYPE[i],
                           HRADC_HEATER_ENABLE[i], HRADC_MONITOR_ENABLE[i]);
    }

    HRADCs_Info.n_HRADC_boards = NUM_PS_MODULES;

    Config_HRADC_SoC(HRADC_FREQ_SAMP);

    /// Initialization of PWM modules
    g_pwm_modules.num_modules = 8;

    PS1_PWM_MODULATOR       = &EPwm7Regs;   // PS-1 Positive polarity switches
    PS1_PWM_MODULATOR_NEG   = &EPwm8Regs;   // PS-1 Negative polarity switches

    PS2_PWM_MODULATOR       = &EPwm5Regs;   // PS-2 Positive polarity switches
    PS2_PWM_MODULATOR_NEG   = &EPwm6Regs;   // PS-2 Negative polarity switches

    PS3_PWM_MODULATOR       = &EPwm3Regs;   // PS-3 Positive polarity switches
    PS3_PWM_MODULATOR_NEG   = &EPwm4Regs;   // PS-3 Negative polarity switches

    PS4_PWM_MODULATOR       = &EPwm1Regs;   // PS-4 Positive polarity switches
    PS4_PWM_MODULATOR_NEG   = &EPwm2Regs;   // PS-4 Negative polarity switches

    disable_pwm_outputs();
    disable_pwm_tbclk();
    init_pwm_mep_sfo();

    /// PS-4 PWM initialization
    init_pwm_module(PS4_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Master, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS4_PWM_MODULATOR_NEG, PWM_FREQ, 1, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-3 PWM initialization
    init_pwm_module(PS3_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS3_PWM_MODULATOR_NEG, PWM_FREQ, 3, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-2 PWM initialization
    init_pwm_module(PS2_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS2_PWM_MODULATOR_NEG, PWM_FREQ, 5, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

    /// PS-1 PWM initialization
    init_pwm_module(PS1_PWM_MODULATOR, PWM_FREQ, 0, PWM_Sync_Slave, 0,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);
    init_pwm_module(PS1_PWM_MODULATOR_NEG, PWM_FREQ, 7, PWM_Sync_Slave, 180,
                    PWM_ChB_Complementary, PWM_DEAD_TIME);

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
    ConfigCpuTimer(&CpuTimer0, C28_FREQ_MHZ,
                   (1000000.0/ISR_FREQ_INTERLOCK_TIMEBASE));
    CpuTimer0Regs.TCR.bit.TIE = 0;
}

static void term_peripherals_drivers(void)
{
}

static void init_controller(void)
{
    static uint16_t i;

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        init_ps_module(&g_ipc_ctom.ps_module[i],
                       g_ipc_mtoc.ps_module[i].ps_status.bit.model,
                       &turn_on, &turn_off, &isr_soft_interlock,
                       &isr_hard_interlock, &reset_interlocks);

        init_event_manager(i, ISR_FREQ_INTERLOCK_TIMEBASE,
                           NUM_HARD_INTERLOCKS, NUM_SOFT_INTERLOCKS,
                           &HARD_INTERLOCKS_DEBOUNCE_TIME,
                           &HARD_INTERLOCKS_RESET_TIME,
                           &SOFT_INTERLOCKS_DEBOUNCE_TIME,
                           &SOFT_INTERLOCKS_RESET_TIME);

        if(!g_ipc_mtoc.ps_module[i].ps_status.bit.active)
        {
            g_ipc_ctom.ps_module[i].ps_status.bit.active = 0;
        }

        init_wfmref(&WFMREF[i], WFMREF_SELECTED_PARAM[i], WFMREF_SYNC_MODE_PARAM[i],
                    ISR_CONTROL_FREQ, WFMREF_FREQUENCY_PARAM[i], WFMREF_GAIN_PARAM[i],
                    WFMREF_OFFSET_PARAM[i], &g_wfmref_data.data_fbp[i],
                    SIZE_WFMREF_FBP, &PS_REFERENCE(i));

        init_scope(&SCOPE_CTOM[i], ISR_CONTROL_FREQ,
                   SCOPE_FREQ_SAMPLING_PARAM[i],
                   &g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM * i / NUM_MAX_PS_MODULES],
                   SIZE_BUF_SAMPLES_CTOM / NUM_MAX_PS_MODULES,
                   SCOPE_SOURCE_PARAM[i], &run_scope_shared_ram);

        /// Initialization of signal generator module
        disable_siggen(&SIGGEN[i]);

        init_siggen(&SIGGEN[i], ISR_CONTROL_FREQ, &PS_REFERENCE(i));

        cfg_siggen(&SIGGEN[i], SIGGEN_TYPE_PARAM, SIGGEN_NUM_CYCLES_PARAM,
                   SIGGEN_FREQ_PARAM, SIGGEN_AMP_PARAM,
                   SIGGEN_OFFSET_PARAM, SIGGEN_AUX_PARAM);
    }

    init_control_framework(&g_controller_ctom);

    init_ipc();

    /**
     * TODO: initialize WfmRef and Samples Buffer
     */

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY

    /**
     *        name:     ERROR_CALCULATOR_PS1
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[0].ps_reference
     *           -:     net_signals[0]
     *         out:     net_signals[4]
     */

    init_dsp_error(ERROR_CALCULATOR_PS1, &PS1_REFERENCE, &PS1_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[4].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS1
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[4]
     *         out:     output_signals[0]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS1, PS1_KP, PS1_KI, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[4].f,
                &g_controller_ctom.output_signals[0].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 2

    /**
     *        name:     ERROR_CALCULATOR_PS2
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[1].ps_reference
     *           -:     net_signals[1]
     *         out:     net_signals[5]
     */

    init_dsp_error(ERROR_CALCULATOR_PS2, &PS2_REFERENCE, &PS2_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[5].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS2
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[5]
     *         out:     output_signals[1]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS2, PS2_KP, PS2_KI, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[5].f,
                &g_controller_ctom.output_signals[1].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 3

    /**
     *        name:     ERROR_CALCULATOR_PS3
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[2].ps_reference
     *           -:     net_signals[2]
     *         out:     net_signals[6]
     */

    init_dsp_error(ERROR_CALCULATOR_PS3, &PS3_REFERENCE, &PS3_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[6].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS3
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[6]
     *         out:     output_signals[2]
     */

    init_dsp_pi(PI_CONTROLLER_ILOAD_PS3, PS3_KP, PS3_KI, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[6].f,
                &g_controller_ctom.output_signals[2].f);

    /// INITIALIZATION OF LOAD CURRENT CONTROL LOOP FOR POWER SUPPLY 4

    /**
     *        name:     ERROR_CALCULATOR_PS4
     * description:     Load current reference error
     *  dsp module:     DSP_Error
     *           +:     ps_module[3].ps_reference
     *           -:     net_signals[3]
     *         out:     net_signals[7]
     */

    init_dsp_error(ERROR_CALCULATOR_PS4, &PS4_REFERENCE, &PS4_LOAD_CURRENT,
                   &g_controller_ctom.net_signals[7].f);

    /**
     *        name:     PI_CONTROLLER_ILOAD_PS4
     * description:     Load current PI controller
     *  dsp module:     DSP_PI
     *          in:     net_signals[7]
     *         out:     output_signals[3]
     */
    init_dsp_pi(PI_CONTROLLER_ILOAD_PS4, PS4_KP, PS4_KI, ISR_CONTROL_FREQ,
                PWM_MAX_DUTY, PWM_MIN_DUTY, &g_controller_ctom.net_signals[7].f,
                &g_controller_ctom.output_signals[3].f);

    /// Reset all internal variables
    reset_controllers();
}

/**
 * Reset all internal variables for controller of specified power supply
 *
 * @param id specified power supply
 */
static void reset_controller(uint16_t id)
{
    set_pwm_duty_hbridge(g_pwm_modules.pwm_regs[id*2], 0.0);

    g_ipc_ctom.ps_module[id].ps_status.bit.openloop = LOOP_STATE;

    PS_SETPOINT(id) = 0.0;
    PS_REFERENCE(id) = 0.0;

    reset_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[id]);
    reset_dsp_pi(&g_controller_ctom.dsp_modules.dsp_pi[id]);

    reset_wfmref(&WFMREF[id]);

    disable_siggen(&SIGGEN[id]);
}

/**
 * Reset all internal variables for all active power supplies
 */
static void reset_controllers(void)
{
    uint16_t i;

    for(i = 0; i < NUM_PS_MODULES; i++)
    {
        reset_controller(i);
    }
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

    /// Enable interlocks time-base timer
    CpuTimer0Regs.TCR.all = 0x4000;
}

/**
 * Disable control ISR
 */
static void disable_controller()
{
    disable_pwm_tbclk();
    HRADCs_Info.enable_Sampling = 0;
    stop_DMA();

    reset_controllers();
}

/**
 * ISR for control initialization
 */
static interrupt void isr_init_controller(void)
{
    uint16_t i;

    EALLOW;
    PieVectTable.EPWM1_INT = &isr_controller;
    EDIS;

    for(i = 0; i < g_pwm_modules.num_modules; i++)
    {
        g_pwm_modules.pwm_regs[i]->ETSEL.bit.INTSEL = ET_CTR_ZERO;
        g_pwm_modules.pwm_regs[i]->ETCLR.bit.INT = 1;
    }

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
    static uint16_t i;
    static float temp[4], temp_old[4];

    //SET_DEBUG_GPIO0;
    SET_DEBUG_GPIO1;

    /// Get HRADC samples
    temp[0] = (float) *(HRADCs_Info.HRADC_boards[0].SamplesBuffer);
    temp[1] = (float) *(HRADCs_Info.HRADC_boards[1].SamplesBuffer);
    temp[2] = (float) *(HRADCs_Info.HRADC_boards[2].SamplesBuffer);
    temp[3] = (float) *(HRADCs_Info.HRADC_boards[3].SamplesBuffer);

    temp[0] *= HRADCs_Info.HRADC_boards[0].gain;
    temp[0] += HRADCs_Info.HRADC_boards[0].offset;

    temp[1] *= HRADCs_Info.HRADC_boards[1].gain;
    temp[1] += HRADCs_Info.HRADC_boards[1].offset;

    temp[2] *= HRADCs_Info.HRADC_boards[2].gain;
    temp[2] += HRADCs_Info.HRADC_boards[2].offset;

    temp[3] *= HRADCs_Info.HRADC_boards[3].gain;
    temp[3] += HRADCs_Info.HRADC_boards[3].offset;

    temp_old[0] = PS1_LOAD_CURRENT;
    temp_old[1] = PS2_LOAD_CURRENT;
    temp_old[2] = PS3_LOAD_CURRENT;
    temp_old[3] = PS4_LOAD_CURRENT;

    PS1_LOAD_CURRENT = temp[0];
    PS2_LOAD_CURRENT = temp[1];
    PS3_LOAD_CURRENT = temp[2];
    PS4_LOAD_CURRENT = temp[3];

    /// Loop through active power supplies
    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        /// Check whether power supply is active
        if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
        {
            /// Check whether power supply is ON
            if(g_ipc_ctom.ps_module[i].ps_status.bit.state > Interlock)
            {
                /// Calculate reference according to operation mode
                switch(g_ipc_ctom.ps_module[i].ps_status.bit.state)
                {
                    case SlowRef:
                    case SlowRefSync:
                    {
                        PS_REFERENCE(i) = PS_SETPOINT(i);
                        break;
                    }
                    case RmpWfm:
                    case MigWfm:
                    {
                        run_wfmref(&WFMREF[i]);
                        break;
                    }
                    case Cycle:
                    {
                        SIGGEN[i].amplitude = SIGGEN_MTOC[i].amplitude;
                        SIGGEN[i].offset = SIGGEN_MTOC[i].offset;
                        SIGGEN[i].p_run_siggen(&SIGGEN[i]);

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

                /// Open-loop
                if(g_ipc_ctom.ps_module[i].ps_status.bit.openloop)
                {
                    g_controller_ctom.output_signals[i].f = 0.01 * PS_REFERENCE(i);

                    SATURATE(g_controller_ctom.output_signals[i].f,
                             PWM_MAX_DUTY_OL, PWM_MIN_DUTY_OL);
                }
                /// Closed-loop
                else
                {
                    SATURATE(PS_REFERENCE(i), MAX_REF[i], MIN_REF[i]);

                    //run_dsp_error(&g_controller_ctom.dsp_modules.dsp_error[i]);

                    // Update error only if current variation is within acceptable range
                    if (fabs(temp[i] - temp_old[i]) <= MAX_OUTPUT_CHANGE(i))
                    {
                        *g_controller_ctom.dsp_modules.dsp_error[i].error =
                            *g_controller_ctom.dsp_modules.dsp_error[i].pos -
                            *g_controller_ctom.dsp_modules.dsp_error[i].neg;
                    }

                    run_dsp_pi_inline(&g_controller_ctom.dsp_modules.dsp_pi[i]);

                    //SATURATE(g_controller_ctom.output_signals[i].f,
                    //         PWM_MAX_DUTY, PWM_MIN_DUTY);
                }

                set_pwm_duty_hbridge_inline(g_pwm_modules.pwm_regs[i*2],
                                     g_controller_ctom.output_signals[i].f);
            }
        }
    }

    RUN_SCOPE(PS1_SCOPE);
    RUN_SCOPE(PS2_SCOPE);
    RUN_SCOPE(PS3_SCOPE);
    RUN_SCOPE(PS4_SCOPE);

    /**
     * Re-enable external interrupt 2 (XINT2) interrupts to allow sync pulses to
     * be handled once per isr_controller
     */
    if(PieCtrlRegs.PIEIER1.bit.INTx5 == 0)
    {
        /// Set alarm if counter is below limit when receiving new sync pulse
        if(counter_sync_period < MIN_NUM_ISR_CONTROLLER_SYNC)
        {
            /// Loop through active power supplies to set alarm
           for(i = 0; i < NUM_MAX_PS_MODULES; i++)
           {
               if(g_ipc_ctom.ps_module[i].ps_status.bit.active)
               {
                   g_ipc_ctom.ps_module[i].ps_alarms = High_Sync_Input_Frequency;
               }
           }
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
    PS1_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS1_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS2_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS3_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR->ETCLR.bit.INT = 1;
    PS4_PWM_MODULATOR_NEG->ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all |= M_INT3;

    //CLEAR_DEBUG_GPIO0;
    CLEAR_DEBUG_GPIO1;
}

/**
 * Initialization of interruptions.
 */
static void init_interruptions(void)
{
    EALLOW;
    PieVectTable.EPWM1_INT = &isr_init_controller;
    //PieVectTable.EPWM2_INT = &isr_controller;
    PieVectTable.TINT0     = &isr_interlocks_timebase;
    EDIS;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  /// ePWM1
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  /// ePWM2
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  /// CpuTimer0

    enable_pwm_interrupt(PS4_PWM_MODULATOR);
    //enable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

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

    disable_pwm_interrupt(PS4_PWM_MODULATOR);
    disable_pwm_interrupt(PS4_PWM_MODULATOR_NEG);

    /// Clear flags
    PieCtrlRegs.PIEACK.all |= M_INT1 | M_INT3 | M_INT11;
}

/**
 * Turn on specified power supply.
 *
 * @param id specified power supply
 */
static void turn_on(uint16_t id)
{
    if(g_ipc_ctom.ps_module[id].ps_status.bit.active)
    {
        #ifdef USE_ITLK
        if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Off)
        #else
        if(g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock)
        #endif
        {
            if(fabs(g_controller_mtoc.net_signals[id].f) < MIN_DCLINK(id))
            {
                BYPASS_HARD_INTERLOCK_DEBOUNCE(id, DCLink_Undervoltage);
                set_hard_interlock(id, DCLink_Undervoltage);
            }

            switch(id)
            {
                case 0:
                {
                    if(!PIN_STATUS_PS1_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(0, DCLink_Fuse_Fault);
                        set_hard_interlock(0, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 1:
                {
                    if(!PIN_STATUS_PS2_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(1, DCLink_Fuse_Fault);
                        set_hard_interlock(1, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 2:
                {
                    if(!PIN_STATUS_PS3_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(2, DCLink_Fuse_Fault);
                        set_hard_interlock(2, DCLink_Fuse_Fault);
                    }
                    break;
                }

                case 3:
                {
                    if(!PIN_STATUS_PS4_FUSE)
                    {
                        BYPASS_HARD_INTERLOCK_DEBOUNCE(3, DCLink_Fuse_Fault);
                        set_hard_interlock(3, DCLink_Fuse_Fault);
                    }
                    break;
                }
            }

            #ifdef USE_ITLK
            if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Off)
            #else
            if(g_ipc_ctom.ps_module[id].ps_status.bit.state <= Interlock)
            #endif
            {
                close_relay(id);

                g_ipc_ctom.ps_module[id].ps_status.bit.state = SlowRef;

                enable_pwm_output(2*id);
                enable_pwm_output((2*id)+1);
            }

            else if(g_ipc_ctom.ps_module[id].ps_status.bit.state == Interlock)
            {
                reset_controller(id);
            }

        }
    }
}

/**
 * Turn off specified power supply.
 *
 * @param id specified power supply
 */
static void turn_off(uint16_t id)
{
    if(g_ipc_ctom.ps_module[id].ps_status.bit.active)
    {
        disable_pwm_output(2*id);
        disable_pwm_output((2*id)+1);

        open_relay(id);

        g_ipc_ctom.ps_module[id].ps_status.bit.openloop = OPEN_LOOP;
        if (g_ipc_ctom.ps_module[id].ps_status.bit.state != Interlock)
        {
            g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
        }
        reset_controller(id);
    }
}

/**
 * Reset interlocks for specified power supply.
 *
 * @param id specified power supply
 */
static void reset_interlocks(uint16_t id)
{
    g_ipc_ctom.ps_module[id].ps_hard_interlock = 0;
    g_ipc_ctom.ps_module[id].ps_soft_interlock = 0;
    g_ipc_ctom.ps_module[id].ps_alarms = 0;

    if(g_ipc_ctom.ps_module[id].ps_status.bit.state < Initializing)
    {
        g_ipc_ctom.ps_module[id].ps_status.bit.state = Off;
    }
}

/**
 * Open relay from specified power supply.
 *
 * @param id specified power supply
 */
static void open_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_OPEN_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_OPEN_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_OPEN_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_OPEN_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * Close relay from specified power supply.
 *
 * @param id specified power supply
 */
static void close_relay(uint16_t id)
{
    switch(id)
    {
        case PS1_ID:
        {
            PIN_CLOSE_PS1_DCLINK_RELAY;
            break;
        }

        case PS2_ID:
        {
            PIN_CLOSE_PS2_DCLINK_RELAY;
            break;
        }

        case PS3_ID:
        {
            PIN_CLOSE_PS3_DCLINK_RELAY;
            break;
        }

        case PS4_ID:
        {
            PIN_CLOSE_PS4_DCLINK_RELAY;
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * Check variables from specified power supply for interlocks
 *
 * @param id specified power supply
 */
static void check_interlocks_ps_module(uint16_t id)
{
    if(fabs(g_controller_ctom.net_signals[id].f) > MAX_ILOAD(id))
    {
        set_hard_interlock(id, Load_Overcurrent);
    }

    if(fabs(g_controller_mtoc.net_signals[id].f) > MAX_DCLINK(id))
    {
        set_hard_interlock(id, DCLink_Overvoltage);
    }

    if(fabs(g_controller_mtoc.net_signals[id+4].f) > MAX_VLOAD(id))
    {
        set_hard_interlock(id, Load_Overvoltage);
    }

    if(fabs(g_controller_mtoc.net_signals[id+8].f) > MAX_TEMP(id))
    {
        set_soft_interlock(id, Heatsink_Overtemperature);
    }

    switch(id)
    {
        case 0:
        {
            if(!PIN_STATUS_PS1_DRIVER_ERROR)
            {
                set_hard_interlock(0, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[0].ps_status.bit.state <= Interlock) &&
                 (PIN_STATUS_PS1_DCLINK_RELAY) )
            {
                set_hard_interlock(0, Welded_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[0].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_PS1_DCLINK_RELAY)
                {
                    set_hard_interlock(0, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS1_FUSE)
                {
                    set_hard_interlock(0, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[0].f) < MIN_DCLINK(0))
                {
                    set_hard_interlock(0, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 1:
        {
            if(!PIN_STATUS_PS2_DRIVER_ERROR)
            {
                set_hard_interlock(1, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[1].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS2_DCLINK_RELAY))
            {
                set_hard_interlock(1, Welded_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[1].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_PS2_DCLINK_RELAY)
                {
                    set_hard_interlock(1, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS2_FUSE)
                {
                    set_hard_interlock(1, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[1].f) < MIN_DCLINK(1))
                {
                    set_hard_interlock(1, DCLink_Undervoltage);
                }
            }

            break;
        }

        case 2:
        {
            if(!PIN_STATUS_PS3_DRIVER_ERROR)
            {
                set_hard_interlock(2, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[2].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS3_DCLINK_RELAY)) {
                set_hard_interlock(2, Welded_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[2].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_PS3_DCLINK_RELAY)
                {
                    set_hard_interlock(2, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS3_FUSE)
                {
                    set_hard_interlock(2, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[2].f) < MIN_DCLINK(2))
                {
                    set_hard_interlock(2, DCLink_Undervoltage);
                }

            }

            break;
        }

        case 3:
        {
            if(!PIN_STATUS_PS4_DRIVER_ERROR)
            {
                set_hard_interlock(3, MOSFETs_Driver_Fault);
            }

            IER &= ~M_INT11;

            if ( (g_ipc_ctom.ps_module[3].ps_status.bit.state <= Interlock) &&
                (PIN_STATUS_PS4_DCLINK_RELAY)) {
                set_hard_interlock(3, Welded_Relay_Fault);
            }

            else if (g_ipc_ctom.ps_module[3].ps_status.bit.state > Interlock)
            {
                if(!PIN_STATUS_PS4_DCLINK_RELAY)
                {
                    set_hard_interlock(3, Opened_Relay_Fault);
                }

                if(!PIN_STATUS_PS4_FUSE)
                {
                    set_hard_interlock(3, DCLink_Fuse_Fault);
                }

                if(fabs(g_controller_mtoc.net_signals[3].f) < MIN_DCLINK(3))
                {
                    set_hard_interlock(3, DCLink_Undervoltage);
                }
            }

            break;
        }
    }

    IER |= M_INT11;

    run_interlocks_debouncing(id);
}

static inline void run_dsp_pi_inline(dsp_pi_t *p_pi)
{
    float dyn_max;
    float dyn_min;
    float temp;

    temp = *(p_pi->in) * p_pi->coeffs.s.kp;
    SATURATE(temp, p_pi->coeffs.s.u_max, p_pi->coeffs.s.u_min);
    p_pi->u_prop = temp;

    dyn_max = (p_pi->coeffs.s.u_max - temp);
    dyn_min = (p_pi->coeffs.s.u_min - temp);

    temp = p_pi->u_int + *(p_pi->in) * p_pi->coeffs.s.ki;
    SATURATE(temp, dyn_max, dyn_min);
    p_pi->u_int = temp;

    *(p_pi->out) = p_pi->u_int + p_pi->u_prop;
}

static inline void set_pwm_duty_hbridge_inline(volatile struct EPWM_REGS
                                               *p_pwm_module, float duty_pu)
{
    uint16_t duty_int;
    uint16_t duty_frac;
    float duty;

    duty = (0.5 * duty_pu + 0.5) * (float)p_pwm_module->TBPRD;

    duty_int  = (uint16_t) duty;
    duty_frac = ((uint16_t) ((duty - (float)duty_int) * MEP_ScaleFactor)) << 8;
    duty_frac += 0x0180;

    p_pwm_module->CMPAM2.half.CMPA    = duty_int;
    p_pwm_module->CMPAM2.half.CMPAHR  = duty_frac;
}

static inline uint16_t insert_buffer_inline(buf_t *p_buf, float data)
{
    if( (p_buf->p_buf_idx >= p_buf->p_buf_start) &&
        (p_buf->p_buf_idx <= p_buf->p_buf_end) )
    {
        if(p_buf->status == Buffering)
        {
            *(p_buf->p_buf_idx) = data;

            if(p_buf->p_buf_idx++ == p_buf->p_buf_end)
            {
                p_buf->p_buf_idx = p_buf->p_buf_start;
            }
        }
        else if(p_buf->status == Postmortem)
        {
            *(p_buf->p_buf_idx) = data;

            if(p_buf->p_buf_idx++ == p_buf->p_buf_end)
            {
                p_buf->p_buf_idx = p_buf->p_buf_start;
                p_buf->status = Idle;
            }
        }
        else
        {
            p_buf->status = Idle;
        }
    }

    else
    {
        p_buf->p_buf_idx = p_buf->p_buf_start;
        p_buf->status = Idle;
    }

    return p_buf->status;
}

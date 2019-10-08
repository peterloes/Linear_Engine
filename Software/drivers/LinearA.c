/***************************************************************************//**
 * @file
 * @brief	Linear Actuator Control
 * @author	Ralf Gerhauser
 * @version	2017-10-14
 *
 * This module provides the functionality to control a linear actuator,
 * realized with a stepper motor and a spindle.  In detail, this includes:
 * - Initialization and configuration of the required devices.
 * - Pulse generation to control the stepper motor.
 * - A setup routine to adjust the end-points and speed of the linear actuator.
 * - EEPROM-like storage to save the configuration.
 *
 * The firmware supports soft-start/stop, i.e. the stepper speed will be
 * ramped-up during the start phase and ramped-down before stopping.
 * After power-up the position of the spindle is not known, therefore the
 * stepper moves it to its physical end-point.  This is then internally treated
 * as position 0.  If a switch is connected to pin @ref POS_ZERO_DETECT,
 * end point zero will be detected directly and the motor will be stopped
 * immediately.
 *
 * Important constants for this module are:
 * - @ref TOTAL_DIST_IN_MM defines the total length that can be moved.
 * - @ref SPEED_UP_DIST_IN_MM defines the way which is used for speed-up.
 * - @ref SLOW_DOWN_DIST_IN_MM defines the way to slow-down.
 * - @ref STEPPER_MIN_SPEED is the minimum speed value during move.
 * - @ref STEPPER_MAX_SPEED is the maximum speed the stepper can rotate.
 * - @ref SETUP_SPEED is the speed value used during setup of the pos 1 and 2.
 * - @ref CALIBRATION_SPEED is the speed value used during position zero
 *   calibration.
 *
 ****************************************************************************//*
Revision History:
2017-10-14,rage	Completed version 2.0.
2017-09-09,rage	Initial version, based on "Servo.c".
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdlib.h>
#include "config.h"	// include project configuration parameters

#if CFG_LINEAR_A  ||  DOXYGEN	// SMC11 stepper

#include "AlarmClock.h"
#include "LinearA.h"
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_int.h"
#include "em_prs.h"
#include "em_timer.h"
#include "eeprom_emulation.h"

/*=============================== Definitions ================================*/

/*!@brief Magic ID for EEPROM block */
#define MAGIC_ID	0x0817

/*!@brief Prescaler value for 16bit timer 2 (clock is 32MHz) */
#define TIMER2_PRESCALER	timerPrescale4	// 32MHz / {prescaler}

/*!@brief Number of pulses for total distance */
#define TOTAL_DIST_IN_PULSES	(TOTAL_DIST_IN_MM * PULSES_PER_MM)

/*!@brief Distance to speed up in number of pulses. */
#define SPEED_UP_DIST		(SPEED_UP_DIST_IN_MM * PULSES_PER_MM)

/*!@brief Distance to slow down in number of pulses. */
#define SLOW_DOWN_DIST	(SLOW_DOWN_DIST_IN_MM * PULSES_PER_MM)

/*!@brief Percentage of speed-up phase relativ to speed-up and slow-down phase */
#define PERCENT_SPEED_UP	(SPEED_UP_DIST * 100L) /	\
				(SPEED_UP_DIST + SLOW_DOWN_DIST)

/*!@brief Number of pulses to move per key hit during setup of end-points. */
#define SETUP_DELTA_POS		(PULSES_PER_MM * SETUP_DELTA_POS_MM)

/*!@brief Power-off delay in [20ms] portions. */
#define POWER_OFF_DELAY		50	// 50 * 20ms = 500ms = 0.5s

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Operating and Setup modes for the stepper. */
typedef enum
{
    OP_MODE_CALIBRATION = (-1),	//!< -1: Initial calibration of position zero
    OP_MODE_OPERATIONAL,	//!<  0: Normal operating mode
    OP_MODE_SETUP_POS_1,	//!<  1: Setup mode: adjust first position
    OP_MODE_SETUP_POS_2,	//!<  2: Setup mode: adjust second position
    OP_MODE_SETUP_SPEED,	//!<  3: Setup mode: adjust speed
    END_OP_MODE			//!<  End of operating mode definitions
} OP_MODE;

/*!@brief Drive control states. */
typedef enum
{
    DRV_CTRL_IDLE,		//!< 0: Idle, no movement
    DRV_CTRL_POWER_ON,		//!< 1: Power on the SMC11 module, set direction
    DRV_CTRL_ENABLE,		//!< 2: Enable stepper
    DRV_CTRL_MOVE,		//!< 3: Move
    DRV_CTRL_STOP,		//!< 4: Stop movement
    DRV_CTRL_DISABLE,		//!< 5: Disable stepper
    END_DRV_CTRL		//!< End of drive control definitions
} DRV_CTRL;

/*================================ Local Data ================================*/

    /*! Timer handle for key inactivity timer */
static TIM_HDL		l_thTimeout = NONE;

    /*! Flag shows if there are valid settings stored in flash. */
static volatile bool	l_LinearDataValid = false;

    /*! Actual linear actuator mode */
static volatile OP_MODE l_OpMode = OP_MODE_CALIBRATION;

    /*! Actual linear actuator drive control state */
static volatile DRV_CTRL l_DrvCtrl = DRV_CTRL_IDLE;

    /*! 20ms timer for drive control (power on/off) */
static volatile int	l_DrvCtrl_20msTimer;

    /*! Linear actuator start position */
static volatile long	l_StartPos;

    /*! Linear actuator target position */
static volatile long	l_TargetPos;

    /*! Next target position (after current move) */
static volatile long	l_NextTargetPos = NONE;

    /*! Linear actuator actual position */
static volatile long	l_ActualPos;

    /*! Position where speed-up phase ends */
static volatile long	l_SpeedUpEnd;

    /*! Position where slow-down phase starts */
static volatile long	l_SlowDownStart;

    /*! Linear actuator end-position 1 */
static volatile long	l_LinearPos_1 = 0;

    /*! Linear actuator end-position 2 */
static volatile long	l_LinearPos_2 = TOTAL_DIST_IN_PULSES;

    /*! Linear actuator nominal speed */
static volatile int	l_NominalSpeed = CALIBRATION_SPEED;

    /*! Maximum speed factor during this move */
static volatile int	l_maxSpeedFactor;

    /*! Stepper direction during speed calibration mode */
static volatile bool	l_DirIncrease;

    /*! Time counter for disabling the DC/DC power converter [20ms] */
static volatile int	l_PowerOffTimer;

    /*! Define the non-volatile variables. */
static EE_Variable_TypeDef  magic, pos1h, pos1l, pos2h, pos2l, speed, chksum;


/*=========================== Forward Declarations ===========================*/

static void	MoveToPos (long pos);
static void	SetSpeed (unsigned int speed);
static void	LinearKeyTimeout (TIM_HDL hdl);
static void	PowerEnable(void);
static void	PowerDisable(void);
static void	StepperEnable(bool startPulsing);
static void	StepperDisable(void);
static bool	ReadLinearParms(void);
static void	WriteLinearParms(void);

/***************************************************************************//**
 *
 * @brief	Linear actuator initialization
 *
 * This routine must be called once to initialize all the devices required to
 * control the linear actuator.  It also loads the configuration parameters
 * (end-points and speed) from FLASH.
 *
 ******************************************************************************/
void	LinearInit (void)
{
    /* PA3_SERVO_ENABLE, SERVO_VDD, high active */
    GPIO_PinModeSet(SERVO_ENABLE_PORT, SERVO_ENABLE_PIN, gpioModePushPull, 0);

    /*
     * Configure GPIOs for the stepper controller.  Disable stepper per default,
     * i.e. no current is supplied to the motor.  All signals are low-active.
     */
    GPIO_PinModeSet (EN_LINEAR_PORT,   EN_LINEAR_PIN,   gpioModePushPull, 1);
    GPIO_PinModeSet (DIR_LINEAR_PORT,  DIR_LINEAR_PIN,  gpioModePushPull, 1);
    GPIO_PinModeSet (ASTR_LINEAR_PORT, ASTR_LINEAR_PIN, gpioModePushPull, 1);

    /*
     * Configure GPIO input for position zero detection (low-active)
     */
    GPIO_PinModeSet (POS_ZERO_DETECT_PORT, POS_ZERO_DETECT_PIN,
		     gpioModeInputPull, 1);

    /*
     * Configure GPIOs for the trigger inputs.  The port pins must be
     * configured for input, and connected to the external interrupt (EXTI)
     * facility.  At this stage, the interrupts are not enabled, this is
     * done later by calling ExtIntInit().
     */
    GPIO_PinModeSet (TRIG_POS_1_PORT, TRIG_POS_1_PIN, gpioModeInputPull, 1);
    GPIO_IntConfig  (TRIG_POS_1_PORT, TRIG_POS_1_PIN, false, false, false);

    GPIO_PinModeSet (TRIG_POS_2_PORT, TRIG_POS_2_PIN, gpioModeInputPull, 1);
    GPIO_IntConfig  (TRIG_POS_2_PORT, TRIG_POS_2_PIN, false, false, false);

    /* Create inactivity timer */
    l_thTimeout = sTimerCreate (LinearKeyTimeout);

    /* Enables the flash controller for writing. */
    MSC_Init();

    /* Initialize the eeprom emulator using 3 pages. */
    if ( !EE_Init(3) )
    {
	/* If the initialization fails we have to take some measure
	 * to obtain a valid set of pages. In this example we simply
	 * format the pages
	 */
	EE_Format(3);
    }

    /* Declare variables (virtual addresses) */
    EE_DeclareVariable(&magic);
    EE_DeclareVariable(&pos1h);
    EE_DeclareVariable(&pos1l);
    EE_DeclareVariable(&pos2h);
    EE_DeclareVariable(&pos2l);
    EE_DeclareVariable(&speed);
    EE_DeclareVariable(&chksum);

    /* Get end-points from flash */
    if (ReadLinearParms())
    {
	l_LinearDataValid = true;
    }

    /*
     * We use timer 1 for the 50Hz time base (LED flashing and power control)
     * and timer 2 for the clock generation of the stepper.  Both timers run
     * independently, timer 1 generates overflow interrupts, CC1 output of
     * timer 2 controls the stepper.
     */
    /* Enable clock for GPIO module should already be done) */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Enable clock for TIMER1 module */
    CMU_ClockEnable(cmuClock_TIMER1, true);

    /* Reset TIMER1, set default values for registers */
    TIMER_Reset(TIMER1);

    /* Select timer parameters */
    static const TIMER_Init_TypeDef timer1Init =
    {
	.enable     = false,			// do not start yet
	.debugRun   = true,
	.prescale   = timerPrescale16,		// 32Mhz / 16 = 2Mhz
	.clkSel     = timerClkSelHFPerClk,
	.fallAction = timerInputActionNone,
	.riseAction = timerInputActionNone,
	.mode       = timerModeUp,
	.dmaClrAct  = false,
	.quadModeX4 = false,
	.oneShot    = false,
	.sync       = false,
    };

    /* Initialize timer */
    TIMER_Init(TIMER1, &timer1Init);

    /* Set top value for the up-counter (50Hz, i.e. 20ms) */
    TIMER_TopSet(TIMER1, 2000000 / 50);

    /*
     * Timer 2 is used to generate pulses with 50% duty cycle on port C pin 9.
     */

    /* Enable clock for TIMER2 module */
    CMU_ClockEnable(cmuClock_TIMER2, true);

    /* Reset TIMER2, set default values for registers */
    TIMER_Reset(TIMER2);

    /*
     * CC1 generates the output pulses for the stepper via GPIO pin PC9.  We
     * use a 16MHz clock to be able to vary the speed with a fine granularity.
     */
    static const TIMER_InitCC_TypeDef timer2_CC1Init =
    {
	.eventCtrl  = timerEventEveryEdge,	// not used
	.edge       = timerEdgeBoth,		// pass both edges to control block
	.prsSel     = timerPRSSELCh0,		// not used
	.cufoa      = timerOutputActionNone,	// N/A for PWM mode
	.cofoa      = timerOutputActionNone,	// N/A for PWM mode
	.cmoa       = timerOutputActionNone,	// N/A for PWM mode
	.mode       = timerCCModePWM,		// start with high, switch to low
	.filter     = false,			// N/A for output modes
	.prsInput   = false,			// not used
	.coist      = false,			// N/A for PWM mode
	.outInvert  = false,			// no output pin inversion
    };

    /* Configure CC channel 1 */
    TIMER_InitCC(TIMER2, 1, &timer2_CC1Init);

    /* Set CC1 location pin PC9 as output */
    GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 0);

    /* Route CC1 to location 2 and enable pin */
    TIMER2->ROUTE |= (TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC2);

    /* Set speed, i.e. pulse frequency value */
    SetSpeed (STEPPER_MIN_SPEED);

    /* Select timer parameters */
    static const TIMER_Init_TypeDef timer2Init =
    {
	.enable     = false,			// do not start yet
	.debugRun   = true,			// continue running during debug
	.prescale   = TIMER2_PRESCALER,		// 32Mhz / {prescaler}
	.clkSel     = timerClkSelHFPerClk,	// use 32Mhz clock
	.fallAction = timerInputActionNone,	// ignore falling edge
	.riseAction = timerInputActionNone,	// ignore rising edge
	.mode       = timerModeUp,		// up-counter for PWM
	.dmaClrAct  = false,			// no DMA
	.quadModeX4 = false,			// no X4 mode
	.oneShot    = false,			// free-run mode
	.sync       = false,			// no sync with other timers
    };

    /* Initialize timer */
    TIMER_Init(TIMER2, &timer2Init);

    /* Enable Overflow Interrupt of Timer 2 */
    TIMER_IntEnable(TIMER2, TIMER_IEN_OF);
    NVIC_EnableIRQ(TIMER2_IRQn);

    /* Enable Overflow Interrupt of Timer 1 */
    TIMER_IntEnable(TIMER1, TIMER_IEN_OF);
    NVIC_EnableIRQ(TIMER1_IRQn);

    /* Enable timer 1 */
    TIMER_Enable(TIMER1, true);

    /* Perform calibration */
    l_ActualPos = TOTAL_DIST_IN_PULSES;
    MoveToPos (0);		// calibration: move to position 0
}


/***************************************************************************//**
 *
 * @brief	Move to Position
 *
 * Setup and start movement to a new position.
 *
 ******************************************************************************/
static void	MoveToPos (long pos)
{
long	distance;


    EFM_ASSERT(0 <= pos  &&  pos <= TOTAL_DIST_IN_PULSES);

    if (pos == l_ActualPos)
	return;			// position reached already

    // TODO: check if a move is still in progress
#if 0
    if (l_DrvCtrl != DRV_CTRL_IDLE)
    {

    }
#endif

    /* be sure to reset timer */
    l_DrvCtrl_20msTimer = 0;

    /* switch power on */
    PowerEnable();

    /* set new target position */
    l_TargetPos = pos;

    /* Consider direction: increase or decrease position, set DIR pin */
    DIR_LINEAR = l_TargetPos > l_ActualPos ? DIR_LINEAR_POS_INC
					   : DIR_LINEAR_POS_DEC;

    /* Calculate distance and speed profile */
    l_StartPos = l_ActualPos;
    distance = labs(l_TargetPos - l_StartPos);

    if (distance < SPEED_UP_DIST + SLOW_DOWN_DIST)
    {
	l_SpeedUpEnd = l_SlowDownStart = (distance * PERCENT_SPEED_UP) / 100;
    }
    else
    {
	l_SpeedUpEnd = SPEED_UP_DIST;
	l_SlowDownStart = distance - SLOW_DOWN_DIST;
    }

    l_maxSpeedFactor = l_SpeedUpEnd * 1000L / SPEED_UP_DIST;

    /* update drive control state */
    l_DrvCtrl = DRV_CTRL_POWER_ON;

    /* set timer for delay after power-up */
    l_DrvCtrl_20msTimer = 5;	// 100ms
}


/***************************************************************************//**
 *
 * @brief	Set speed
 *
 * This routine sets the stepper speed, i.e. the pulse frequency value.
 * Timer 2 is 16bit, so 65536 is divided by the speed value to generate
 * the stepper pulses with the required frequency.  The prescaler define
 * TIMER2_PRESCALER may be changed to adapt the frequency range.
 *
 ******************************************************************************/
static void	SetSpeed (unsigned int speed)
{
    EFM_ASSERT(2 <= speed  &&  speed <= 65536);

    /* Set Top Value based on a 32MHz/TIMER2_PRESCALER clock */
    TIMER_TopSet(TIMER2, 65536 / speed);

    /* PWM signal should always be 50% duty cycle */
    TIMER_CompareSet(TIMER2, 1, TIMER_TopGet(TIMER2) / 2);
}


/***************************************************************************//**
 *
 * @brief	Timer 1 Interrupt Handler
 *
 * This is the interrupt handler for the timer 1 reload interrupt which occurs
 * every 20ms.  It is used as time base for power-on/off timing, and flashing
 * the LED.
 *
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
static uint32_t	cnt20ms;
static int	phase;

    /* increase 20ms counter */
    cnt20ms++;

    /* determine reason for this interrupt */
    if (TIMER_IntGet(TIMER1) & TIMER_IF_OF)
    {
	TIMER_IntClear(TIMER1, TIMER_IFC_OF);

	/*
	 * Another 20ms is over, handle the following conditions:
	 * - LED control
	 * - DC/DC power on/off
	 */

	/* LED control */
	if (l_OpMode == OP_MODE_CALIBRATION)
	{
	    POWER_LED = (cnt20ms & 0x04) ? 1:0;	// fast flashing
	}
	else if (l_OpMode != OP_MODE_OPERATIONAL)
	{
	    if ((cnt20ms & 0x0F) == 0)
	    {
		phase++;
		if (phase < 2 * (int)l_OpMode + 1)
		{
		    /* switch LED on or off */
		    POWER_LED = (phase & 0x01 ? 1 : 0);
		}
		else if (phase > 2 * (int)l_OpMode + 3)
		{
		    phase = 0;
		}
	    }
	}

	/* if in speed setup mode, move between the two end points */
	if (l_OpMode == OP_MODE_SETUP_SPEED)
	{
	    /* first check if stepper is still moving */
	    if (l_DrvCtrl == DRV_CTRL_IDLE)
	    {
		if (l_DirIncrease)
		{
		    long maxPos = l_LinearPos_2 > l_LinearPos_1 ?
				  l_LinearPos_2 : l_LinearPos_1;

		    MoveToPos(maxPos);
		    l_DirIncrease = false;	// for the next time
		}
		else
		{
		    long minPos = l_LinearPos_2 < l_LinearPos_1 ?
				  l_LinearPos_2 : l_LinearPos_1;

		    MoveToPos(minPos);
		    l_DirIncrease = true;	// for the next time
		}
	    }
	}

	/* check drive control timer */
	if (l_DrvCtrl_20msTimer > 0)
	{
	    if (--l_DrvCtrl_20msTimer == 0)
	    {
		/* check linear actuator drive state */
		switch (l_DrvCtrl)
		{
		    case DRV_CTRL_POWER_ON:	// power is on, enable stepper
			StepperEnable(false);	// but don't start the timer yet
			l_DrvCtrl_20msTimer = 5;	// wait another 100ms
			l_DrvCtrl = DRV_CTRL_ENABLE;	// update state
			break;

		    case DRV_CTRL_ENABLE:	// stepper is enabled
			SetSpeed (STEPPER_MIN_SPEED);	// set start speed
			l_DrvCtrl = DRV_CTRL_MOVE;	// update state
			TIMER_Enable(TIMER2, true);	// start timer
			break;

		    case DRV_CTRL_STOP:		// actuator has been stopped
			if (l_NextTargetPos >= 0L)
			{
			    // if there is a next target position, move to it
			    MoveToPos(l_NextTargetPos);
			    l_NextTargetPos = NONE;
			    break;
			}
			StepperDisable();	// disable stepper
			l_DrvCtrl = DRV_CTRL_IDLE;	// update state
			break;

		    default:	// nothing to do
			break;
		}
	    }
	}

	/* handle power-off timer */
	if (l_DrvCtrl == DRV_CTRL_IDLE  &&  l_PowerOffTimer > 0)
	{
	    if (--l_PowerOffTimer == 0)
		PowerDisable();
	}
    }	// end TIMER_IF_OF

    g_flgIRQ = true;		// Interrupt happened
}


/***************************************************************************//**
 *
 * @brief	Timer 2 Interrupt Handler
 *
 * This is the interrupt handler for the timer 2 reload interrupt.  It counts
 * the number of pulses supplied to the stepper, i.e. the moving distance.
 *
 ******************************************************************************/
void TIMER2_IRQHandler(void)
{
long	distance, speedFactor, speed;

    TIMER_IntClear(TIMER2, TIMER_IFC_OF);

    if (DIR_LINEAR == DIR_LINEAR_POS_DEC  &&  POS_ZERO_DETECT == 0)
    {
	/* position decrement (moving to 0) and position zero detected */
	l_ActualPos = l_TargetPos = 0;
    }

    if (l_ActualPos == l_TargetPos)
    {
	/* stop this timer (to stop the stepper motor) */
	TIMER_Enable(TIMER2, false);

	/* set timer for delay after stopping the linear actuator */
	l_DrvCtrl_20msTimer = 5;	// 100ms

	/* if calibration mode, change to setup or operational mode */
	if (l_OpMode == OP_MODE_CALIBRATION)
	{
	    if (l_LinearDataValid)
	    {
		l_OpMode = OP_MODE_OPERATIONAL;
		POWER_LED = 0;	// end of calibration, switch LED off
		l_NextTargetPos = l_LinearPos_1;
	    }
	    else
	    {
		/* activate setup mode */
		l_OpMode = OP_MODE_SETUP_POS_1;
		SetSpeed (SETUP_SPEED);		// moving slowly
	    }
	}

	/* update state */
	l_DrvCtrl = DRV_CTRL_STOP;
    }
    else
    {
	/* still moving */
	if (DIR_LINEAR == DIR_LINEAR_POS_INC)
	    l_ActualPos++;
	else
	    l_ActualPos--;

	/* no ramped movement during Pos1 and Pos2 setup */
	if (l_OpMode != OP_MODE_SETUP_POS_1
	&&  l_OpMode != OP_MODE_SETUP_POS_2)
	{
	    /* standard (ramped) moving, calculate speed */
	    distance = labs(l_ActualPos - l_StartPos);
	    if (distance < l_SpeedUpEnd)
	    {
		/* speed-up phase */
		speedFactor = (distance * 1000L) / SPEED_UP_DIST;
	    }
	    else if (distance < l_SlowDownStart)
	    {
		/* range of constant speed */
		speedFactor = 1000L;
	    }
	    else
	    {
		/* slow-down phase */
		speedFactor = l_maxSpeedFactor
			    - (distance - l_SlowDownStart) * 1000L / SLOW_DOWN_DIST;
	    }

	    /* calculate and limit speed value */
	    speed = STEPPER_MIN_SPEED + ((l_NominalSpeed - STEPPER_MIN_SPEED)
					* speedFactor / 1000L);

	    if (l_OpMode == OP_MODE_CALIBRATION)
	    {
		if (speed > CALIBRATION_SPEED)
		    speed = CALIBRATION_SPEED;
	    }

	    /* set timer according to the new speed value */
	    SetSpeed (speed);
	}
    }

    g_flgIRQ = true;		// Interrupt happened
}


/***************************************************************************//**
 *
 * @brief	Key Handler for Setup Procedure
 *
 * This handler receives the translated key codes from the interrupt-driven
 * key handler, including autorepeat keys.  That is, whenever the user asserts
 * a key (push button), the resulting code is sent to this function.
 *
 * The following keys are recognized:
 * - <b>S1</b> increases the current value, this can be the linear actuator
 *   position, or speed.  If the key is kept asserted, autorepeat gets active.
 * - <b>S2</b> decreases the current value, this can be the linear actuator
 *   position, or speed.  If the key is kept asserted, autorepeat gets active.
 * - <b>S3</b> changes between the setup modes, these are: end position 1, end
 *   position 2, speed selection, exit (storing values in flash).
 *
 * @warning
 * 	This function is called in interrupt context!
 *
 * @param[in] keycode
 *	Translated key code of type KEYCODE.
 *
 ******************************************************************************/
void	LinearKeyHdl (KEYCODE keycode)
{
    /* re-trigger key inactivity timer */
    if (l_thTimeout != NONE)
	sTimerStart (l_thTimeout, KEY_INACTIVITY_TIMEOUT);

    /* asserting a key activates the power for another 500ms */
    PowerEnable();

    /* proceed key code */
    switch (keycode)
    {
	case KEYCODE_S1_ASSERT:		// S1 was asserted
	case KEYCODE_S1_REPEAT:		// or repeated
	    /* depends on the mode what to do */
	    switch (l_OpMode)
	    {
		case OP_MODE_CALIBRATION:	// ignore keys during calibration
		    break;

		case OP_MODE_OPERATIONAL:
		    if (keycode == KEYCODE_S1_ASSERT)	// only accept first hit
			MoveToPos(l_LinearPos_2);
		    break;

		case OP_MODE_SETUP_POS_1:	// increase position
		case OP_MODE_SETUP_POS_2:
		    if (l_ActualPos != l_TargetPos)
			break;			// still moving

		    l_TargetPos += SETUP_DELTA_POS;
		    if (l_TargetPos > TOTAL_DIST_IN_PULSES)
			l_TargetPos = TOTAL_DIST_IN_PULSES;

		    /* Consider direction, enable stepper and timer */
		    StepperEnable(true);
		    break;

		case OP_MODE_SETUP_SPEED:	// increase speed
		    if (++l_NominalSpeed > STEPPER_MAX_SPEED)
			l_NominalSpeed = STEPPER_MAX_SPEED;
		    break;

		default:	// ignore all other modes
		    break;
	    }
	    break;

	case KEYCODE_S2_ASSERT:		// S2 was asserted
	case KEYCODE_S2_REPEAT:		// or repeated
	    /* depends on the mode what to do */
	    switch (l_OpMode)
	    {
		case OP_MODE_CALIBRATION:	// ignore keys during calibration
		    break;

		case OP_MODE_OPERATIONAL:
		    if (keycode == KEYCODE_S2_ASSERT)	// only accept first hit
			MoveToPos(l_LinearPos_1);
		    break;

		case OP_MODE_SETUP_POS_1:	// decrease position
		case OP_MODE_SETUP_POS_2:
		    if (l_ActualPos != l_TargetPos)
			break;			// still moving
		    l_TargetPos -= SETUP_DELTA_POS;
		    if (l_TargetPos < 0)
			l_TargetPos = 0;

		    /* Consider direction, enable stepper and timer */
		    StepperEnable(true);
		    break;

		case OP_MODE_SETUP_SPEED:	// decrease speed
		    if (--l_NominalSpeed < STEPPER_MIN_SPEED)
			l_NominalSpeed = STEPPER_MIN_SPEED;
		    break;

		default:	// ignore all other modes
		    break;
	    }
	    break;

	case KEYCODE_S3_ASSERT:		// S3 (mode key) was asserted
	    /* change mode */
	    switch (l_OpMode)
	    {
		case OP_MODE_CALIBRATION:	// ignore keys during calibration
		    break;

		case OP_MODE_OPERATIONAL:
		    l_OpMode = OP_MODE_SETUP_POS_1;
		    l_PowerOffTimer = 0;	// 0 inhibits decrement
		    SetSpeed (SETUP_SPEED);	// moving slowly
		    break;

		case OP_MODE_SETUP_POS_1:
		    /* store adjusted position 1 */
		    l_LinearPos_1 = l_TargetPos;
		    l_OpMode = OP_MODE_SETUP_POS_2;
		    break;

		case OP_MODE_SETUP_POS_2:
		    /* store adjusted position 2 */
		    l_LinearPos_2 = l_TargetPos;
		    l_DirIncrease = false;
		    l_OpMode = OP_MODE_SETUP_SPEED;
		    break;

		default:	// return to operational mode
		    l_OpMode = OP_MODE_OPERATIONAL;
		    StepperDisable();	// disable stepper

		    /* store adjustments into non-volatile memory */
		    WriteLinearParms();

		    POWER_LED = 0;	// end of setup mode, switch LED off
		    MoveToPos(l_LinearPos_1);	// move to position 1
		    break;
	    }
	    break;

	default:	// ignore all other key codes
	    return;
    }
}


/***************************************************************************//**
 *
 * @brief	Key inactivity timeout routine
 *
 * This routine will be called after @ref KEY_INACTIVITY_TIMEOUT seconds
 * when no key has been asserted.  It terminates the setup mode if this is
 * still active.
 *
 ******************************************************************************/
static void	LinearKeyTimeout (TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    if (l_OpMode > OP_MODE_OPERATIONAL  &&  l_LinearDataValid)
    {
	ReadLinearParms();		// read data back from EEPROM
	l_OpMode = OP_MODE_OPERATIONAL;
	POWER_LED = 0;			// end of setup mode, switch LED off
    }
}


/***************************************************************************//**
 *
 * @brief	Enable DC/DC power converter
 *
 * This routine enables the power (DC/DC converter) for the stepper and its
 * control module SMC11.
 *
 * @note
 * This routine is called from MoveToPos() to initially enable the power before
 * starting movement.  It is also called by LinearKeyHdl() to re-enable power
 * for another POWER_OFF_DELAY period.
 *
 ******************************************************************************/
static void	PowerEnable(void)
{
    SERVO_ENABLE = 1;			// power ON
    Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 1;
    if (l_OpMode == OP_MODE_OPERATIONAL)	// otherwise leave 0
	l_PowerOffTimer = POWER_OFF_DELAY;	// set time counter
}


/***************************************************************************//**
 *
 * @brief	Disable DC/DC power converter
 *
 * This routine disables the power (DC/DC converter) for the stepper and its
 * control module SMC11.
 *
 ******************************************************************************/
static void	PowerDisable(void)
{
    SERVO_ENABLE = 0;			// power OFF
    Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 0;
}


/***************************************************************************//**
 *
 * @brief	Enable stepper driver
 *
 * This routine enables the stepper via its control module SMC11.  It sets
 * the signals EN, DIR, and ASTR.
 *
 ******************************************************************************/
static void	StepperEnable(bool startPulsing)
{
    // set direction bit
    DIR_LINEAR = l_TargetPos > l_ActualPos ? DIR_LINEAR_POS_INC
					   : DIR_LINEAR_POS_DEC;
    EN_LINEAR = 0;		// enable stepper (low-active)
    Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 1;
    ASTR_LINEAR = 1;		// disable power-saving

    if (startPulsing)
	TIMER_Enable(TIMER2, true);	// (re-)start timer
}


/***************************************************************************//**
 *
 * @brief	Enable stepper driver
 *
 * This routine enables the stepper via its control module SMC11.  It sets
 * the signals EN, DIR, and ASTR.
 *
 ******************************************************************************/
static void	StepperDisable(void)
{
    if (l_OpMode == OP_MODE_OPERATIONAL)	// otherwise keep it enabled
	EN_LINEAR = 1;		// disable stepper (low-active)

    ASTR_LINEAR = 0;		// enable power-saving
}


/***************************************************************************//**
 *
 * @brief	Read Linear Parameters from FLASH
 *
 * This routine reads the configuration parameters from FLASH and stores them
 * into the respective variables.
 *
 * @return	true if valid variables could be read, false otherwise.
 *
 ******************************************************************************/
static bool	ReadLinearParms(void)
{
uint16_t	data, sum;

    EE_Read(&magic, &data);
    if (data == MAGIC_ID)
    {
	/* Verify if whole block is valid */
	sum = data;
	EE_Read(&pos1h, &data);
	sum += data;
	EE_Read(&pos1l, &data);
	sum += data;
	EE_Read(&pos2h, &data);
	sum += data;
	EE_Read(&pos2l, &data);
	sum += data;
	EE_Read(&speed, &data);
	sum += data;
	EE_Read(&chksum, &data);

	if (sum == data)
	{
	    /* Checksum is valid, use data from flash */
	    EE_Read(&pos1h, &data);
	    EE_Read(&pos1l, &sum);
	    l_LinearPos_1 = (long)(data << 16) + (long)sum;
	    EE_Read(&pos2h, &data);
	    EE_Read(&pos2l, &sum);
	    l_LinearPos_2 = (long)(data << 16) + (long)sum;
	    EE_Read(&speed, &data);
	    l_NominalSpeed = (int) data;

	    return true;
	}
    }

    return false;
}


/***************************************************************************//**
 *
 * @brief	Write Linear Parameters to FLASH
 *
 * This routine writes the configuration parameters to FLASH.  To protect the
 * data a magic word and a checksum are stored also.
 *
 ******************************************************************************/
static void	WriteLinearParms(void)
{
uint16_t	data, sum;

    INT_Disable();	// disable IRQs during FLASH programming

    /* store adjustments into non-volatile memory */
    sum = data = MAGIC_ID;
    EE_Write(&magic, data);

    data = (uint16_t)((l_LinearPos_1 >> 16) & 0xFFFF);
    sum += data;
    EE_Write(&pos1h, data);
    data = (uint16_t)(l_LinearPos_1 & 0xFFFF);
    sum += data;
    EE_Write(&pos1l, data);

    data = (uint16_t)((l_LinearPos_2 >> 16) & 0xFFFF);
    sum += data;
    EE_Write(&pos2h, data);
    data = (uint16_t)(l_LinearPos_2 & 0xFFFF);
    sum += data;
    EE_Write(&pos2l, data);

    data = (uint16_t)l_NominalSpeed;
    sum += data;
    EE_Write(&speed, data);

    EE_Write(&chksum, sum);

    INT_Enable();
}


/***************************************************************************//**
 *
 * @brief	Trigger Input Handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * state of a trigger input changes.  When a falling edge is detected, this
 * lets the linear actuator move to its respective end position.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of a trigger input.  This is identical with
 *	the pin number, e.g. @ref TRIG_POS_1_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.
 *
 * @param[in] timeStamp
 *	Time stamp when the event has been received.  This parameter is not
 *	used here.
 *
 ******************************************************************************/
void	LinearTrigHdl (int extiNum, bool extiLvl, uint32_t timeStamp)
{
    (void) timeStamp;		// suppress compiler warning "unused parameter"

    if (l_OpMode == OP_MODE_OPERATIONAL	// normal operation required
    &&  extiLvl == 0)		// only a falling edge triggers the move
    {
	if (extiNum == TRIG_POS_1_PIN)
	    MoveToPos (l_LinearPos_1);		// select first position
	else if (extiNum == TRIG_POS_2_PIN)
	    MoveToPos (l_LinearPos_2);		// select second position
    }

    g_flgIRQ = true;		// keep on running
}

#endif

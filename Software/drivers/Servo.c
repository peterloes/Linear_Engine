/***************************************************************************//**
 * @file
 * @brief	Servo Control
 * @author	Ralf Gerhauser
 * @version	2016-10-05
 *
 * This module provides the functionality to control a servo.  In detail, this
 * includes:
 * - Initialization and configuration of the required devices.
 * - PWM signal generation to control the servo position.
 * - A setup routine to adjust the end-points and speed of the servo.
 * - EEPROM-like storage to save the configuration.
 *
 * The firmware supports soft-start, i.e. the servo speed will be ramped-up
 * during the start phase.
 *
 * @warning
 * There must be a stable PWM signal before powering the servo, otherwise
 * the servo will start moving to a wrong position.
 *
 ****************************************************************************//*
Revision History:
2016-10-05,rage	Completed version 1.0, added support for external inputs.
2016-09-09,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "config.h"	// include project configuration parameters

#if ! CFG_LINEAR_A  ||  DOXYGEN		// standard PWM servo

#include "AlarmClock.h"
#include "Servo.h"
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_int.h"
#include "em_prs.h"
#include "em_timer.h"
#include "eeprom_emulation.h"

/*=============================== Definitions ================================*/

/*!@brief Minimum and maximum servo speed, i.e. the step width per 20ms. */
#define SERVO_MIN_SPEED	1
#define SERVO_MAX_SPEED	256

/*!@brief Servo position (PWM value) where calibration starts. */
#define SERVO_START_POS	((SERVO_PWM_MIN_VALUE + SERVO_PWM_MAX_VALUE) / 2)

/*! Magic ID for EEPROM block */
#define MAGIC_ID	0x0815

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Operating and Setup modes for the servo. */
typedef enum
{
    SERVO_MODE_OPERATIONAL,	//!< 0: Normal servo operating mode
    SERVO_MODE_SETUP_POS_1,	//!< 1: Setup mode: adjust first position
    SERVO_MODE_SETUP_POS_2,	//!< 2: Setup mode: adjust second position
    SERVO_MODE_SETUP_SPEED,	//!< 3: Setup mode: adjust speed
    END_SERVO_MODE		//!< End of servo mode definitions
} SERVO_MODE;

/*================================ Local Data ================================*/

    /*! Timer handle for key inactivity timer */
static TIM_HDL		l_thTimeout = NONE;

    /*! Flag shows if there are valid servo settings stored in flash. */
static volatile bool	l_ServoDataValid = false;

    /*! Actual servo mode */
static volatile SERVO_MODE l_ServoMode = SERVO_MODE_SETUP_POS_1;

    /*! Actual servo position (PWM value) */
static volatile int	l_ServoPos = SERVO_START_POS;

    /*! Servo end-position 1 */
static volatile int	l_ServoPos_1 = SERVO_START_POS;

    /*! Servo end-position 2 */
static volatile int	l_ServoPos_2 = SERVO_START_POS;

    /*! Servo speed, i.e. delta position per 20ms */
static volatile int	l_ServoSpeed = SERVO_PWM_DFLT_SPEED;

    /*! Servo direction during speed calibration mode */
static volatile bool	l_DirIncrease;

    /*! Servo enable flag: true means ON, false means OFF */
static volatile bool	l_ServoEnable;

    /*! Servo start time counter (soft start) when servo starts moving [%] */
static volatile int	l_ServoStartCnt;

    /*! Servo stopping time counter after position has been reached [20ms] */
static volatile int	l_ServoStopCnt = SERVO_STOPPING_TIME;

    /*! Define the non-volatile variables. */
static EE_Variable_TypeDef  magic, pos1, pos2, speed, chksum;


/*=========================== Forward Declarations ===========================*/

static void	ServoKeyTimeout (TIM_HDL hdl);
static void	ServoCtrlEnable(void);
static void	ServoCtrlDisable(void);
static bool	ReadServoParms(void);
static void	WriteServoParms(void);

/***************************************************************************//**
 *
 * @brief	Servo control initialization
 *
 * This routine must be called once to initialize all the devices required to
 * control a servo.  It also loads the servo configuration (end-points and
 * speed) from FLASH.
 *
 ******************************************************************************/
void	ServoInit (void)
{
#if 1	// RAGE: should be hardwired
    /* Always enable GND for the Servo via PA6_APP_SWITCH */
    GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 1);
#endif
    /* PA3_SERVO_ENABLE, SERVO_VDD*/
    GPIO_PinModeSet(SERVO_ENABLE_PORT, SERVO_ENABLE_PIN, gpioModePushPull, 0);

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
    l_thTimeout = sTimerCreate (ServoKeyTimeout);

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
    EE_DeclareVariable(&pos1);
    EE_DeclareVariable(&pos2);
    EE_DeclareVariable(&speed);
    EE_DeclareVariable(&chksum);

    /* Get servo end-points from flash */
    if (ReadServoParms())
    {
	l_ServoDataValid = true;
	l_ServoPos = l_ServoPos_1;
	l_ServoMode = SERVO_MODE_OPERATIONAL;
    }

    /*
     * We use timer 1 for the 50Hz time base (servo requires PWM pulse every
     * 20ms) and timer 2 for the pulse generation.  This is, timer 2 run in
     * one-shot mode and is triggered (reloaded and started) by timer 1.
     * CC0 (capture/compare channel 0) of timer 2 is routed to PC8.
     */
    /* Enable clock for GPIO module should already be done) */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Enable clock for TIMER1 module */
    CMU_ClockEnable(cmuClock_TIMER1, true);

    /* Enable clock for the Peripheral Reflex System */
    CMU_ClockEnable(cmuClock_PRS, true);

    /* Enable clock for TIMER2 module */
    CMU_ClockEnable(cmuClock_TIMER2, true);

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
     * Build a connection from timer 1 overflow to timer 2 via the Peripheral
     * Reflex System (PRS) for triggering timer 2.
     */
    PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER1,
			   PRS_CH_CTRL_SIGSEL_TIMER1OF, prsEdgePos);

    /*
     * Timer 2 is used to generate a pulse on port C pin 8.
     */

    /* Enable clock for TIMER2 module */
    CMU_ClockEnable(cmuClock_TIMER2, true);

    /* Reset TIMER2, set default values for registers */
    TIMER_Reset(TIMER2);

    /*
     * Select CC0 channel parameters
     *
     * CC0 is only used to route channel 0 of the Peripheral Reflex System
     * (PRS) to the start/stop/reload logic of the timer. See Reference Manual,
     * figure "TIMER Block Overview" which shows a connection from CC0 to the
     * Counter Control block.
     */
    static const TIMER_InitCC_TypeDef timer2_CC0Init =
    {
	.eventCtrl  = timerEventEveryEdge,	// not used
	.edge       = timerEdgeBoth,		// pass both edges to control block
	.prsSel     = timerPRSSELCh0,		// PRS channel 0 is used
	.cufoa      = timerOutputActionNone,	// not used
	.cofoa      = timerOutputActionNone,	// not used
	.cmoa       = timerOutputActionNone,	// not used
	.mode       = timerCCModeOff,		// no capture or compare
	.filter     = false,			// not used
	.prsInput   = true,			// input from PRS channel
	.coist      = false,			// not used
	.outInvert  = false,			// not used
    };

    /* Configure CC channel 0 */
    TIMER_InitCC(TIMER2, 0, &timer2_CC0Init);

    /*
     * CC1 generates the output pulse for the servo via GPIO pin PC9.  We
     * use a 16MHz clock to be able to select a pulse width between 500us
     * and 2700us (max. 4000us) for the servo signal.
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

    /* Set Top Value - max. pulse width is 4ms based on a 16MHz clock */
    TIMER_TopSet(TIMER2, 16000 * 4);	// [kHz] * [ms]

    /* Generate a PWM signal that positions the servo in the middle */
    TIMER_CompareSet(TIMER2, 1, l_ServoPos);

    /* Select timer parameters */
    static const TIMER_Init_TypeDef timer2Init =
    {
	.enable     = false,			// do not start yet
	.debugRun   = true,			// continue running during debug
	.prescale   = timerPrescale2,		// 32Mhz / 2 = 16Mhz
	.clkSel     = timerClkSelHFPerClk,	// use 32Mhz clock
	.fallAction = timerInputActionNone,	// ignore falling edge
	.riseAction = timerInputActionReloadStart, // reload+start on rising edge
	.mode       = timerModeUp,		// up-counter for PWM
	.dmaClrAct  = false,			// no DMA
	.quadModeX4 = false,			// no X4 mode
	.oneShot    = true,			// use one-shot mode
	.sync       = false,			// no sync with other timers
    };

    /* Initialize timer */
    TIMER_Init(TIMER2, &timer2Init);

    /* Enable Overflow Interrupt of Timer 1 */
    TIMER_IntEnable(TIMER1, TIMER_IEN_OF);
    NVIC_EnableIRQ(TIMER1_IRQn);

    /* Enable timers */
    TIMER_Enable(TIMER1, true);
    TIMER_Enable(TIMER2, true);

    /* Automatically enter setup mode if servo is not configured */
    if (l_ServoMode != SERVO_MODE_OPERATIONAL)
    {
	ServoCtrlEnable();
	l_ServoSpeed = SERVO_PWM_DFLT_SPEED; // select default speed
    }
    else
    {
	/* let the servo move to end position 1 */
	ServoCtrlEnable();
	l_ServoStopCnt = 50;	// (* 20ms) extend stopping time to one second
    }
}


/***************************************************************************//**
 *
 * @brief	Timer 1 Interrupt Handler
 *
 * This is the interrupt handler for the timer 1 reload interrupt which occurs
 * every 20ms.  It is used as time base for servo speed control, flashing the
 * LED, and for the servo stopping time.
 *
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
static uint32_t	cnt20ms;
static int	phase;
int		currPos;

    /* increase 20ms counter */
    cnt20ms++;

    /* determine reason for this interrupt */
    if (TIMER_IntGet(TIMER1) & TIMER_IF_OF)
    {
	TIMER_IntClear(TIMER1, TIMER_IFC_OF);

	/*
	 * Another 20ms is over, handle the following conditions:
	 * - LED control
	 * - servo position
	 * - servo stopping time
	 */

	/* LED control */
	if (l_ServoMode != SERVO_MODE_OPERATIONAL)
	{
	    if ((cnt20ms & 0x0F) == 0)	// every 20ms x 16 = 320ms
	    {
		phase++;
		if (phase < 2 * (int)l_ServoMode + 1)
		{
		    /* switch LED on or off */
		    POWER_LED = (phase & 0x01 ? 1 : 0);
		}
		else if (phase > 2 * (int)l_ServoMode + 3)
		{
		    phase = 0;
		}
	    }
	}

	/* if in speed setup mode, move between the two end points */
	if (l_ServoMode == SERVO_MODE_SETUP_SPEED)
	{
	    if (l_DirIncrease)
	    {
		int maxPos = l_ServoPos_2 > l_ServoPos_1 ?
			     l_ServoPos_2 : l_ServoPos_1;

		l_ServoPos += l_ServoSpeed;
		if (l_ServoPos > maxPos)
		{
		    l_ServoPos = maxPos;
		    l_DirIncrease = false;
		}
	    }
	    else
	    {
		int minPos = l_ServoPos_2 < l_ServoPos_1 ?
			     l_ServoPos_2 : l_ServoPos_1;

		l_ServoPos -= l_ServoSpeed;
		if (l_ServoPos < minPos)
		{
		    l_ServoPos = minPos;
		    l_DirIncrease = true;
		}
	    }

	    l_ServoStartCnt = 100;	// 100% during speed adjustment
	}
	else if (l_ServoMode == SERVO_MODE_OPERATIONAL)
	{
	    /* increase servo start time counter until it reaches 100% */
	    if (l_ServoStartCnt < 100)
	    {
		l_ServoStartCnt++;
	    }
	}
	else
	{
	    l_ServoStartCnt = 100;	// 100% during end position adjustment
	}

	/* only change position if counter is positive */
	if (l_ServoStartCnt > 0)
	{
	    /* be sure servo power is enabled */
	    SERVO_ENABLE = 1;

	    /* update servo position according selected speed */
	    currPos = (int)TIMER_CaptureGet(TIMER2, 1);
	    if (currPos < l_ServoPos)
	    {
		/* current position is too low - increase it */
		currPos += (l_ServoSpeed * l_ServoStartCnt / 100);
		if (currPos > l_ServoPos)
		    currPos = l_ServoPos;

		TIMER_CompareSet(TIMER2, 1, currPos);
	    }
	    else if (currPos > l_ServoPos)
	    {
		/* current position is too high - increase it */
		currPos -= (l_ServoSpeed * l_ServoStartCnt / 100);
		if (currPos < l_ServoPos)
		    currPos = l_ServoPos;

		TIMER_CompareSet(TIMER2, 1, currPos);
	    }
	    else if (l_ServoMode == SERVO_MODE_OPERATIONAL)
	    {
		/* current position is correct - decrease stopping time */
		if (--l_ServoStopCnt <= 0)
		{
		    ServoCtrlDisable();
		}
	    }
	}
    }

    g_flgIRQ = true;		// Interrupt happened
}


/***************************************************************************//**
 *
 * @brief	Key Handler for Servo Setup Procedure
 *
 * This handler receives the translated key codes from the interrupt-driven
 * key handler, including autorepeat keys.  That is, whenever the user asserts
 * a key (push button), the resulting code is sent to this function.
 *
 * The following keys are recognized:
 * - <b>S1</b> increases the current value, this can be the servo position, or
 *   the servo speed.  If the key is kept asserted, autorepeat gets active.
 * - <b>S2</b> decreases the current value, this can be the servo position, or
 *   the servo speed.  If the key is kept asserted, autorepeat gets active.
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
void	ServoKeyHdl (KEYCODE keycode)
{
    /* re-trigger key inactivity timer */
    if (l_thTimeout != NONE)
	sTimerStart (l_thTimeout, KEY_INACTIVITY_TIMEOUT);

    /* asserting a key activates the servo control logic for another 500ms */
    ServoCtrlEnable();

    /* proceed key code */
    switch (keycode)
    {
	case KEYCODE_S1_ASSERT:		// S1 was asserted
	case KEYCODE_S1_REPEAT:		// or repeated
	    /* depends on the mode what to do */
	    switch (l_ServoMode)
	    {
		case SERVO_MODE_OPERATIONAL:
		    if (keycode == KEYCODE_S1_ASSERT)	// only accept first hit
			l_ServoPos = l_ServoPos_2;	// select second position
		    break;

		case SERVO_MODE_SETUP_POS_1:	// increase position
		case SERVO_MODE_SETUP_POS_2:
		    l_ServoPos += l_ServoSpeed;
		    if (l_ServoPos > SERVO_PWM_MAX_VALUE)
			l_ServoPos = SERVO_PWM_MAX_VALUE;
		    break;

		case SERVO_MODE_SETUP_SPEED:	// increase speed
		    if (++l_ServoSpeed > SERVO_MAX_SPEED)
			l_ServoSpeed = SERVO_MAX_SPEED;
		    break;

		default:	// ignore all other modes
		    break;
	    }
	    break;

	case KEYCODE_S2_ASSERT:		// S2 was asserted
	case KEYCODE_S2_REPEAT:		// or repeated
	    /* depends on the mode what to do */
	    switch (l_ServoMode)
	    {
		case SERVO_MODE_OPERATIONAL:
		    if (keycode == KEYCODE_S2_ASSERT)	// only accept first hit
			l_ServoPos = l_ServoPos_1;	// select first position
		    break;

		case SERVO_MODE_SETUP_POS_1:	// decrease position
		case SERVO_MODE_SETUP_POS_2:
		    l_ServoPos -= l_ServoSpeed;
		    if (l_ServoPos < SERVO_PWM_MIN_VALUE)
			l_ServoPos = SERVO_PWM_MIN_VALUE;
		    break;

		case SERVO_MODE_SETUP_SPEED:	// decrease speed
		    if (--l_ServoSpeed < SERVO_MIN_SPEED)
			l_ServoSpeed = SERVO_MIN_SPEED;
		    break;

		default:	// ignore all other modes
		    break;
	    }
	    break;

	case KEYCODE_S3_ASSERT:		// S3 (mode key) was asserted
	    /* change mode */
	    switch (l_ServoMode)
	    {
		case SERVO_MODE_OPERATIONAL:
		    l_ServoMode = SERVO_MODE_SETUP_POS_1;
		    l_ServoSpeed = SERVO_PWM_DFLT_SPEED; // select default speed
		    break;

		case SERVO_MODE_SETUP_POS_1:
		    /* store adjusted position 1 */
		    l_ServoPos_1 = l_ServoPos;
		    l_ServoMode = SERVO_MODE_SETUP_POS_2;
		    break;

		case SERVO_MODE_SETUP_POS_2:
		    /* store adjusted position 2 */
		    l_ServoPos_2 = l_ServoPos;
		    l_DirIncrease = false;
		    l_ServoMode = SERVO_MODE_SETUP_SPEED;
		    break;

		default:	// return to operational mode
		    l_ServoMode = SERVO_MODE_OPERATIONAL;

		    /* store adjustments into non-volatile memory */
		    WriteServoParms();

		    POWER_LED = 0;	// end of setup mode, switch LED off
		    l_ServoPos = l_ServoPos_1;	// move to position 1
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
static void	ServoKeyTimeout (TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    if (l_ServoMode != SERVO_MODE_OPERATIONAL  &&  l_ServoDataValid)
    {
	ReadServoParms();
	l_ServoMode = SERVO_MODE_OPERATIONAL;
	POWER_LED = 0;			// end of setup mode, switch LED off
    }
}


/***************************************************************************//**
 *
 * @brief	Enable Servo Control Logic
 *
 * This routine enables the power (DC/DC converter) for the servo and activates
 * the timers to drive the PWM servo signal.
 *
 ******************************************************************************/
static void	ServoCtrlEnable(void)
{
    if (! l_ServoEnable)
    {
	l_ServoEnable = true;

	TIMER_Enable(TIMER2, true);
	Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 1;

	/* establish PWM signal 40ms before power-up of the servo */
	l_ServoStartCnt = (-2);
	l_ServoStopCnt = SERVO_STOPPING_TIME;
    }
}


/***************************************************************************//**
 *
 * @brief	Disable Servo Control Logic
 *
 * This routine disables the power (DC/DC converter) for the servo and stops
 * the timers for the PWM servo signal.
 *
 ******************************************************************************/
static void	ServoCtrlDisable(void)
{
    TIMER_Enable(TIMER2, false);
    SERVO_ENABLE = 0;			// switch off servo power
    Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 0;

    l_ServoEnable = false;
}


/***************************************************************************//**
 *
 * @brief	Read Servo Parameters from FLASH
 *
 * This routine reads the servo parameters from FLASH and stores them into the
 * respective variables.
 *
 * @return	true if valid variables could be read, false otherwise.
 *
 ******************************************************************************/
static bool	ReadServoParms(void)
{
uint16_t	data, sum;

    EE_Read(&magic, &data);
    if (data == MAGIC_ID)
    {
	/* Verify if whole block is valid */
	sum = data;
	EE_Read(&pos1, &data);
	sum += data;
	EE_Read(&pos2, &data);
	sum += data;
	EE_Read(&speed, &data);
	sum += data;
	EE_Read(&chksum, &data);

	if (sum == data)
	{
	    /* Checksum is valid, use data from flash */
	    EE_Read(&pos1, &data);
	    l_ServoPos_1 = (int) data;
	    EE_Read(&pos2, &data);
	    l_ServoPos_2 = (int) data;
	    EE_Read(&speed, &data);
	    l_ServoSpeed = (int) data;

	    return true;
	}
    }

    return false;
}


/***************************************************************************//**
 *
 * @brief	Write Servo Parameters to FLASH
 *
 * This routine writes the servo parameters to FLASH.  To protect the data
 * a magic word and a checksum are stored also.
 *
 ******************************************************************************/
static void	WriteServoParms(void)
{
uint16_t	data, sum;

    INT_Disable();	// disable IRQs during FLASH programming

    /* store adjustments into non-volatile memory */
    sum = data = MAGIC_ID;
    EE_Write(&magic, data);

    data = (uint16_t)l_ServoPos_1;
    sum += data;
    EE_Write(&pos1, data);

    data = (uint16_t)l_ServoPos_2;
    sum += data;
    EE_Write(&pos2, data);

    data = (uint16_t)l_ServoSpeed;
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
 * lets the servo move to its respective end position.
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
void	ServoTrigHdl (int extiNum, bool extiLvl, uint32_t timeStamp)
{
    (void) timeStamp;		// suppress compiler warning "unused parameter"

    if (l_ServoMode == SERVO_MODE_OPERATIONAL	// normal operation required
    &&  extiLvl == 0)		// only a falling edge triggers the servo
    {
	if (extiNum == TRIG_POS_1_PIN)
	    l_ServoPos = l_ServoPos_1;	// select first position
	else if (extiNum == TRIG_POS_2_PIN)
	    l_ServoPos = l_ServoPos_2;	// select second position

	ServoCtrlEnable();
    }

    g_flgIRQ = true;		// keep on running
}

#endif

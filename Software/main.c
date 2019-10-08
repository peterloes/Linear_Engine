/***************************************************************************//**
 * @file
 * @brief	Servo_LinearA
 * @author	Ralf Gerhauser
 * @version	2017-10-14
 *
 * This application consists of the following modules:
 * - Servo.c - The servo control module.
 * - ExtInt.c - External interrupt handler.
 * - Keys.c - Key interrupt handling and translation.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - clock.c - An implementation of the POSIX time() function.
 * - eeprom_emulation.c - EEPROM emulation via FLASH
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * and AN0019 "EEPROM emulation" from Energy Micro AS.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2017-10-14,rage	Completed version 2.0 which includes support for a linear
		actuator (via stepper motor), see Module LinearA.  Use the
		define CFG_LINEAR_A to select PWM servo or linear actuator.
2016-10-05,rage	Completed version 1.0, added support for external inputs.
2016-09-09,rage	Initial version.
*/

/*!
 * @mainpage
 * <b>Description</b><br>
 * This is the firmware for the projects Servo_2015 (standard PWM servo) and
 * LinearA_2017 (linear servo, i.e. stepper motor with spindle).
 * Setup and control of the servo is done
 * via 3 keys, and, for selecting the two end positions, also via two external
 * inputs, please see below.  The firmware supports soft-start, i.e. the servo
 * speed will be ramped-up during the start phase.<br>
 * Use define @ref CFG_LINEAR_A to select between PWM servo and linear actuator.
 *
 * <b>Microcontroller</b><br>
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board must be configured to use the
 * external 32MHz XTAL for that purpose, set define @ref USE_EXT_32MHZ_CLOCK
 * to 1.
 *
 * <b>Keys (Push Buttons)</b><br>
 * There exist 3 push buttons on the board.  When asserted, the following
 * action will be taken:
 * - In setup mode, S1 increases the current value, this can be the servo
 *   position, or the servo speed.  If the key is kept asserted, autorepeat
 *   gets active.  During normal operation, S1 lets the servo move to end
 *   position 2.
 * - In setup mode, S2 decreases the current value, this can be the servo
 *   position, or the servo speed.  If the key is kept asserted, autorepeat
 *   gets active.  During normal operation, S2 lets the servo move to end
 *   position 1.
 * - S3 changes between the setup modes, these are: end position 1, end
 *   position 2, speed selection, exit (storing values in FLASH).
 *
 * If no key has been asserted for more than 30 seconds, the setup mode is
 * aborted and the firmware returns to normal operation.
 *
 * <b>Setup Mode</b><br>
 * The servo setup procedure consists of 4 steps:
 * -# Adjust end position 1.  For PWM servos this can be the most left or the
 *    most right end point.  This position will be reached after power-up if
 *    valid servo data has been found in FLASH.  In contrast, a linear actuator
 *    doesn't know about its absolute position, therefore it first moves to
 *    to its physical start position (near the motor) after power-up.  From
 *    that, position 1 can be adjusted.
 * -# Adjust end position 2.  This is the opposite end position to position 1.
 * -# Adjust servo speed.  The servo permanently moves between the two end
 *    points.  The speed can be adjusted via S1 and S2.
 * -# Asserting S3 the 4th time stores the servo parameters into FLASH and
 *    returns to normal operation.
 *
 * <b>LEDs</b><br>
 * There is just one red LED connected to GPIO port PC0.
 * - It is enabled for a short time during power-up to show the board is
 *   healthy.
 * - During the servo setup, the LED shows the current mode (1 to 3) by
 *   flashing the respective number of times, followed by a pause.
  *
 * <b>External Inputs</b><br>
 * In normal operation mode, the servo can also be controlled via two external
 * inputs, these are:
 * - Port PC6 is equivalent to S1, a low edge signal lets the servo move to end
 *   position 2 (this port may also be used as SDA for an I2C connection).
 * - Port PC7 is equivalent to S2, a low edge signal lets the servo move to end
 *   position 1 (this port may also be used as SCL for an I2C connection).
 *
 * <b>Firmware</b><br>
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# LED is switched on for test purposes (lamp test)
 * -# Further hardware initialization (Keys, Interrupts, Alarm Clock)
 * -# LED is switched off
 * -# If no valid servo parameters have been found, setup mode 1 is activated,
 *    the LED flashes one time, followed by a pause.
 * -# If valid servo parameters exist, the servo moves to end position 1.
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Servo setup or control (interrupt driven, runs in background)
 * - Entering the right energy mode
 */
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dma.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "Keys.h"
#include "AlarmClock.h"
// be sure to include only one of the two header files!
#if CFG_LINEAR_A	// SMC11 stepper
  #include "LinearA.h"
#else			// standard PWM servo
  #include "Servo.h"
#endif

/*================================ Global Data ===============================*/

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the RTC still work in EM1.
 *
 * Examples:
 *
   @code
   // Module Servo requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 1;
   ...
   // Module Servo is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_SERVO) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;

/*================================ Local Data ================================*/

#if CFG_LINEAR_A	// SMC11 stepper
    #define TRIG_HANDLER	LinearTrigHdl
    #define KEY_HANDLER		LinearKeyHdl
#else			// standard PWM servo
    #define TRIG_HANDLER	ServoTrigHdl
    #define KEY_HANDLER		ServoKeyHdl
#endif

/*! EXTI initialization structure
 *
 * Connect the external interrupts of the push buttons to the key handler, the
 * DCF77 signal to the atomic clock module, the outer and inner light barrier
 * to their handler.
 */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	KEY_EXTI_MASK,	KeyHandler	},	// Keys
    {	TRIG_EXTI_MASK,	TRIG_HANDLER	},	// external trigger
    {	0,		NULL		}
};

/*!
 * Initialization structure to define a function to be called for each
 * translated key.
 */
static const KEY_INIT  l_KeyInit =
{
    .AR_Threshold = 250,	// Threshold in [ms] after autorepeat starts
    .AR_Rate = 20,		// Key rate in [ms] when autorepeat is active
    .KeyFct = KEY_HANDLER	// Fct. to be called for each translated key
};

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* Set up clocks */
    cmuSetup();

#ifdef DEBUG
    dbgInit();
#endif

    /* Configure port to drive the red Power-On LED (LED1) - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

#if CFG_LINEAR_A	// SMC11 stepper
    /* Initialize Linear Actuator */
    LinearInit();
#else			// standard PWM servo
    /* Initialize PWM Output for Servo */
    ServoInit();
#endif

    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */

    /* Initialize key hardware */
    KeyInit (&l_KeyInit);

    /*
     * Initialize External Interrupts
     */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();

    msDelay(500);	// show LED for 0.5s, then switch it off
    POWER_LED = 0;

    /* Enable all other External Interrupts */
    ExtIntEnableAll();


    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
	/*
	 * Check for current power mode:  If a minimum of one active module
	 * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
	 * entered.  If no one requires EM1 activity, EM2 is entered.
	 */
	if (! g_flgIRQ)		// enter EM only if no IRQ occurred
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	    else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}

/***************************************************************************//**
 * @file
 * @brief	Header file of module LinearA.c
 * @author	Ralf Gerhauser
 * @version	2017-10-14
 ****************************************************************************//*
Revision History:
2017-10-14,rage	Completed version 2.0.
2017-09-09,rage	Initial version, based on "Servo.h".
*/

#ifndef __INC_LinearA_h
#define __INC_LinearA_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters
#include "Keys.h"

/*=============================== Definitions ================================*/

/*!@brief Definitions for the PA3_SERVO_ENABLE signal (DC/DC converter) */
#define SERVO_ENABLE_PORT	gpioPortA
#define SERVO_ENABLE_PIN	3
#define SERVO_ENABLE   IO_Bit(GPIO->P[SERVO_ENABLE_PORT].DOUT, SERVO_ENABLE_PIN)

/*!@brief Definitions for the PE10_EN_LINEAR signal (low-active) */
#define EN_LINEAR_PORT		gpioPortE
#define EN_LINEAR_PIN		10
#define EN_LINEAR	IO_Bit(GPIO->P[EN_LINEAR_PORT].DOUT, EN_LINEAR_PIN)

/*!@brief Definitions for the PE11_DIR_LINEAR signal */
#define DIR_LINEAR_PORT		gpioPortE
#define DIR_LINEAR_PIN		11
#define DIR_LINEAR	IO_Bit(GPIO->P[DIR_LINEAR_PORT].DOUT, DIR_LINEAR_PIN)
#define DIR_LINEAR_POS_INC	1	// logical state for position increment
#define DIR_LINEAR_POS_DEC	0	// logical state for position decrement

/*!@brief Definitions for the PE12_ASTR_LINEAR signal (low-active) */
#define ASTR_LINEAR_PORT	gpioPortE
#define ASTR_LINEAR_PIN		12
#define ASTR_LINEAR	IO_Bit(GPIO->P[ASTR_LINEAR_PORT].DOUT, ASTR_LINEAR_PIN)

/*!@brief Definitions for the position zero detection input (low-active) */
#define POS_ZERO_DETECT_PORT	gpioPortA
#define POS_ZERO_DETECT_PIN	5
#define POS_ZERO_DETECT IO_Bit(GPIO->P[POS_ZERO_DETECT_PORT].DIN, POS_ZERO_DETECT_PIN)

/*!@brief Stepper pulses per [mm].
 * According to the data sheet of the L2018S0604 stepper motor, it moves
 * 0.005 mm/step, i.e. 200 steps/mm.  Because the SMC-11 controller is
 * configured for microstep mode, this means 16 pulses per step, there are
 * 3200 pulses/mm required.
 */
#define PULSES_PER_MM			(16L * 200L)

/*!@brief Total distance to move in [mm]. */
//#define TOTAL_DIST_IN_MM		100L	// 10cm
#define TOTAL_DIST_IN_MM		50L	// 5cm


/*!@brief Distance to speed up in [mm]. */
#define SPEED_UP_DIST_IN_MM		10L	// 1cm

/*!@brief Distance to slow down in [mm]. */
#define SLOW_DOWN_DIST_IN_MM		10L	// 1cm

/*!@brief Number of [mm] to move per key hit during setup of end-points. */
#define SETUP_DELTA_POS_MM		1 / 5	// DO NOT USE BRACKETS!

/*!@brief Minimum speed, used to reach final target position. */
#define STEPPER_MIN_SPEED		100L

/*!@brief Speed during setup process of the end-points. */
#define SETUP_SPEED			150L

/*!@brief Speed during calibration, i.e. reaching the physical end position. */
#define CALIBRATION_SPEED		400L

/*!@brief Maximum allowed speed. */
#define STEPPER_MAX_SPEED		600L


/*!@brief Timeout in [s] to leave setup mode when no key has been asserted */
#define KEY_INACTIVITY_TIMEOUT		30	// 30s

/*!@brief Here follows the definition of the two trigger inputs and their
 * related hardware configuration.  A falling edge triggers the movement of
 * the servo to the respective end position 1 or 2.
 */
#define TRIG_POS_2_PORT	gpioPortC
#define TRIG_POS_2_PIN	6

#define TRIG_POS_1_PORT	gpioPortC
#define TRIG_POS_1_PIN	7

/*!@brief Bit mask of all affected external interrupts (EXTIs). */
#define TRIG_EXTI_MASK	((1 << TRIG_POS_1_PIN) | (1 << TRIG_POS_2_PIN))

/*================================ Prototypes ================================*/

    /* Initialize servo hardware */
void	LinearInit(void);

    /* Key handler to setup and control the servo */
void	LinearKeyHdl(KEYCODE keycode);

    /* Handler for the two trigger inputs */
void	LinearTrigHdl(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_LinearA_h */

/***************************************************************************//**
 * @file
 * @brief	Header file of module Servo.c
 * @author	Ralf Gerhauser
 * @version	2017-07-
 ****************************************************************************//*
Revision History:
2016-10-05,rage	Added support for external trigger inputs.
2016-09-09,rage	Initial version.
*/

#ifndef __INC_Servo_h
#define __INC_Servo_h

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

/*!@brief Servo PWM timing is based on a 16MHz clock, pulse width should be
 * between 1.125ms (left end) and 2.25ms (right end).  These end points have
 * been determined by tests.
 */
#define SERVO_PWM_MIN_VALUE		(1125 * 16)	// [us] * [MHz]
#define SERVO_PWM_MAX_VALUE		(2250 * 16)	// [us] * [MHz]

/*!@brief Default speed for position adjustment is 16 steps per 20ms */
#define SERVO_PWM_DFLT_SPEED		16

/*!@brief Time to keep PWM signal active after position has been reached [20ms]*/
#define SERVO_STOPPING_TIME		10	// 10 x 20ms = 200ms = 0.2s

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
void	ServoInit(void);

    /* Key handler to setup and control the servo */
void	ServoKeyHdl(KEYCODE keycode);

    /* Handler for the two trigger inputs */
void	ServoTrigHdl(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_Servo_h */

/***************************************************************************//**
 * @file
 * @brief	Header file of module Keys.c
 * @author	Ralf Gerhauser
 * @version	2016-09-09
 ****************************************************************************//*
Revision History:
2016-09-09,rage	Derived from project "Akku_LCD".
*/

#ifndef __INC_Keys_h
#define __INC_Keys_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Per default the key autorepeat function is enabled, however for
 * debugging purposes, it may be useful to disable it by setting the define
 * to 0.
 */
#define KEY_AUTOREPEAT	1

/*!@brief Here follows the definition of all keys (push buttons) and their
 * related hardware configurations.
 */
#define KEY_S1_PORT	gpioPortA	// increase value (position or speed)
#define KEY_S1_PIN	0

#define KEY_S2_PORT	gpioPortA	// decrease value (position or speed)
#define KEY_S2_PIN	1

#define KEY_S3_PORT	gpioPortA	// mode select
#define KEY_S3_PIN	2


/*!@brief Bit mask of all affected external interrupts (EXTIs). */
#define KEY_EXTI_MASK	((1 << KEY_S1_PIN)|(1 << KEY_S2_PIN)|(1 << KEY_S3_PIN))

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Translated key codes. */
typedef enum
{
    KEYCODE_NONE,		//!< No key code active
    KEYCODE_S1_ASSERT,		//!< Key code for S1 once asserted
    KEYCODE_S1_REPEAT,		//!< Key code for S1 autorepeat, still active
    KEYCODE_S1_RELEASE,		//!< Key code for S1 released again
    KEYCODE_S2_ASSERT,		//!< Key code for S2 once asserted
    KEYCODE_S2_REPEAT,		//!< Key code for S2 autorepeat, still active
    KEYCODE_S2_RELEASE,		//!< Key code for S2 released again
    KEYCODE_S3_ASSERT,		//!< Key code for S3 once asserted
    KEYCODE_S3_REPEAT,		//!< Key code for S3 autorepeat, still active
    KEYCODE_S3_RELEASE,		//!< Key code for S3 released again
    END_KEYCODE			//!< End of key code definitions
} KEYCODE;

/*!@brief Offsets to be added to the ASSERT key code */
#define KEYOFFS_REPEAT	(KEYCODE)1	// +1 for REPEAT code
#define KEYOFFS_RELEASE	(KEYCODE)2	// +2 for RELEASE code

/*!@brief Function to be called for each translated key code. */
typedef void	(* KEY_FCT)(KEYCODE keycode);

/*!@brief Key initialization structure.
 *
 * Initialization structure to define the timings for the autorepeat (AR)
 * threshold and rate (in milliseconds), and a function to be called for each
 * translated key.
 *
 * <b>Typical Example:</b>
 * @code
 * static const KEY_INIT l_KeyInit =
 * {
 *    250,		// Autorepeat threshold is 250ms
 *    100,		// Autorepeat rate is 100ms (10Hz)
 *    ServoSetup	// Key handler
 * };
 * @endcode
 */
typedef struct
{
    uint16_t  AR_Threshold;	//!< Threshold in [ms] after autorepeat starts
    uint16_t  AR_Rate;		//!< Key rate in [ms] when autorepeat is active
    KEY_FCT   KeyFct;		//!< Fct. to be called for each translated key
} KEY_INIT;

/*================================ Prototypes ================================*/

/* Initialize key hardware */
void	KeyInit (const KEY_INIT *pInitStruct);

/* Key handler, called from interrupt service routine */
void	KeyHandler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_Keys_h */

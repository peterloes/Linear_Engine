/***************************************************************************//**
 * @file
 * @brief	Project configuration file
 * @author	Ralf Gerhauser
 * @version	2016-10-05
 *
 * This file allows to set miscellaneous configuration parameters.  It must be
 * included by all modules.
 *
 ****************************************************************************//*
Revision History:
2017-07-01,rage	Added configuration CFG_LINEAR_A.
2016-09-09,rage	Derived from project "SNB_Heaven".
*/

#ifndef __INC_config_h
#define __INC_config_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"

/*=============================== Definitions ================================*/

/*
 * Basic defines - should all be moved to Generic.h
 */
    /* terminators for lists and strings */

#define	EOL		NULL		/* EndOfList		*/
#define EOS		'\0'		/* EndOfString		*/
#define	NONE		(-1)

    /* macro to calculate the number of elements of an array */
#define ELEM_CNT(array)  (sizeof (array) / sizeof ((array)[0]))


/*!
 * @brief Select hardware platform for this firmware.
 * If CFG_LINEAR_A is 1, the firmware supports the SMC11 stepper module
 * (file LinearA.c), if 0, a standard PWM servo (file Servo.c) is supported.
 */
#define CFG_LINEAR_A	1

/*
 * LED Definitions for this project
 */
    /*!@brief GPIO Port of the (red) Power-LED. */
#define POWER_LED_PORT		gpioPortC
    /*!@brief GPIO Pin of the (red) Power-LED. */
#define POWER_LED_PIN		0
    /*! @brief Macro to set or clear the Power-LED */
#define POWER_LED   IO_Bit(GPIO->P[POWER_LED_PORT].DOUT, POWER_LED_PIN)

/*!
 * @brief MPU Clock Configuration.
 *
 * Set to 0 to use the internal RC oscillator, if 1 the external 32MHz XTAL
 * is used - must be set 1 for this project.
 */
#define USE_EXT_32MHZ_CLOCK	1

/*
 * Configuration for module "AlarmClock"
 */
    /*!@brief RTC frequency in [Hz]. */
#define RTC_COUNTS_PER_SEC	32768


/*================================== Macros ==================================*/

#ifdef DEBUG
    /*
     * Debugging output via ITM
     */
    #define DBG_PUTC(ch)	ITM_SendChar(ch)
    #define DBG_PUTS(str)	ITM_SendStr(str)
    uint32_t ITM_SendChar (uint32_t ch);
    void ITM_SendStr(const char *pStr);
    void dbgInit(void);
#else
    #define DBG_PUTC(ch)
    #define DBG_PUTS(str)
#endif

    /*! Macro to address a single bit in the I/O range (peripheral range) in
     *  an atomic manner.
     * @param address   I/O register address.
     * @param bitNum    Bit number within this register.
     */
#define IO_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_PER_BASE				\
			+ (((uint32_t)(address)) - PER_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access an I/O-bit. */
#define IO_Bit(regName, bitNum)	*IO_BIT_ADDR(&regName, bitNum)

    /*! Macro to address a single bit in an SRAM variable in an atomic manner.
     * @param address   Address of the variable in SRAM.
     * @param bitNum    Bit number within this variable.
     */
#define SRAM_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_RAM_BASE				\
			+ (((uint32_t)(address)) - RAM_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access a bit in a variable. */
#define Bit(varName, bitNum)	*SRAM_BIT_ADDR(&varName, bitNum)

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Structure to hold Project Information */
typedef struct
{
    char const  ID[12];
    char const  Date[16];
    char const  Time[10];
    char const  Version[16];
} PRJ_INFO;


/*!@brief Enumeration of Alarm Identifiers
 *
 * This is the list of Alarm IDs used by this application.  They are used to
 * identify a particular alarm time entry via the <b>alarmNum</b> parameter
 * when calling alarm functions, e.g. AlarmSet().
 */
typedef enum
{
    END_ALARM_ID
} ALARM_ID;


/*!@brief Enumeration of the EM1 Modules
 *
 * This is the list of Software Modules that require EM1 to work, i.e. they
 * will not work in EM2 because clocks, etc. would be disabled.  These enums
 * are used to set/clear the appropriate bit in the @ref g_EM1_ModuleMask.
 */
typedef enum
{
    EM1_MOD_SERVO,	//!<  0: Servo timers require the 32MHz HF clock
    END_EM1_MODULES
} EM1_MODULES;

/*======================== External Data and Routines ========================*/

extern volatile bool	 g_flgIRQ;		// Flag: Interrupt occurred
extern volatile uint16_t g_EM1_ModuleMask;	// Modules that require EM1


#endif /* __INC_config_h */

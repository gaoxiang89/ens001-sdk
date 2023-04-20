/**************************************************************************//**
 * @file     system_CMSDK_CM0.c
 * @brief    CMSIS Cortex-M0 Device Peripheral Access Layer Source File for
 *           Device CMSDK
 * @version  V3.01
 * @date     06. March 2012
 *
 * @note
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#include <stdint.h>
#include "ens001.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#define HSI_FREQ    (16000000UL)            /* Oscillator frequency               */


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency = HSI_FREQ;    /*!< System Clock Frequency (Core Clock)  */
uint32_t SystemCoreClock = HSI_FREQ;    /*!< Processor Clock Frequency            */


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  SystemCoreClock = HSI_FREQ;
}

void mtp_wait_time_set()
{
  // Set MTP Wait cycles
#if defined(MTP_WAIT_1)
  CMSDK_MTPREG->MTP_CR = 0x00000001; // 16MHZ(RTL SIM)
#elif defined(MTP_WAIT_2)
  CMSDK_MTPREG->MTP_CR = 0x00000002; // 32MHZ(RTL SIM), 16MHZ(TIMING SIM)
#elif defined(MTP_WAIT_3)
  CMSDK_MTPREG->MTP_CR = 0x00000003; // 32MHZ(TIMING SIM), 16MHZ(TIMING SIM)
#elif defined(MTP_WAIT_4)
  CMSDK_MTPREG->MTP_CR = 0x00000004; // 8MHZ(TIMING SIM)
#elif defined(MTP_WAIT_5)
  CMSDK_MTPREG->MTP_CR = 0x00000005;
#elif defined(MTP_WAIT_6)
  CMSDK_MTPREG->MTP_CR = 0x00000006; // 16MHZ(TIMING SIM)
#elif defined(MTP_WAIT_7)
  CMSDK_MTPREG->MTP_CR = 0x00000007; // 32MHZ(TIMING SIM)
#elif defined(MTP_WAIT_0)
  CMSDK_MTPREG->MTP_CR = 0x00000000; // 8MHZ/4MHZ
#else
  // set to max clock cycle so that it can be run for timing simulation too.
  CMSDK_MTPREG->MTP_CR = 0x00000003;
#endif
}


void sys_clock_set(uint32_t freq)
{
	uint8_t mask = 2;
  CMSDK_SYSCON->HSI_CTRL = (CMSDK_SYSCON->HSI_CTRL & ~CMSDK_SYSCON_HSI_FREQ_Msk) | (mask << CMSDK_SYSCON_HSI_FREQ_Pos);
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  SystemCoreClock = HSI_FREQ;
	mtp_wait_time_set();
	sys_clock_set(HSI_FREQ);
}

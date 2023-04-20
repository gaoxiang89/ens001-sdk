/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm Limited or its affiliates.
 *
 *            (C) COPYRIGHT 2010-2013 Arm Limited or its affiliates.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Arm Limited or its affiliates.
 *
 *      SVN Information
 *
 *      Checked In          : $Date: 2017-10-10 15:55:38 +0100 (Tue, 10 Oct 2017) $
 *
 *      Revision            : $Revision: 371321 $
 *
 *      Release Information : Cortex-M System Design Kit-r1p1-00rel0
 *-----------------------------------------------------------------------------
 */

 /*

 UART functions for retargetting

 */

#include "ens001.h"


void UartStdOutInit(void)
{
#ifndef UART1_DRIVE
  CMSDK_UART0->DLL = 15;
	CMSDK_UART0->DLH = 0;
  ENS001_GPIO->ALTFL = (1<<6);
#else
  CMSDK_UART1->DLL = 0x0;
  ENS001_GPIO->ALTFL = (1<<26);
#endif
  return;
}
// Output a character
unsigned char UartPutc(unsigned char my_ch)
{
#ifndef UART1_DRIVE
  while ((CMSDK_UART0->LSR & CMSDK_UART_LSR_THR_EMPTY_Msk) == 0x0); // Wait if Transmit Holding register is full
  CMSDK_UART0->THR = my_ch; // write to transmit holding register
#else
  while ((CMSDK_UART1->LSR & CMSDK_UART_LSR_THR_EMPTY_Msk) == 0x0); // Wait if Transmit Holding register is full
  CMSDK_UART1->THR = my_ch; // write to transmit holding register
#endif
  return (my_ch);
}
// Get a character
/*unsigned char UartGetc(void)
{
  while ((CMSDK_UART2->STATE & 2)==0); // Wait if Receive Holding register is empty
  return (CMSDK_UART2->DATA);
}*/

void UartEndSimulation(void)
{
  UartPutc((char) 0x4); // End of simulation
  while(1);
}


/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//	File:		RTDM.h
//
// This program along with MPLAB DMCI ( MPLAB 8.10 or higher) create an alternative link
//between Host PC and target device for debugging applications in real-time.
//It is required to include the RTDM.C file and RTDM.h into the application project
//in order to send/receive data through the UART to/from the host PC running under
//MPLAB (release 8.10 or higher) DMCI environment.
// NOTE:DMCI included in MPLAB 8.10 or higher is ready and enabled to support data exchange
//between the host PC and target device. Previous versions of DMCI do not support this feature.
// NOTE: RTDM is currently supported by PIC24H, dsPIC30F, dsPIC33F and dsPIC33E processors
//
//
//	Written By:		M.Ellis, D. Torres,
//				Microchip Technology Inc
//
//
// The following files should be included in the MPLAB project:
//
//		RTDM.c			-- RTDM source code file
//		RTDM.h			-- RTDM header file
//		RTDMUSER.h		-- RTDM user definitions file
//		libpXXXX-coff.a		-- Your dsPIC/24H Peripheral Library
//		pXXFJXXX.gld		-- your dsPIC/24H Linker script file
//
//
/*****************************************************************************/
//
// Revision History
//
// 4/7/08  -- First Version Release
/****************************************************************************/

#include <string.h>
#include "uart.h"
#include "RTDMUSER.h"

#ifndef RTDM_H
#define RTDM_H

#if defined(__dsPIC33F__) || defined(__PIC24H__) || defined(__dsPIC33E__) || defined(__PIC24E__) || \
    defined(__dsPIC30F1010__) || defined(__dsPIC30F2020__) || defined(__dsPIC30F2023__)
#define RTDM_UART_V2
#elif defined(__dsPIC30F__)
#define RTDM_UART_V1
#endif

 #if defined RTDM_FCY
	#if defined RTDM_BAUDRATE
	 #define RTDM_BRG	(RTDM_FCY/(16*RTDM_BAUDRATE))-1
	#else
	  #error Cannot calculate BRG value. Please define RTDM_BAUDRATE in RTDMUSER.h file
	#endif
 #else
	 #error Cannot calculate RTDM_BRG value. Please define RTDM_FCY in RTDMUSER.h file
 #endif

 #define RTDM_BAUDRATE_ACTUAL	(RTDM_FCY/(16*(RTDM_BRG+1)))
 #define RTDM_BAUD_ERROR		((RTDM_BAUDRATE_ACTUAL > RTDM_BAUDRATE) ? RTDM_BAUDRATE_ACTUAL - RTDM_BAUDRATE : RTDM_BAUDRATE - RTDM_BAUDRATE_ACTUAL)
 #define RTDM_BAUD_ERROR_PERCENT	(((RTDM_BAUD_ERROR*100)+(RTDM_BAUDRATE/2))/RTDM_BAUDRATE)

 #if	(RTDM_BAUD_ERROR_PERCENT > 1)
	 #error The value loaded to the BRG register produces a baud rate error higher than 2%
 #endif


/**********************  RTDM FUNCTIONS **************************/
int RTDM_ProcessMsgs();
int RTDM_Close();
int RTDM_Start();
unsigned int RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);

#endif


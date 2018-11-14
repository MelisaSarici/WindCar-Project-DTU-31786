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
#ifndef _GENERAL_H
#define _GENERAL_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/* Defintions used by Button Scan function */
/* Status to indicate valid button press */
#define BTN_PRESSED     	2
/* Status value indicating that button is not ptressed */
#define BTN_NOT_PRESSED 	0
/* Status value to indicate wating for button debounce period */
#define BTN_DEBOUNCE        	1

/* General Definitions  */
/* Setting 1 as 'ON'*/
#define ON              	1
/* Setting 0 as 'OFF' */
#define OFF             	0
/* Setting 1 as 'SET' */
#define SET              	1
/* Setting 0 as 'RESET' */
#define RESET            	0
/* Setting 1 as 'ENABLE' */
#define ENABLE              	1
/* Setting 0 as 'DISABLE' */
#define DISABLE             	0
/* Setting 1 as 'HIGH' */
#define HIGH             	1
/* Setting 0 as 'LOW' */
#define LOW             	0
/* Setting 1 as 'TRUE' */
#define TRUE                    1
/* Setting 0 as 'FALSE' */
#define FALSE                   0
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* General syestem flag data type

  Description:
    This structure will host parameters related to application system flags.
 */
typedef union
{
    struct
    {
        /* Run motor indication */
        unsigned RunMotor:1;
        /* Open loop/clsd loop indication */
        unsigned OpenLoop:1;
        /* Indicates the button scan */
        unsigned Btn_Scan:1;
        /* Mode changed indication - from open to clsd loop */
        unsigned ChangeMode:1;
        /* Speed doubled indication */
        unsigned ChangeSpeed:1;
       /* Unused bits */
        unsigned    :10;
    } bits;
    uint16_t Word;
} UGF_T;
// *****************************************************************************
/* Buttton data type

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef union
{
    struct
    {
        unsigned status : 2;
        unsigned value : 1;
        unsigned :5;
        unsigned char debounce_count;
    } member;
    uint16_t  Word;
} BUTTON_T;

// *****************************************************************************
// *****************************************************************************
// Section: C/API Interface Routines
// *****************************************************************************
// *****************************************************************************
#define Q15(Float_Value)	\
((Float_Value < 0.0) ? (int16_t)(32768 * (Float_Value) - 0.5) \
: (int16_t)(32767 * (Float_Value) + 0.5))

int16_t Q15SQRT(int16_t );
void Delay(uint16_t t);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of GENERAL_H


/* EOF */
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
#ifndef _READADC_H
#define _READADC_H
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
// Section: Data types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Read ADC Parameter data type

  Description:
    This structure will host parameters used by Read ADC function
    parameters.
 */
typedef struct
{
    int16_t qK;                 
    int16_t qADValue;           
    int16_t qAnRef;		
} READ_ADC_PARM_T;

// *****************************************************************************
// *****************************************************************************
// Section: C/API Interface Routines
// *****************************************************************************
// *****************************************************************************
/* Returns unsigned value 0 -> 2*iK */
void ReadADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm );
/* Returns signed value -2*iK -> 2*iK */
void ReadSignedADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm ); 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of READADC_H


/* EOF */
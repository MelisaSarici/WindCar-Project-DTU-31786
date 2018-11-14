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
#ifndef _FDWEAK_H
#define _FDWEAK_H

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
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Fead weakening Parameter data type

  Description:
    This structure will host parameters related to field weakening function.
 */
typedef struct
{
    /* d-current reference */
    int16_t qIdRef;
    /* Flux weakening on speed -*/
    int16_t qFwOnSpeed;
    /* Lookup tables index */
    int16_t qIndex;
    /* Curve for magnetizing current variation with speed */
    int16_t qFwCurve[18];
    /* Curve for InvKfi constant InvKfi = Omega/BEMF variation with speed */
    int16_t qInvKFiCurve[18];
    /* Curve for Ls variation with speed */
    int16_t qLsCurve[18];    
} FDWEAK_PARM_T;

extern FDWEAK_PARM_T tFdWeakParmA;

// *****************************************************************************
// *****************************************************************************
// Section: C/API Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitFWParams();
int FieldWeakening( int qMotorSpeed );

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of FDWEAK_H


/* EOF */
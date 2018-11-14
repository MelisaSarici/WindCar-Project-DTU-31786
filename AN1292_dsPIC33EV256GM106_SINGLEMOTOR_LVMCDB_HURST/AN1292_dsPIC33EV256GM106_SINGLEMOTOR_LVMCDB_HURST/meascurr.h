#ifndef MEASCURR_H
#define MEASCURR_H

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
/* Measure Current Parameter data type

  Description:
    This structure will host parameters related to measured currents
 */
typedef struct
{
    int16_t   qKa;        
    int16_t   Offseta;
    int16_t   qKb;        
    int16_t   Offsetb;
    int16_t   qIa;
    int16_t   qIb;
} MEAS_CURR_PARM_T;
/* current measurement params */
extern MEAS_CURR_PARM_T tMeasCurrParmA;         
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void MeasCompCurr(int16_t MeasuredIa,int16_t MeasuredIb,
                            MEAS_CURR_PARM_T *pMeasCurrParm);
void InitMeasCompCurr( int16_t Offseta, int16_t Offsetb,
                            MEAS_CURR_PARM_T *pMeasCurrParm);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of MEASCURR_H


/* EOF */
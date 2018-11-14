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
#ifndef _CONTROL_H
#define _CONTROL_H

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
/* Control Parameter data type

  Description:
    This structure will host parameters related to application control
    parameters.
 */
typedef struct
{
    /* Reference velocity */
    int16_t   qVelRef;
    /* Vd flux reference value */
    int16_t   qVdRef;
    /* Vq torque reference value */
    int16_t   qVqRef;
    /* Ramp for speed reference value */
    int16_t   qRefRamp;
    /* Speed of the ramp */
    int16_t   qDiff;	  
} CTRL_PARM_T;
/* Motor Parameter data type

  Description:
    This structure will host parameters related to motor parameters.
*/
typedef struct
{
    /* Start up ramp in open loop. */
    uint32_t startup_Ramp;
    /* counter that is incremented in CalculateParkAngle() up to LOCK_TIME,*/
    uint16_t startup_Lock;
    /* Start up ramp increment */
    uint16_t tuning_add_rampup;	
    uint16_t tuning_delay_rampup;
} MOTOR_PARM_T;

CTRL_PARM_T tCtrlParmA;
MOTOR_PARM_T tMotorParmA;

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of _CONTROL_H


/* EOF */
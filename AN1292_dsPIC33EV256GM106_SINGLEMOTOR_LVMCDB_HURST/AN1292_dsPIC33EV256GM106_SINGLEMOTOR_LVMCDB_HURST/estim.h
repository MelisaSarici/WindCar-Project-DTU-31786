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
#ifndef _ESTIM_H
#define _ESTIM_H

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
#include "motor_control.h"
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Estimator Parameter data type

  Description:
    This structure will host parameters related to angle/speed estimator
    parameters.
 */
typedef struct
{
    /* Integration constant */
    int16_t qDeltaT;
    /* angle of estimation */
    int16_t qRho;
    /* internal variable for angle */
    int32_t qRhoStateVar;
    /* primary speed estimation */
    int16_t qOmegaMr;
    /* last value for Ialpha */
    int16_t qLastIalpha;
    /* last value for Ibeta */
    int16_t qLastIbeta;
    /* difference Ialpha */
    int16_t qDIalpha;
    /* difference Ibeta */
    int16_t qDIbeta;
    /* BEMF alpha */
    int16_t qEsa;
    /* BEMF beta */
    int16_t qEsb;
    /* BEMF d */
    int16_t qEsd;
    /* BEMF q */
    int16_t qEsq;
    /* counter in Last DI tables */
    int16_t qDiCounter;
    /* dI*Ls/dt alpha */
    int16_t qVIndalpha;
    /* dI*Ls/dt beta */
    int16_t qVIndbeta;
    /* BEMF d filtered */
    int16_t qEsdf;
    /* state var for BEMF d Filtered */
    int32_t qEsdStateVar;
    /* BEMF q filtered */
    int16_t qEsqf;
    /* state var for BEMF q Filtered */
    int32_t qEsqStateVar;
    /* filter constant for d-q BEMF */
    int16_t qKfilterEsdq;
    /* Estimated speed */
    int16_t qVelEstim;
    /* Filter Konstant for Estimated speed */
    int16_t qVelEstimFilterK;
    /* State Variable for Estimated speed */
    int32_t qVelEstimStateVar;
    /* Value from last control step Ialpha */
    int16_t qLastValpha;
    /* Value from last control step Ibeta */
    int16_t qLastVbeta;
    /* dIalphabeta/dt */
    int16_t qDIlimitLS;
    /* dIalphabeta/dt */
    int16_t qDIlimitHS;
    /*  last  value for Ialpha */
    int16_t qLastIalphaHS[8];
    /* last  value for Ibeta */
    int16_t qLastIbetaHS[8];
    /* estima angle init offset */
    int16_t qRhoOffset;

} ESTIM_PARM_T;
/* Motor Estimator Parameter data type

  Description:
    This structure will host motor parameters parameters required by angle
    estimaotr.
 */
typedef struct
{
    /* Rs value - stator resistance */
    int16_t qRs;
    /* Ls/dt value - stator inductand / dt - variable with speed */
    int16_t qLsDt;
    /* Ls/dt value - stator inductand / dt for base speed (nominal) */
    int16_t qLsDtBase;
    /* InvKfi constant value ( InvKfi = Omega/BEMF ) */
    int16_t qInvKFi;
    /* InvKfi constant - base speed (nominal) value */
    int16_t qInvKFiBase;            
} MOTOR_ESTIM_PARM_T;

extern ESTIM_PARM_T tEstimParmA;
extern MOTOR_ESTIM_PARM_T tMotorEstimParmA;
extern MC_ALPHABETA_T tClarkVoltageA,tClarkCurrentA;
extern MC_ALPHABETA_T tClarkVoltageA,tClarkCurrentA;
extern MC_SINCOS_T tSinCosThetaA;
extern MC_DQ_T tParkVoltageA,tParkCurrentA;
extern MC_DUTYCYCLEOUT_T tPWMDutyValueA;
extern MC_ABC_T   tPhaseVoltagesA,tPhaseCurrentsA;
// *****************************************************************************
// *****************************************************************************
// Section: C/API Interface Routines
// *****************************************************************************
// *****************************************************************************
void Estim(void);
void InitEstimParm(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ESTIM_H


/* EOF */
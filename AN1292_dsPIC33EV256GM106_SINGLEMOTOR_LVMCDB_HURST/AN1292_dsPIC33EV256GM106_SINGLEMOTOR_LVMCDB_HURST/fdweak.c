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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__
#include    "general.h"
#include    "fdweak.h"
#include    "estim.h"
#include    "user_parms.h"
// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
FDWEAK_PARM_T  tFdWeakParmA;
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define FWONSPEED_A NOMINAL_SPEED_RPM*NOPOLESPAIRS
// *****************************************************************************
/* Function:
    InitFWParams()

  Summary:
    Initialises field weakening parameters

  Description:
    This routine initilases field weakening structure variables

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitFWParams(void)
{

    /* Field Weakening constant for constant torque range */
    /* Flux reference value */
    tFdWeakParmA.qIdRef = IDREF_BASESPEED;     
    /* Startspeed for Fieldweakening  */
    tFdWeakParmA.qFwOnSpeed=FWONSPEED_A-(1<<SPEED_INDEX_CONST);		

    /* Initialize magnetizing curve values */
    tFdWeakParmA.qFwCurve[0]	= IDREF_SPEED0;
    tFdWeakParmA.qFwCurve[1]	= IDREF_SPEED1;
    tFdWeakParmA.qFwCurve[2]	= IDREF_SPEED2;
    tFdWeakParmA.qFwCurve[3]	= IDREF_SPEED3;
    tFdWeakParmA.qFwCurve[4]	= IDREF_SPEED4;
    tFdWeakParmA.qFwCurve[5]	= IDREF_SPEED5;
    tFdWeakParmA.qFwCurve[6]	= IDREF_SPEED6;
    tFdWeakParmA.qFwCurve[7]	= IDREF_SPEED7;
    tFdWeakParmA.qFwCurve[8]	= IDREF_SPEED8;
    tFdWeakParmA.qFwCurve[9]	= IDREF_SPEED9;
    tFdWeakParmA.qFwCurve[10]	= IDREF_SPEED10;
    tFdWeakParmA.qFwCurve[11]	= IDREF_SPEED11;
    tFdWeakParmA.qFwCurve[12]	= IDREF_SPEED12;
    tFdWeakParmA.qFwCurve[13]	= IDREF_SPEED13;
    tFdWeakParmA.qFwCurve[14]	= IDREF_SPEED14;
    tFdWeakParmA.qFwCurve[15]	= IDREF_SPEED15;
    tFdWeakParmA.qFwCurve[16]	= IDREF_SPEED16;
    tFdWeakParmA.qFwCurve[17]	= IDREF_SPEED17;


    /* Init inverse Kfi curve values */
    tFdWeakParmA.qInvKFiCurve[0] = INVKFI_SPEED0;
    tFdWeakParmA.qInvKFiCurve[1] = INVKFI_SPEED1;
    tFdWeakParmA.qInvKFiCurve[2] = INVKFI_SPEED2;
    tFdWeakParmA.qInvKFiCurve[3] = INVKFI_SPEED3;
    tFdWeakParmA.qInvKFiCurve[4] = INVKFI_SPEED4;
    tFdWeakParmA.qInvKFiCurve[5] = INVKFI_SPEED5;
    tFdWeakParmA.qInvKFiCurve[6] = INVKFI_SPEED6;
    tFdWeakParmA.qInvKFiCurve[7] = INVKFI_SPEED7;
    tFdWeakParmA.qInvKFiCurve[8] = INVKFI_SPEED8;
    tFdWeakParmA.qInvKFiCurve[9] = INVKFI_SPEED9;
    tFdWeakParmA.qInvKFiCurve[10] = INVKFI_SPEED10;
    tFdWeakParmA.qInvKFiCurve[11] = INVKFI_SPEED11;
    tFdWeakParmA.qInvKFiCurve[12] = INVKFI_SPEED12;
    tFdWeakParmA.qInvKFiCurve[13] = INVKFI_SPEED13;
    tFdWeakParmA.qInvKFiCurve[14] = INVKFI_SPEED14;
    tFdWeakParmA.qInvKFiCurve[15] = INVKFI_SPEED15;
    tFdWeakParmA.qInvKFiCurve[16] = INVKFI_SPEED16;
    tFdWeakParmA.qInvKFiCurve[17] = INVKFI_SPEED17;

    /* Init Ls variation curve */
    tFdWeakParmA.qLsCurve[0]= LS_OVER2LS0_SPEED0;
    tFdWeakParmA.qLsCurve[1]= LS_OVER2LS0_SPEED1;
    tFdWeakParmA.qLsCurve[2]= LS_OVER2LS0_SPEED2;
    tFdWeakParmA.qLsCurve[3]= LS_OVER2LS0_SPEED3;
    tFdWeakParmA.qLsCurve[4]= LS_OVER2LS0_SPEED4;
    tFdWeakParmA.qLsCurve[5]= LS_OVER2LS0_SPEED5;
    tFdWeakParmA.qLsCurve[6]= LS_OVER2LS0_SPEED6;
    tFdWeakParmA.qLsCurve[7]= LS_OVER2LS0_SPEED7;
    tFdWeakParmA.qLsCurve[8]= LS_OVER2LS0_SPEED8;
    tFdWeakParmA.qLsCurve[9]= LS_OVER2LS0_SPEED9;
    tFdWeakParmA.qLsCurve[10]= LS_OVER2LS0_SPEED10;
    tFdWeakParmA.qLsCurve[11]= LS_OVER2LS0_SPEED11;
    tFdWeakParmA.qLsCurve[12]= LS_OVER2LS0_SPEED12;
    tFdWeakParmA.qLsCurve[13]= LS_OVER2LS0_SPEED13;
    tFdWeakParmA.qLsCurve[14]= LS_OVER2LS0_SPEED14;
    tFdWeakParmA.qLsCurve[15]= LS_OVER2LS0_SPEED15;
    tFdWeakParmA.qLsCurve[16]= LS_OVER2LS0_SPEED16;
    tFdWeakParmA.qLsCurve[17]= LS_OVER2LS0_SPEED17;

}
// *****************************************************************************
/* Function:
    FieldWeakening()

  Summary:
    Routine impliments field weakening

  Description:
    Function caluclates the Id reference based on the motor speed

  Precondition:
    None.

  Parameters:
    Motor Speed

  Returns:
    Id reference.

  Remarks:
    None.
 */
int16_t	FieldWeakening(int16_t qMotorSpeed)
{
    int16_t iTempInt1, iTempInt2;

    int16_t qInvKFi;
    int16_t qLsDt;

    /* LsDt value - for base speed */
    qLsDt=tMotorEstimParmA.qLsDtBase;

    /* If the speed is less than one for activating the FW */
    if (qMotorSpeed <= tFdWeakParmA.qFwOnSpeed)
    {
        /* Set Idref as first value in magnetizing curve */
        tFdWeakParmA.qIdRef = tFdWeakParmA.qFwCurve[0];

        /* Adapt fileter parameter */
        tEstimParmA.qKfilterEsdq = KFILTER_ESDQ;

        /* Inverse Kfi constant for base speed */
        qInvKFi = tMotorEstimParmA.qInvKFiBase;
    }
    else
    {
        /* Get the index parameter */
        /* Index in FW-Table */
        tFdWeakParmA.qIndex =(qMotorSpeed - tFdWeakParmA.qFwOnSpeed) >> SPEED_INDEX_CONST;

        iTempInt1 = tFdWeakParmA.qFwCurve[tFdWeakParmA.qIndex]-
                    tFdWeakParmA.qFwCurve[tFdWeakParmA.qIndex+1];
        iTempInt2 = (tFdWeakParmA.qIndex<<SPEED_INDEX_CONST)+
                     tFdWeakParmA.qFwOnSpeed;
        iTempInt2 = qMotorSpeed - iTempInt2;

        /* Interpolation betwen two results from the Table */
        tFdWeakParmA.qIdRef = tFdWeakParmA.qFwCurve[tFdWeakParmA.qIndex]-
                                (int16_t)(__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);

        /* Adapt filer parameter */
        tEstimParmA.qKfilterEsdq = KFILTER_ESDQ_FW;

        /* Interpolation betwen two results from the Table */
        iTempInt1  = tFdWeakParmA.qInvKFiCurve[tFdWeakParmA.qIndex]-
                     tFdWeakParmA.qInvKFiCurve[tFdWeakParmA.qIndex+1];

        qInvKFi = tFdWeakParmA.qInvKFiCurve[tFdWeakParmA.qIndex] -
                  (int16_t)(__builtin_mulss(iTempInt1, iTempInt2)>> SPEED_INDEX_CONST);


        /* Interpolation betwen two results from the Table */
        iTempInt1 = tFdWeakParmA.qLsCurve[tFdWeakParmA.qIndex]-
                    tFdWeakParmA.qLsCurve[tFdWeakParmA.qIndex+1];

        iTempInt1 = tFdWeakParmA.qLsCurve[tFdWeakParmA.qIndex] -
                    (int16_t)(__builtin_mulss(iTempInt1,iTempInt2)>>SPEED_INDEX_CONST);

        /* Lsdt = Lsdt0*Ls/Ls0 */
        qLsDt = (int16_t) ( __builtin_mulss(qLsDt,iTempInt1)>>14);
    }

    tMotorEstimParmA.qInvKFi = qInvKFi;
    tMotorEstimParmA.qLsDt = qLsDt;
    return tFdWeakParmA.qIdRef;
}


/* EOF */
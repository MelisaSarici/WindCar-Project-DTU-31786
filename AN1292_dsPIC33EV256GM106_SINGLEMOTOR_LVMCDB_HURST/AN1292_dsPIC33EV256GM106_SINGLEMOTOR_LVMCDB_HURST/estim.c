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
#include "general.h"
#include "motor_control.h"
#include "estim.h"
#include "user_parms.h"
/* q15 abs function use */
#include <libq.h> 

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
#define DECIMATE_NOMINAL_SPEED_A NOMINAL_SPEED_RPM*NOPOLESPAIRS/10
#define NOMINAL_ELECTRICAL_SPEED_A NOMINAL_SPEED_RPM*NOPOLESPAIRS
// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************

MC_SINCOS_T	tSincosParm;
MC_ALPHABETA_T tClarkBEMF;
MC_DQ_T tParkBEMF;
MC_SINCOS_T tSinCosTheta;

// *****************************************************************************
/* Function:
    Estim()

  Summary:
    Motor speed and angle estimator

  Description:
    Estimation of the speed of the motor and field angle based on inverter
    voltages and motor currents.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Estim(void)
{
    int32_t tempint;
    uint16_t Index=(tEstimParmA.qDiCounter-7)&0x0007;

    /* dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
       for lower speed the granularity of differnce is higher - the
       difference is made between 2 sampled values @ 8 ADC ISR cycles */
    if (_Q15abs(tEstimParmA.qVelEstim)<NOMINAL_ELECTRICAL_SPEED_A)
    {

        tEstimParmA.qDIalpha	= (tClarkCurrentA.alpha-
                                   tEstimParmA.qLastIalphaHS[Index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (tEstimParmA.qDIalpha>tEstimParmA.qDIlimitLS)
        {
            tEstimParmA.qDIalpha=tEstimParmA.qDIlimitLS;
        }
        if (tEstimParmA.qDIalpha<-tEstimParmA.qDIlimitLS)
        {
            tEstimParmA.qDIalpha=-tEstimParmA.qDIlimitLS;
        }
        tEstimParmA.qVIndalpha = (int16_t)(__builtin_mulss(tMotorEstimParmA.qLsDt, tEstimParmA.qDIalpha)>>10);

        tEstimParmA.qDIbeta	=(tClarkCurrentA.beta-tEstimParmA.qLastIbetaHS[Index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR cycle
           the following limitation assures a limitation per low speed - up to
           the nominal speed */
        if (tEstimParmA.qDIbeta>tEstimParmA.qDIlimitLS)
        {
            tEstimParmA.qDIbeta=tEstimParmA.qDIlimitLS;
        }
        if (tEstimParmA.qDIbeta<-tEstimParmA.qDIlimitLS)
        {
            tEstimParmA.qDIbeta=-tEstimParmA.qDIlimitLS;
        }
        tEstimParmA.qVIndbeta = (int16_t)(__builtin_mulss(tMotorEstimParmA.qLsDt, tEstimParmA.qDIbeta)>>10);

    }
    else
    {

        tEstimParmA.qDIalpha = (tClarkCurrentA.alpha -
                                 tEstimParmA.qLastIalphaHS[(tEstimParmA.qDiCounter)]);
        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (tEstimParmA.qDIalpha>tEstimParmA.qDIlimitHS)
        {
            tEstimParmA.qDIalpha=tEstimParmA.qDIlimitHS;
        }
        if (tEstimParmA.qDIalpha<-tEstimParmA.qDIlimitHS)
        {
            tEstimParmA.qDIalpha=-tEstimParmA.qDIlimitHS;
        }
        tEstimParmA.qVIndalpha = (int16_t)(__builtin_mulss(tMotorEstimParmA.qLsDt, tEstimParmA.qDIalpha)>>7);

        tEstimParmA.qDIbeta = (tClarkCurrentA.beta -
                                tEstimParmA.qLastIbetaHS[(tEstimParmA.qDiCounter)]);

        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (tEstimParmA.qDIbeta>tEstimParmA.qDIlimitHS)
        {
            tEstimParmA.qDIbeta=tEstimParmA.qDIlimitHS;
        }
        if (tEstimParmA.qDIbeta<-tEstimParmA.qDIlimitHS)
        {
            tEstimParmA.qDIbeta=-tEstimParmA.qDIlimitHS;
        }
        tEstimParmA.qVIndbeta= (int16_t)(__builtin_mulss(tMotorEstimParmA.qLsDt, tEstimParmA.qDIbeta)>>7);

    }

    /* Update  LastIalpha and LastIbeta */
    tEstimParmA.qDiCounter = (tEstimParmA.qDiCounter+1) & 0x0007;
    tEstimParmA.qLastIalphaHS[tEstimParmA.qDiCounter]	= tClarkCurrentA.alpha;
    tEstimParmA.qLastIbetaHS[tEstimParmA.qDiCounter] 	= tClarkCurrentA.beta;

    /* Stator voltage eqations
     Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
     BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt */

    tClarkBEMF.alpha = tClarkVoltageA.alpha -
                        (int16_t)(__builtin_mulss( tMotorEstimParmA.qRs, tClarkCurrentA.alpha)	>>14)-
                        tEstimParmA.qVIndalpha;

    /* The multiplication between the Rs and Ialpha was shifted by 14 instead
       of 15 because the Rs value normalized exceeded Q15 range, so it was
       divided by 2 immediatelky after the normalization - in userparms.h */

    /* Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
       BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt */
    tClarkBEMF.beta = tClarkVoltageA.beta -
                       (int16_t)(__builtin_mulss( tMotorEstimParmA.qRs, tClarkCurrentA.beta )	>>14)-
                       tEstimParmA.qVIndbeta;

    /* The multiplication between the Rs and Ibeta was shifted by 14 instead of 15
     because the Rs value normalized exceeded Q15 range, so it was divided by 2 
     immediatelky after the normalization - in userparms.h */


    MC_CalculateSineCosine_Assembly_Ram((tEstimParmA.qRho + tEstimParmA.qRhoOffset),
                                        &tSinCosTheta);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&tClarkBEMF,&tSinCosTheta,&tParkBEMF);

    /* Filter first order for Esd and Esq
       EsdFilter = 1/TFilterd * Intergal{ (Esd-EsdFilter).dt } */
    tempint = (int16_t)(tParkBEMF.d - tEstimParmA.qEsdf);
    tEstimParmA.qEsdStateVar += __builtin_mulss(tempint, tEstimParmA.qKfilterEsdq);
    tEstimParmA.qEsdf = (int16_t)(tEstimParmA.qEsdStateVar>>15);

    tempint = (int16_t)(tParkBEMF.q - tEstimParmA.qEsqf);
    tEstimParmA.qEsqStateVar += __builtin_mulss(tempint, tEstimParmA.qKfilterEsdq);
    tEstimParmA.qEsqf = (int16_t)(tEstimParmA.qEsqStateVar>>15);

    /* OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf)
       For stability the conditio for low speed */
    if (_Q15abs(tEstimParmA.qVelEstim) > DECIMATE_NOMINAL_SPEED_A)
    {
        if(tEstimParmA.qEsqf>0)
        {
            tempint = (int16_t)(tEstimParmA.qEsqf- tEstimParmA.qEsdf);
            tEstimParmA.qOmegaMr = (int16_t)(__builtin_mulss(tMotorEstimParmA.qInvKFi, tempint)>>15);
        }
        else
        {
            tempint = (int16_t)(tEstimParmA.qEsqf + tEstimParmA.qEsdf);
            tEstimParmA.qOmegaMr = (int16_t)(__builtin_mulss(tMotorEstimParmA.qInvKFi, tempint)>>15);
        }
    }
    /* if est speed<10% => condition VelRef<>0 */
    else 
    {
        if(tEstimParmA.qVelEstim>0)
        {
            tempint = (int16_t)(tEstimParmA.qEsqf - tEstimParmA.qEsdf);
            tEstimParmA.qOmegaMr = (int16_t)(__builtin_mulss(tMotorEstimParmA.qInvKFi,tempint)>>15);
        }
        else
        {
            tempint = (int16_t)(tEstimParmA.qEsqf + tEstimParmA.qEsdf);
            tEstimParmA.qOmegaMr = (int16_t)(__builtin_mulss(tMotorEstimParmA.qInvKFi,tempint)>>15);
        }
    }
    /* the result of the calculation above is shifted left by one because
       initally the value of InvKfi was shifted by 2 after normalizing -
       assuring that extended range of the variable is possible in the
       lookup table the initial value of InvKfi is defined in userparms.h */
    tEstimParmA.qOmegaMr = tEstimParmA.qOmegaMr<<1;


    /* the integral of the angle is the estimated angle */
    tEstimParmA.qRhoStateVar += __builtin_mulss(tEstimParmA.qOmegaMr, tEstimParmA.qDeltaT);
    tEstimParmA.qRho = (int16_t) (tEstimParmA.qRhoStateVar>>15);


    /* The estiamted speed is a filter value of the above calculated OmegaMr.
       The filter implementation is the same as for BEMF d-q components
       filtering */
    tempint = (int16_t)(tEstimParmA.qOmegaMr - tEstimParmA.qVelEstim);
    tEstimParmA.qVelEstimStateVar += __builtin_mulss(tempint, tEstimParmA.qVelEstimFilterK);
    tEstimParmA.qVelEstim = (int16_t)(tEstimParmA.qVelEstimStateVar>>15);

}
// *****************************************************************************
/* Function:
    InitEstimParm ()

  Summary:
    Initialises Motor speed and angle estimator parameters

  Description:
    Initialisation of the parameters of the estimator.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void	InitEstimParm(void)  
{
    /* Constants are defined in usreparms.h */

    tMotorEstimParmA.qLsDtBase = NORM_LSDTBASE;
    tMotorEstimParmA.qLsDt = tMotorEstimParmA.qLsDtBase;
    tMotorEstimParmA.qRs = NORM_RS;

    tMotorEstimParmA.qInvKFiBase = NORM_INVKFIBASE;
    tMotorEstimParmA.qInvKFi = tMotorEstimParmA.qInvKFiBase;

    tEstimParmA.qRhoStateVar=0;
    tEstimParmA.qOmegaMr=0;
    tEstimParmA.qDiCounter=0;
    tEstimParmA.qEsdStateVar=0;
    tEstimParmA.qEsqStateVar=0;

    tEstimParmA.qDIlimitHS = D_ILIMIT_HS;
    tEstimParmA.qDIlimitLS = D_ILIMIT_LS;

    tEstimParmA.qKfilterEsdq = KFILTER_ESDQ;
    tEstimParmA.qVelEstimFilterK = KFILTER_VELESTIM;

    tEstimParmA.qDeltaT = NORM_DELTAT;
    tEstimParmA.qRhoOffset = INITOFFSET_TRANS_OPEN_CLSD;

}


/* EOF */
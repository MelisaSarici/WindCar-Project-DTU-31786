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

#include <libq.h>      
#include "general.h"   
#include "motor_control.h"
#include "user_parms.h" 
#include "periph.h"   
#include "readadc.h"   
#include "meascurr.h"  
#include "control.h"   
#include "estim.h"
#include "fdweak.h"
#include <uart.h>
#include "MCP802x_DE2.h"   
#include "RTDM.h"

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
/******************************************************************************/
volatile UGF_T tUGFlagsA;
ESTIM_PARM_T tEstimParmA;
MOTOR_ESTIM_PARM_T tMotorEstimParmA;
MEAS_CURR_PARM_T tMeasCurrParmA;   
READ_ADC_PARM_T tReadADCParmA;   
volatile int16_t qRotorAngleMA = 0,qAngleOpenloopA = 0;
uint16_t iPWMPeriodA;
uint16_t iLoopCount = 0;
BUTTON_T tBtnStartStopA,tBtnHalfDoubleA;
ADC_OFFSET_T iADC1OffsetA;
uint16_t iADCisrCnt = 0;
/******************* Library Struct & Variable defines *******************/
MC_ALPHABETA_T tClarkVoltageA,tClarkCurrentA;
MC_SINCOS_T tSinCosThetaA;
MC_DQ_T tParkVoltageA,tParkCurrentA;
MC_DUTYCYCLEOUT_T tPWMDutyValueA;
MC_ABC_T   tPhaseVoltagesA,tPhaseCurrentsA;

MC_PIPARMIN_T tPIParmInIQA;
MC_PIPARMOUT_T tPIParmOutIQA;
MC_PIPARMIN_T tPIParmInIDA;
MC_PIPARMOUT_T tPIParmOutIDA;
MC_PIPARMIN_T tPIParmInWA;
MC_PIPARMOUT_T tPIParmOutWA;

/******************** Variables used for RTDM/DMCI Commnication ***************/
#ifdef RTDM
/* Buffer to store the data samples for the DMCI data viewer Graph1 */
volatile int16_t RecorderBuffer1[DATA_BUFFER_SIZE];
/* Buffer to store the data samples for the DMCI data viewer Graph2 */
volatile int16_t RecorderBuffer2[DATA_BUFFER_SIZE];
/* Buffer to store the data samples for the DMCI data viewer Graph3 */
volatile int16_t RecorderBuffer3[DATA_BUFFER_SIZE];
/* Buffer to store the data samples for the DMCI data viewer Graph4 */
volatile int16_t RecorderBuffer4[DATA_BUFFER_SIZE];

/* Tail pointer for the DMCI Graph1 */
volatile int16_t * PtrRecBuffer1 = &RecorderBuffer1[0];
/* Tail pointer for the DMCI Graph2 */
volatile int16_t * PtrRecBuffer2 = &RecorderBuffer2[0];
/* Tail pointer for the DMCI Graph3 */
volatile int16_t * PtrRecBuffer3 = &RecorderBuffer3[0];
/* Tail pointer for the DMCI Graph4 */
volatile int16_t * PtrRecBuffer4 = &RecorderBuffer4[0];

/* Buffer Recorder Upper Limit */
volatile int16_t * RecBuffUpperLimit = RecorderBuffer1 + DATA_BUFFER_SIZE - 1;

typedef struct DMCIFlags
{
    /* Flag needs to be set to start buffering data */
    unsigned Recorder : 1;
    unsigned StartStop : 1;
    unsigned unused : 14;
} DMCIFLAGS;
volatile DMCIFLAGS DMCIFlags;

volatile int16_t SnapCount = 0;
volatile int16_t SnapShotDelayCnt = 0;
volatile int16_t SnapShotDelay = SNAPDELAY;
volatile int16_t SpeedReference = 32767;

#endif // End of #ifdef RTDM
   
// *****************************************************************************
// *****************************************************************************
// Section: Local Definitions
// *****************************************************************************
// *****************************************************************************
/* Maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*NOPOLESPAIRS
/* Nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*NOPOLESPAIRS

/* End speed conveted to fit the startup ramp */
#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0)*1024
/* End speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM*NOPOLESPAIRS
/* Number of control loops that must execute before the button routine is executed.
   button polling loop period in sec */
#define	DISPLOOPTIME_SEC	0.001
#define	DISPLOOPTIME            (DISPLOOPTIME_SEC/LOOPTIME_SEC)
/* Maximum pot value in sfrac mode 0xFFC0
shifted left with 1 meaning 16368 */ 
#define  MAXPOTVAL_SHL1         16368

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void __attribute__((__interrupt__)) _AD1Interrupt(void);
void InitControlParameters(void);
void DoControl( void );
void CalculateParkAngle(void);
void ButtonScan(BUTTON_T *);
void ResetParmsA(void);
void InitPeripherals(void);

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system init function group containing the
    buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{

    InitOscillator();

    SetupGPIOPorts();

    CORCONbits.SATA  = 0;

    /* Setting CE_A = 0 */
    CE_A_MCP802x = DISABLE;

    DelayinMicrosecond(1);
    
    /* Setting CE_A = 1 */
    CE_A_MCP802x = ENABLE;

   /* Init PI control parameters */
    InitControlParameters();

    /* Init estim parameters */
    InitEstimParm();

    /* Init flux weakening params */
    InitFWParams();

    /* Configure MCP8024 */
    ConfigureMcp8024(DE2_UART_MCP8024_A);

    /* Debug initiation - start services */
    #ifdef RTDM
        RTDM_Start();
        DMCIFlags.StartStop = 0;
        DMCIFlags.Recorder = 0;
    #endif

    /* Initialise Peripherals */
    InitPeripherals();

    while(1)
    {
        /* Clear flags */
        tUGFlagsA.Word = 0;                   
     
        /* Init PI control parameters */
   	InitControlParameters();        
        /* Init estim parameters */
        InitEstimParm();
        /* Init flux weakening params */
        InitFWParams();
        /* Reset parameters used for running motor through Inverter A*/
        ResetParmsA();

        while(1)
        {
            #ifdef RTDM
               RTDM_ProcessMsgs();
            #endif
            
            /* Button scanning loop for Button 1 to start Motor A */
            tBtnStartStopA.member.value = BTN_START_STOP_A;
            ButtonScan(&tBtnStartStopA);
            if(tBtnStartStopA.member.status == BTN_PRESSED)
            {
                tBtnStartStopA.member.status = BTN_NOT_PRESSED;
                if(tUGFlagsA.bits.RunMotor == 1)
                {
                    ResetParmsA();
                }
                else
                {
                    tUGFlagsA.bits.RunMotor = 1;
                }

            }
            if(tUGFlagsA.bits.RunMotor == 1)
            {
                /* Button scanning loop for SW2 to enter into filed
                   weakening mode */
                tBtnHalfDoubleA.member.value = BTN_SPD_HALF_DBL_A;
                ButtonScan(&tBtnHalfDoubleA);
                if(tBtnHalfDoubleA.member.status == BTN_PRESSED)
                {
                    tBtnHalfDoubleA.member.status = BTN_NOT_PRESSED;
                    tUGFlagsA.bits.ChangeSpeed = !tUGFlagsA.bits.ChangeSpeed;

                }
            }
            if (iADCisrCnt ==  30000)
            {
                iADCisrCnt =  0;
                if(tUGFlagsA.bits.RunMotor == 1)
                {
                   /* To verify the DE2 status and take action */
                    StatusCheckMcp802x(DE2_UART_MCP8024_A);
                }
            }
        }

    } // End of Main loop

    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    Button_Scan()

  Summary:
    Routine to scan the push buttons on the board

  Description:
    Function scans the pushbutton and indicates the status in the structure

  Precondition:
    None.

  Parameters:
    pointer to button structure

  Returns:
    None.

  Remarks:
    None.
 */
void ButtonScan(BUTTON_T *pButton)
{
    if(pButton->member.value == 0)
    {
        if(pButton->member.debounce_count < BTN_DEBOUNCE_COUNT)
        {
            pButton->member.debounce_count++;
            pButton->member.status = BTN_DEBOUNCE;
        }
    }
    else
    {
        if(pButton->member.debounce_count < BTN_DEBOUNCE_COUNT)
        {
            pButton->member.status = BTN_NOT_PRESSED;
        }
        else
        {
           pButton->member.status = BTN_PRESSED;
        }
        pButton->member.debounce_count = 0;
    }
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the paramenters requuired for Motor through Inv-A

  Description:
    Reintilises the duty cyle,resets all the ocunters etc when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmsA(void)
{
    /* Re initialise the duty cycle to minimum value */
    INVERTERA_PWM_PDC1 = MIN_DUTY;
    INVERTERA_PWM_PDC2 = MIN_DUTY;
    INVERTERA_PWM_PDC3 = MIN_DUTY;

    /* Make sure adc generate interrupt while initialising parameters */
    DisableADC1Interrupt();

    /* Stop the motor   */
    tUGFlagsA.bits.RunMotor = 0;        
    tBtnStartStopA.Word = 0;
    tBtnHalfDoubleA.Word = 0;
    /* Set the reference speed value to 0 */
    tCtrlParmA.qVelRef = 0;
    /* Restart in open loop */
    tUGFlagsA.bits.OpenLoop = 1;
    /* Disables the button scan */
    tUGFlagsA.bits.Btn_Scan  = 0;
    /* Change speed */
    tUGFlagsA.bits.ChangeSpeed =0;
    /* Change mode */
    tUGFlagsA.bits.ChangeMode =0;

    /* Zero out i sums */
    tPIParmInIQA.piState.integrator = 0;
    tPIParmInIDA.piState.integrator = 0;
    tPIParmInWA.piState.integrator = 0;

    /* Enable ADC interrupt and begin main loop timing */
    ClearADC1IF();
    EnableADC1Interrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI itteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI itteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary vars for sqrt calculation of q reference */
    volatile int16_t temp_qref_pow_q15;

    #ifdef TUNING
        /* Tuning speed ramp value  */
        tMotorParmA.tuning_add_rampup = 0;
        /* tuning speed ramp increase delay */
        tMotorParmA.tuning_delay_rampup;   
    #endif

    if( tUGFlagsA.bits.OpenLoop )
    {
        /* OPENLOOP:  force rotating angle,Vd,Vq */
        if( tUGFlagsA.bits.ChangeMode )
        {
            /* Just changed to openloop */
            tUGFlagsA.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            tCtrlParmA.qVqRef = 0;
            tCtrlParmA.qVdRef = 0;

            /* Reinit vars for initial speed ramp */
            tMotorParmA.startup_Lock = 0;
            tMotorParmA.startup_Ramp = 0;
            #ifdef TUNING
                MotorParm_A.tuning_add_rampup = 0;
            #endif
        }
        /* Speed reference */
        tCtrlParmA.qVelRef = Q_CURRENT_REF_OPENLOOP;
        /* q current reference is equal to the vel reference 
         while d current reference is equal to 0
        for maximum startup torque, set the q current to maximum acceptable 
        value represents the maximum peak value */
        tCtrlParmA.qVqRef    = tCtrlParmA.qVelRef;

        /* PI control for Q */
        tPIParmInIQA.inMeasure  = tParkCurrentA.q;
        tPIParmInIQA.inReference  = tCtrlParmA.qVqRef;
        MC_ControllerPIUpdate_Assembly(tPIParmInIQA.inReference,
                                       tPIParmInIQA.inMeasure,
                                       &tPIParmInIQA.piState,
                                       &tPIParmOutIQA.out);
        tParkVoltageA.q    = tPIParmOutIQA.out;

        /* PI control for D */
        tPIParmInIDA.inMeasure = tParkCurrentA.d;
        tPIParmInIDA.inReference  = tCtrlParmA.qVdRef;
        MC_ControllerPIUpdate_Assembly(tPIParmInIDA.inReference,
                                       tPIParmInIDA.inMeasure,
                                       &tPIParmInIDA.piState,
                                       &tPIParmOutIDA.out);
        tParkVoltageA.d    = tPIParmOutIDA.out;

    }
    else
    /* Closed Loop Vector Control */
    {
        /* if change speed indication, double the speed */
        if(tUGFlagsA.bits.ChangeSpeed)
        {
            /* if Bidirectional functioning, disable speed doubling */
            #ifndef BIDIRECTIONAL_SPEED
                /* read not signed ADC */
                ReadADC0( ADCBUF_SPEED_REF_A,&tReadADCParmA );

                tReadADCParmA.qAnRef=(tReadADCParmA.qADValue);
                if(tReadADCParmA.qAnRef >MAXIMUMSPEED_ELECTR)
                {
                    tReadADCParmA.qAnRef = MAXIMUMSPEED_ELECTR;
                }
            #else
                /* unsigned values */
                ReadSignedADC0( ADCBUF_SPEED_REF_A,&tReadADCParmA );

                /* ADC values are shifted with 2 */
                /* Speed pot ref max value +-8190 */
                tReadADCParmA.qAnRef=tReadADCParmA.qADValue>>2;
            #endif
        }
        else
        {
            /* if Bidirectional, read pot value signed */
            #ifdef BIDIRECTIONAL_SPEED
                /* signed values, different than in case of FW above */
                ReadSignedADC0( ADCBUF_SPEED_REF_A,&tReadADCParmA );
            #else
                /* unsigned values */
                ReadADC0( ADCBUF_SPEED_REF_A,&tReadADCParmA );
            #endif
            /* ADC values are shifted with 2 */
            /* Speed pot ref max value +-8190 */
            tReadADCParmA.qAnRef=tReadADCParmA.qADValue>>2;
        }

        /* Ramp generator to limit the change of the speed reference
          the rate of change is defined by CtrlParm.qRefRamp */
        tCtrlParmA.qDiff=tCtrlParmA.qVelRef - tReadADCParmA.qAnRef;
        /* Speed Ref Ramp */
        if (tCtrlParmA.qDiff < 0)
        {
            /* Set this cycle reference as the sum of
            previously calculated one plus the reframp value */
            tCtrlParmA.qVelRef=tCtrlParmA.qVelRef+tCtrlParmA.qRefRamp;
        }
        else
        {
            /* Same as above for speed decrease */
            tCtrlParmA.qVelRef=tCtrlParmA.qVelRef-tCtrlParmA.qRefRamp;
        }
        /* If difference less than half of ref ramp, set reference
        directly from the pot */
        if (_Q15abs(tCtrlParmA.qDiff) < (tCtrlParmA.qRefRamp<<1))
        {
            tCtrlParmA.qVelRef = tReadADCParmA.qAnRef;
        }

        /* Tuning is generating a software ramp
        with sufficiently slow ramp defined by 
        TUNING_DELAY_RAMPUP constant */
        #ifdef TUNING
            /* if delay is not completed */
            if(tMotorParmA.tuning_delay_rampup > TUNING_DELAY_RAMPUP)
            {
                tMotorParmA.tuning_delay_rampup = 0;
            }
            /* While speed less than maximum and delay is complete */
            if((tMotorParmA.tuning_add_rampup < (MAXIMUMSPEED_ELECTR - ENDSPEED_ELECTR)) &&
                                                  (tMotorParmA.tuning_delay_rampup == 0) )
            {
                /* Increment ramp add */
                tMotorParmA.tuning_add_rampup++;
            }
            tMotorParmA.tuning_delay_rampup++;
            /* The reference is continued from the open loop speed up ramp */
            tCtrlParmA.qVelRef =   ENDSPEED_ELECTR +  tMotorParmA.tuning_add_rampup;
        #endif

        if( tUGFlagsA.bits.ChangeMode )
        {
            /* Just changed from openloop */
            tUGFlagsA.bits.ChangeMode = 0;
            tPIParmInWA.piState.integrator = (int32_t)tCtrlParmA.qVqRef << 13;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
            tPIParmInWA.inMeasure  = tEstimParmA.qVelEstim;
            tPIParmInWA.inReference  = tCtrlParmA.qVelRef;
            MC_ControllerPIUpdate_Assembly(tPIParmInWA.inReference,
                                           tPIParmInWA.inMeasure,
                                           &tPIParmInWA.piState,
                                           &tPIParmOutWA.out);
            tCtrlParmA.qVqRef    = tPIParmOutWA.out;
        #else
            tCtrlParmA.qVqRef = tCtrlParmA.qVelRef;
        #endif
        
        /* Flux weakenign control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d curent component 
        adapt the estimator parameters in concordance with the speed */
        tCtrlParmA.qVdRef=FieldWeakening(_Q15abs(tCtrlParmA.qVelRef));

        /* PI control for D */
        tPIParmInIDA.inMeasure = tParkCurrentA.d;
        tPIParmInIDA.inReference  = tCtrlParmA.qVdRef;
        MC_ControllerPIUpdate_Assembly(tPIParmInIDA.inReference,
                                       tPIParmInIDA.inMeasure,
                                       &tPIParmInIDA.piState,
                                       &tPIParmOutIDA.out);
        tParkVoltageA.d    = tPIParmOutIDA.out;

        /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(tPIParmOutIDA.out ,
                                                      tPIParmOutIDA.out)>>15);
        temp_qref_pow_q15 = Q15(0.98) - temp_qref_pow_q15;
        tPIParmInIQA.piState.outMax = Q15SQRT (temp_qref_pow_q15);

        /* PI control for Q */
        tPIParmInIQA.inMeasure  = tParkCurrentA.q;
        tPIParmInIQA.inReference  = tCtrlParmA.qVqRef;
        MC_ControllerPIUpdate_Assembly(tPIParmInIQA.inReference,
                                       tPIParmInIQA.inMeasure,
                                       &tPIParmInIQA.piState,
                                       &tPIParmOutIQA.out);
        tParkVoltageA.q    = tPIParmOutIQA.out;
    }
       
}
// *****************************************************************************
/* Function:
    ADC1Interrupt()

  Summary:
    ADC1Interrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void)
{
    /* Clear Interrupt Flag */
    ClearADC1IF();

    iADCisrCnt++;

    if( tUGFlagsA.bits.RunMotor )
    {
	/* Calculate qIa,qIb */
        MeasCompCurr(ADCBUF_INV_A_IPHASE1, ADCBUF_INV_A_IPHASE2,&tMeasCurrParmA);
        tPhaseCurrentsA.a = tMeasCurrParmA.qIa;
        tPhaseCurrentsA.b = tMeasCurrParmA.qIb;
        /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
        MC_TransformClarke_Assembly(&tPhaseCurrentsA,&tClarkCurrentA);
        MC_TransformPark_Assembly(&tClarkCurrentA,&tSinCosThetaA,
                                   &tParkCurrentA);

        /* Speed and field angle estimation */
        Estim();
        /* Calculate control values */
        DoControl();
        /* Calculate qAngle from QEI Module */
	CalculateParkAngle();
        /* if open loop */
        if(tUGFlagsA.bits.OpenLoop == 1)
        {
            /* the angle is given by parkparm */
            qRotorAngleMA = qAngleOpenloopA;

        }
        else
        {
            /* if closed loop, angle generated by estim */
            qRotorAngleMA = tEstimParmA.qRho;
        }
        MC_CalculateSineCosine_Assembly_Ram(qRotorAngleMA,&tSinCosThetaA);
        MC_TransformParkInverse_Assembly(&tParkVoltageA,&tSinCosThetaA,
                                                        &tClarkVoltageA);

        MC_TransformClarkeInverseSwappedInput_Assembly(&tClarkVoltageA,
                                                       &tPhaseVoltagesA);
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&tPhaseVoltagesA,
                                                     iPWMPeriodA,
                                                     &tPWMDutyValueA);

        if(tPWMDutyValueA.dutycycle1 < MIN_DUTY)
        {
            tPWMDutyValueA.dutycycle1 = MIN_DUTY;
        }
        if(tPWMDutyValueA.dutycycle2 < MIN_DUTY)
        {
            tPWMDutyValueA.dutycycle2 = MIN_DUTY;
        }
        if(tPWMDutyValueA.dutycycle3 < MIN_DUTY)
        {
            tPWMDutyValueA.dutycycle3 = MIN_DUTY;
        }
        INVERTERA_PWM_PDC1 = tPWMDutyValueA.dutycycle1;
        INVERTERA_PWM_PDC2 = tPWMDutyValueA.dutycycle2;
	INVERTERA_PWM_PDC3 = tPWMDutyValueA.dutycycle3;

        #ifdef RTDM
        if (DMCIFlags.Recorder)
        {
            SnapShotDelayCnt++;
            if (SnapShotDelayCnt == SnapShotDelay)
            {
                SnapShotDelayCnt = 0;
                *PtrRecBuffer1++ = SNAP1;
                *PtrRecBuffer2++ = SNAP2;
                *PtrRecBuffer3++ = SNAP3;
                *PtrRecBuffer4++ = SNAP4;

                if (PtrRecBuffer1 > RecBuffUpperLimit)
                {
                    PtrRecBuffer1 = RecorderBuffer1;
                    PtrRecBuffer2 = RecorderBuffer2;
                    PtrRecBuffer3 = RecorderBuffer3;
                    PtrRecBuffer4 = RecorderBuffer4;
                    DMCIFlags.Recorder = 0;
                }
            }
        }
        #endif
    }

}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calulates the angle for open loop control

  Description:
    Generate the start sinwaves feeding the motor's terminals
    Open loop control, forcing the motor to allign and to start speeding up .
 *
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if(tUGFlagsA.bits.OpenLoop)
    {
        /* begin wiht the lock sequence, for field alligniament */
        if (tMotorParmA.startup_Lock < LOCK_TIME)
        {
            tMotorParmA.startup_Lock+=1;
        }
        /* Then ramp up till the end speed */
        else if (tMotorParmA.startup_Ramp < END_SPEED)
        {
            tMotorParmA.startup_Ramp+=OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        /* Switch to closed loop */
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                tUGFlagsA.bits.ChangeMode = 1;
                tUGFlagsA.bits.OpenLoop = 0;
            #endif
        }
        /* The angle set depends on startup ramp */
        qAngleOpenloopA+= (int16_t)(tMotorParmA.startup_Ramp >> 10);

    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if(tEstimParmA.qRhoOffset>0)
        {
            tEstimParmA.qRhoOffset--;
        }
    }
    return;
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initialises control parameters

  Description:
    Init control parameters: PI coefficients, scalling consts etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{

    /* ADC - Measure Current & Pot */
    /* Scaling constants: Determined by calibration or hardware design.*/
    tReadADCParmA.qK      = KPOT;
    tMeasCurrParmA.qKa    = KCURRA;
    tMeasCurrParmA.qKb    = KCURRB;
    
    tCtrlParmA.qRefRamp = SPEEDREFRAMP;
    
    /* SVGen  */
    /* Set PWM period to Loop Time */
    iPWMPeriodA = LOOPTIME_TCY;
 
    /* PI D Term */
    tPIParmInIDA.piState.kp = D_CURRCNTR_PTERM;
    tPIParmInIDA.piState.ki = D_CURRCNTR_ITERM;
    tPIParmInIDA.piState.kc = D_CURRCNTR_CTERM;
    tPIParmInIDA.piState.outMax = D_CURRCNTR_OUTMAX;
    tPIParmInIDA.piState.outMin = -tPIParmInIDA.piState.outMax;
    tPIParmInIDA.piState.integrator = 0;
    tPIParmOutIDA.out = 0;

    /* PI Q Term */
    tPIParmInIQA.piState.kp = Q_CURRCNTR_PTERM;
    tPIParmInIQA.piState.ki = Q_CURRCNTR_ITERM;
    tPIParmInIQA.piState.kc = Q_CURRCNTR_CTERM;
    tPIParmInIQA.piState.outMax = Q_CURRCNTR_OUTMAX;
    tPIParmInIQA.piState.outMin = -tPIParmInIQA.piState.outMax;
    tPIParmInIQA.piState.integrator = 0;
    tPIParmOutIQA.out = 0;

    /* PI W Term */
    tPIParmInWA.piState.kp = SPEEDCNTR_PTERM;
    tPIParmInWA.piState.ki = SPEEDCNTR_ITERM;
    tPIParmInWA.piState.kc = SPEEDCNTR_CTERM;
    tPIParmInWA.piState.outMax = SPEEDCNTR_OUTMAX;
    tPIParmInWA.piState.outMin = -tPIParmInWA.piState.outMax;
    tPIParmInWA.piState.integrator = 0;
    tPIParmOutWA.out = 0;
    return;
}
// *****************************************************************************
/* Function:
    Init_Peripherals()

  Summary:
    Routine initalises controller peripherals

  Description:
    Routine to initailise Peripherals used for Inverter Control

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPeripherals(void)
{
    /* Initialise comparator and amplifiers */
    InitAmplifiersComparator();
    /* Initialise PWM generators */
    InitPWMGenerators();
    /* Initialise the ADC used for sensing Inverter A paramneters */
    InitADCModule1(&iADC1OffsetA);
    /* Initialising Current offsets in structure variable */
    InitMeasCompCurr(iADC1OffsetA.ch2,
            iADC1OffsetA.ch1, &tMeasCurrParmA);
    DelayinMicrosecond(1);
}


/* EOF */
/*******************************************************************************
  Peripheral Intialisations Routine Header File

  File Name:
    periph.h

  Summary:
    This header file lists peripheral initialisation routine for dsPIC DSC
    modules ADC,PWM,OP-AMP,Comparator and PPS and GPIO pins

  Description:
    Defintions in the file are for dsPIC33EVGM106 PIM plugged onto
    Low-Voltage Motor Control Development board bundle from Microchip
Bundle
*******************************************************************************/
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
#ifndef _PERIPH_H
#define _PERIPH_H

#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "user_parms.h"
#include "sys_config.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// OSCILLATOR Related Defintions
#define OSC_FRC                 0               // Internal RC Oscllator
#define OSC_XTAL                1               // Primary oscillator - XTAL
// Select Oscillator Mode
#define OSC_MODE                OSC_XTAL
// Instruction cycle frequency (Hz)
#define FCY_MHZ                 (uint32_t)70
// Instruction cycle frequency (MHz)
#define FCY_HZ                  (uint32_t)(FCY_MHZ*1000000)
// Instruction cycle period (sec)
#define TCY_SEC                 (1.0/FCY_HZ)

// DEVICE RELATED SPECIFICATIONS
// Specify device operative voltage VDD in volts
#define DEVICE_VDD_VOLTS        5.0
// Specify device anlog supply voltage VDD in volts
#define DEVICE_AVDD_VOLTS       DEVICE_VDD_VOLTS

// Digital I/O defintions
// MC PWM MODULE Related Defintions
// Calculating deadtime in units of Tcy(Centre aligned Mode)
#define PWM_DT_ERRATA

#define DDEADTIME               (uint16_t)(2*DEADTIME_MICROSEC*FCY_MHZ)
// Basic loop period in units of Tcy
#define LOOPTIME_TCY            (uint16_t)(FCY_HZ/PWMFREQUENCY_HZ)

#ifdef PWM_DT_ERRATA
// Should be >= DDEADTIME/2 for PWM Errata workaround
    #define MIN_DUTY            (uint16_t)(DDEADTIME/2 + 1)
#else
    #define MIN_DUTY            0x0000
#endif

// OPAMP/CMP MODULE Related Defintions
// Calculate comaprator Voltage Reference 
// CVR<6:0> = (CVREF / CVRSRC ) * 128 = (CVREF / AVDD) * 128
#define CVREF_1      (uint8_t)((INVA_OC_LEVEL_REF_VOLTS/DEVICE_AVDD_VOLTS)*128)

// ADC MODULE Related Defintions
// Specify Minimum ADC Clock Period (TAD)in uSec
#define ADC_MIN_TAD_MICROSEC     0.075
// Specify Max Time to stabilize Analog Stage from ADC OFF to ADC ON in uSecs
// The parameter, tDPU, is the time required for the ADC module to stabilize at
// the appropriate level when the module is turned on (AD1CON1<ADON> = 1).
// During this time, the ADC result is indeterminate.
#define ADC_TON_DELAY_MICROSEC  300

// Definitions for Channels 1, 2, 3 Positive Input Select bits
/* 1xx = CH1 positive input is AN0 (Op Amp 2), 
         CH2 positive input is AN25 (Op Amp 5), 
         CH3 positive input is AN6 (Op Amp 3) */
#define CH123_IS_OA2_OA5_OA3    0x04
/* 011 = CH1 positive input is AN3 (Op Amp 1),
         CH2 positive input is AN0 (Op Amp 2), 
         CH3 positive input is AN25 (Op Amp 5) */
#define CH123_IS_OA1_OA2_OA5    0x03
/* 010 = CH1 positive input is AN3 (Op Amp 1),
   CH2 positive input is AN0 (Op Amp 2),
   CH3 positive input is AN6 (Op Amp 3) */
#define CH123_IS_OA1_OA2_OA3    0x02
/* 001 = CH1 positive input is AN3,
         CH2 positive input is AN4,
         CH3 positive input is AN5 */
#define CH123_IS_AN3_AN4_AN5    0x01
/* 000 = CH1 positive input is AN0,
         CH2 positive input is AN1,
         CH3 positive input is AN2 */
#define CH123_IS_AN0_AN1_AN2    0x00

// Setting Channel No connected to ADC1 Sample/Hold Channel #0(ADC1-CH0)
// POT1 is connected for sample/conversion by ADC1 CH0
#define ADC1_ANx_CH0            ANx_POT_1
// Setting Channels to be connected to ADC1 Sample/Hold Channels 1,2,3
// for simultaneous sampling  : OA2(IB),OA5(IA),OA3(IBUS)
#define ADC1_ANx_CH123          CH123_IS_OA2_OA5_OA3
// Caluclating  ADC conversion clock count ADCS from Min ADC Clock Period (TAD)
// TAD = Tp*(ADCS<7:0> + 1)= (1/Fp)*(ADCS<7:0> + 1)
// ADCS<7:0> = (MIN_TAD * Fp ) - 1 ~ (MIN_TAD * Fp )
// Subtraction by 1 is ignored as Min TAD cycle has to be met
#define ADC_MIN_ADCS_COUNTS     (uint8_t)((ADC_MIN_TAD_MICROSEC * FCY_MHZ))

#define EnableADC1Interrupt()   IEC0bits.AD1IE = 1
#define DisableADC1Interrupt()  IEC0bits.AD1IE = 0
#define ClearADC1IF()           IFS0bits.AD1IF = 0

/* Other Defintions */
#define	BTN_DEBOUNCETIME_mSEC	 0.01
#define	BTN_DEBOUNCE_COUNT      (BTN_DEBOUNCETIME_mSEC/LOOPTIME_SEC)

#define CountsinMicrosecond(n)  (uint32_t)(n*FCY_MHZ)
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ADC initial offset storage data type

  Description:
    This structure will host parameters related to ADC module offset for all
    four sample and hold channels.
*/
typedef struct
{
    // ch0 component
    int16_t ch0;

    // ch1 component
    int16_t ch1;

    // ch2 component
    int16_t ch2;

    // ch3 component
    int16_t ch3;
} ADC_OFFSET_T;
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitADCModule1(ADC_OFFSET_T *);
void InitAmplifiersComparator(void);
void InitPWMGenerators(void);
void SetupGPIOPorts(void);
void DelayinMicrosecond (uint16_t n);
void LongDelay(uint16_t i);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PERIPH_H


/* EOF */
/*******************************************************************************
  Hardware specific routine definition and interfaces Header File

  File Name:
    sys_config.h

  Summary:
    This header file lists hardware specific initialisations for dsPIC DSC
    modules ADC,PWM,OP-AMP,Comparator and PPS and GPIO pins

  Description:
    Definitions in the file are for dsPIC33EVGM106 PIM plugged onto
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
#ifndef _SYSCONFIG_H
#define _SYSCONFIG_H

#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// CURRENT SENSING/OVER CURRENT COMAPRATOR CIRCUIT Details
// Value of Shunt resistor (in Ohms)used for sensing Inverter A bus current
#define INVA_BUS_SHUNTRES_OHM   0.015
// Specify gain of the Inverter A bus current amplifier
#define INVA_IBUS_AMP_GAIN      23.5
// Specify DC bias(DC OFFSET)of the Inverter A bus current amplifier in volts
#define INVA_IBUS_AMP_VOFFSET   2.5
// Specify Peak value of Over current(in AMP)of Inverter A
#define INVA_OVER_CURRENT_AMPS  4.57
// Calculate Inverter A Over Current Trip level in Volts
// INVA_OC_TRIP_LEVEL_VOLTS = ((INVA_OVER_CURRENT_AMPS*INVA_IBUS_AMP_GAIN
//                        *INVA_BUS_SHUNTRES_OHM)+INVA_IBUS_AMP_VOFFSET)
#define INVA_OC_LEVEL_REF_VOLTS (float)((INVA_OVER_CURRENT_AMPS*INVA_IBUS_AMP_GAIN\
                                *INVA_BUS_SHUNTRES_OHM)+INVA_IBUS_AMP_VOFFSET)

// Digital I/O definitions
// Push button Switches
// SW1 : PIM #69 (RC4)
#define BTN_1                   PORTCbits.RC4
// SW2 : PIM #84 (RG6)
#define BTN_2                   PORTGbits.RG6
// SW3 : PIM #40 (RB8)
#define BTN_3                   PORTBbits.RB8
// SW4 : PIM #41 (RC3)
#define BTN_4                   PORTCbits.RC3

// SW1 : PIM #69 - Used as START/STOP button of Motor A
#define BTN_START_STOP_A        BTN_1
// SW2 : PIM #84 - Used as Speed HALF/DOUBLE button of Motor A
#define BTN_SPD_HALF_DBL_A      BTN_2

// Debug LEDs
// LED1 : PIM #01(RA10)
#define LED1                    LATAbits.LATA10
// LED2 : PIM #59 (RD8)
#define LED2                    LATDbits.LATD8
#define TOGGLE_LED1             __builtin_btg((unsigned int*)&LATA, 10)
#define TOGGLE_LED2             __builtin_btg((unsigned int*)&LATD, 8)

// MCP8024 Driver Chip Enable Outputs from dsPIC DSC
// CE_A : PIM #82 (RC13)
#define CE_A_MCP802x            LATCbits.LATC13
// CE_B : PIM #90 (RA7)
#define CE_B_MCP802x            LATAbits.LATA7
        
#define RSHUNT			0.015	// Value in Ohms of shunt resistors used.
#define DIFF_AMP_GAIN		23.5		// Gain of differential amplifier.
#define IPEAK                   14                // maximum current (peak to peak) that can be read by adc with the shunt/opamp settings above


// MC PWM MODULE Related Definitions
#define INVERTERA_PWM_PDC1      PDC1
#define INVERTERA_PWM_PDC2      PDC2
#define INVERTERA_PWM_PDC3      PDC3

// ADC MODULE Related Definitions
// Analog Channel No of Potentiometer #1 - used as Speed Reference
// POT1 : PIM #32 (AN13)
#define ANx_POT_1               13
// Analog Channel No of Inverter A DC bus voltage VDC_A
// VBUS_A : PIM #35 (AN10)
#define ANx_VBUS_A              10

#define ADCBUF_SPEED_REF_A      ADC1BUF0
#define ADCBUF_INV_A_IPHASE1    ADC1BUF2
#define ADCBUF_INV_A_IPHASE2    ADC1BUF1
#define ADCBUF_INV_A_IBUS       ADC1BUF3

// Specify bootstrap charging time in no of us
#define BOOTSTRAP_CHARGING_TIME 30000
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void MapGPIOHWFuntion(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of SYSCONFIG_H


/* EOF */
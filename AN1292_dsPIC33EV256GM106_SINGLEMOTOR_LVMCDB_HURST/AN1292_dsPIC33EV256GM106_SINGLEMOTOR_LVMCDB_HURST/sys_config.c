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
#include <xc.h>
#include <stdint.h>
#include "sys_config.h"

// *****************************************************************************
/* Function:
    Map_GPIO_HW_Funtion()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initialises GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFuntion(void)
{
 
    /* ANALOG SIGNALS */

    // Configure OP-AMP/Comparator Pins
    // OA1 is not used in this application
    TRISBbits.TRISB2 = 1;   // PGEC1/OA1IN+/AN4/C1IN3-/C1IN1+/C2IN3-/RPI34/RB2
    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB3 = 1;   // PGED1/OA1IN-/AN5/C1IN1-/(CTMUC)/RP35/RB3
    ANSELBbits.ANSB3 = 1;
    TRISBbits.TRISB1 = 1;   // PGEC3/OA1OUT/AN3/C1IN4-/C4IN2-/RPI33/CTED1/RB1
    ANSELBbits.ANSB1 = 1;

    // OA2 is used for amplifying Inverter A - PHASE2 Current (IPHASE2/IB)
    // SHUNT_HIGH_2_A(IPHASE2_POS_A/IB_POS_A) : PIM #73
    TRISAbits.TRISA1 = 1;    // OA2IN+/AN1/C2IN1+/RPI17/RA1
    ANSELAbits.ANSA1 = 1;
    // SHUNT_HIGH_SUM_A(IPHASE2_NEG_A/IB_NEG_A) : PIM #71
    TRISBbits.TRISB0 = 1;    // PGED3/OA2IN-/AN2/C2IN1-/SS1/RPI32/CTED2/RB0
    ANSELBbits.ANSB0 = 1;
    // IPHASE2_A/IB_A(OA2OUT) : PIM #72
    TRISAbits.TRISA0 = 1;    // OA2OUT/AN0/C2IN4-/C4IN3-/RPI16/RA0
    ANSELAbits.ANSA0 = 1;

    // OA5 is used for amplifying Inverter A - PHASE1 Current (IPHASE1/IA)
    // SHUNT_HIGH_1_A(IPHASE1_POS_A/IA_POS_A) : PIM #74
    TRISAbits.TRISA4 = 1;    // OA5IN+/AN24/C5IN3-/C5IN1+/SDO1/RP20/T1CK/RA4
    ANSELAbits.ANSA4 = 1;
    // SHUNT_HIGH_SUM_A(IPHASE1_NEG_A/IA_NEG_A) : PIM #81
    TRISBbits.TRISB9 = 1;    // OA5IN-/AN27//C5IN1-/ASDA1/RP41/RB9
    ANSELBbits.ANSB9 = 1;
    // IPHASE1_A/IB_A(OA5OUT) : PIM #77
    TRISBbits.TRISB7 = 1;    // OA5OUT/AN25/C5IN4-/RP39/INT0/RB7
    ANSELBbits.ANSB7 = 1;

    // OA3 is used for amplifying Inverter A - BUS Current (IBUS)
    // SHUNT_HIGH_SUM_A(IBUS_POS_A) : PIM #66
    TRISCbits.TRISC2 = 1; // OA3IN+/AN8/C3IN3-/C3IN1+/RPI50/U1RTS/BCLK1/FLT3/RC2
    ANSELCbits.ANSC2 = 1;
    // SHUNT_HIGH_SUM_A(IBUS_NEG_A) : PIM #67
    TRISCbits.TRISC1 = 1; // OA3IN-/AN7/C3IN1-/C4IN1-/RP49/RC1
    ANSELCbits.ANSC1 = 1;
    // IBUS_A(OA3OUT) : PIM #68
    TRISCbits.TRISC0 = 1; // OA3OUT/AN6/C3IN4-/C4IN4-/C4IN1+/RP48/RC0
    ANSELCbits.ANSC0 = 1;

    // POT inputs
    // Potentiometer #1 input - used as Speed Reference
    // POT1 : PIM #32
    TRISEbits.TRISE13 = 1;          // AN13/C3IN2-/U2CTS/FLT6/RE13
    ANSELEbits.ANSE13 = 1;

    // Inverter A DC bus voltage VDC_A
    // VBUS_A : PIM #35
    TRISAbits.TRISA12 = 1;          // AN10/RPI28/RA12
    ANSELAbits.ANSA12 = 1;

    // Back-EMF signals - Inverter A
    // VPHASE1_MA : PIM #22
    TRISEbits.TRISE14 = 1;          // AN14/RPI94/FLT7/RE14
    ANSELEbits.ANSE14 = 1;
    // VPHASE2_MA : PIM #21
    TRISEbits.TRISE15 = 1;          // AN15/RPI95/FLT8/RE15
    ANSELEbits.ANSE15 = 1;
    // VPHASE3_MA : PIM #20
    TRISGbits.TRISG9 = 1;           // AN16/RPI121/RG9
    ANSELGbits.ANSG9 = 1;

    // RECN_MA/IPHASE3_MA/IBRAKE_A : PIM #24
    TRISCbits.TRISC11 = 1;          // AN11/C1IN2-/U1CTS/FLT4/RC11
    ANSELCbits.ANSC11 = 1;

    // DIGITAL INPUT/OUTPUT PINS

    // InverterA - PWM 0utputs
    // PWM1L_A : PIM #93  (RPI47/PWM1L1/T5CK/RB15)
    // PWM1H_A : PIM #94  (RPI46/PWM1H1/T3CK/RB14)
    // PWM2L_A : PIM #98  (RP45/PWM1L2/CTPLS/RB13)
    // PWM2H_A : PIM #99  (RP44/PWM1H2/RB12)
    // PWM3L_A : PIM #100 (RP43/PWM1L3/RB11)
    // PWM3H_A : PIM #03  (RP42/PWM1H3/RB10)
    TRISB = (TRISB & 0x03FF);        // 0b0000 00XX XXXX XXXX
    LATB  = (LATB & 0x03FF);         // Setting all PWM outputs as 'LOW'
    // BRAKE_EN_A : PIM #60
    TRISDbits.TRISD5 = 0 ;           // RP69/RD5
    
    // FAULT Pins
    // Combined Fault input FAULT_AB = FAULT_A 'AND' FAULT_B
    // FAULT_AB : PIM #18
    TRISBbits.TRISB4 = 1;            // FLT32/RP36/RB4
    // InvertA - Over Current fault O/P FAULT_MA
    // FAULT_MA : PIM #70
    LATCbits.LATC7 = 1;              // Setting FAULT_MA =  '1'
    TRISCbits.TRISC7 = 0;            // AN52/RP55/RC7

    // Hall inputs - Inverter A
    // HALLA_MA : PIM #80
    TRISAbits.TRISA8 = 1;            // RPI24/RA8
    // HALLB_MA : PIM #47
    TRISCbits.TRISC6 = 1;            // AN53/RP54/RC6
    // HALLC_MA : PIM #48
    TRISFbits.TRISF0 = 1;            // RPI96/RF0
    // HOME_MA : PIM #61
    TRISCbits.TRISC10 = 1;           // AN48/CVREF2O/RPI58/RC10

    // Gate Driver Interface
    // CE_A : PIM #82 (Output)
    TRISCbits.TRISC13 = 0;           // RP161/RC13
    // DE2_TX_A : PIM #76 (Output)
    TRISGbits.TRISG8 = 0;            // AN17/RP120/RG8
    // DE2_RX_A : PIM #83 (Input)
    TRISGbits.TRISG7 = 1;            // AN18/RP119/RG7

    // Debug LEDs
    // LED1 : PIM #01
    TRISAbits.TRISA10 = 0;           // AN56/RA10
    // LED2 : PIM #59
    TRISDbits.TRISD8 = 0;            // RPI72/RD8

    // Push button Switches
    // SW1 : PIM #69
    TRISCbits.TRISC4 = 1;            // AN30/CVREF+/RPI52/RC4
    // SW2 : PIM #84
    TRISGbits.TRISG6 = 1;            // AN19/RP118/RG6
    // SW3 : PIM #40
    TRISBbits.TRISB8 = 1;            // AN26/CVREF1O/ASCL1/RP40/T4CK/RB8
    // SW4 : PIM #41
    TRISCbits.TRISC3 = 1;            // AN29/SCK1/RPI51/RC3

    // UART - for RTDM/DMCI Communication
    // UART_RX : PIM #49 (Input)
    TRISCbits.TRISC5 = 1;            // AN31/RPI53/RC5
    // UART_TX : PIM #50(Output)
    TRISFbits.TRISF1 = 0;            // RP97/RF1

    // CE_B : PIM #90 (Output)
    TRISAbits.TRISA7 = 0;            // AN55/RA7
    LATAbits.LATA7 = 0;              // Setting CE_B = 'LOW'

    /************************** Remappable PIn configuration ******************/

    //Unlock registers by clearing IOLOCK bit OSCCON(OSCONbits.IOLOCK = 0)
    __builtin_write_OSCCONL(OSCCON & (~(1 << 6))); 

    // Fault Input/Outputs
    // FAULT_MA : PIM #70 
    // Remap C4OUT(Comparator #4 Output) = FAULT_MA to RP55
    _RP55R = 0X32;                  // AN52/RP55/RC7
    
    // RTDM Communication RX and TX configuration ( UART #2)
    // UART_RX : PIM #49 (Input)
    // Configure RP53 as U2RX
    RPINR19bits.U2RXR = 53;         // AN31/RPI53/RC5
    // UART_TX : PIM #50 (Output)
    // Remap RP53 as U2RX
    _RP97R = 0x03;                  // RP97/RF1

    // DE2 Communication Interface between MCP8024(U8) and dsPIC (UART #1)
    // DE2_RX_A : PIM #83 (Input)
    // Configure RP119 as U1RX
    RPINR18bits.U1RXR = 119;        // AN18/RP119/RG7
    // DE2_TX_A : PIM #76 (Output)
    // Configure RP120 as U1TX
    _RP120R = 0x01;                 // AN17/RP120/RG8

    // Lock registers by setting IOLOCK bit OSCCON(OSCONbits.IOLOCK = 1)
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Set bit 6

    /**************************************************************************/

    return;
}

/*EOF*/

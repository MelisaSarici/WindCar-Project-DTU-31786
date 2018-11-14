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
#ifndef _MCP802x_DE2_H
#define _MCP802x_DE2_H

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
#include <uart.h>
#include "periph.h"
#include "sys_config.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/* This define has to be the system operating freq, this
   value is used to calculate the value of the BRG register */
#define DE2_FCY	 	FCY_HZ  
/* This is the desired baud-rate for the UART module to be
   used to implement DE2 communication between dsPIC and MCP802x driver */
#define DE2_BAUDRATE	9600UL    
/* Specify Voltage offset at the output of bus current amplifier in mV */
#define VREF_EXT        (DEVICE_VDD_VOLTS/2)*1000  
/* Define the UART module to be used by MCP8024 on inverter A side */
#define DE2_UART_MCP8024_A      DE2_UART_1
/* Defines the UART module to be used by MCP8024 on inverter B side */
//#define DE2_UART_MCP8024_B      DE2_UART_3

/* Based on selection made for DE2_UART_MCP8024_A uncomment the UART used for 
   communication  */
/* UART 1 module to be used for DE2 communication with MCp802x*/
#define DE2_UART_1	1
/* UART 2 module to be used for DE2 communication with MCp802x*/
//#define DE2_UART_2	2
/* UART 3 module to be used for DE2 communication with MCp802x*/
//#define DE2_UART_3	3
/* UART 4 module to be used for DE2 communication with MCp802x*/
//#define DE2_UART_4	4
/* UART 1 RX interrupt priority assigned to receive the messages from MCP802x */
#define DE2_UART_1_PRIORITY	6
/* UART 2 RX interrupt priority assigned to receive the messages from MCP802x */
//#define DE2_UART_2_PRIORITY	5
/* UART 3 RX interrupt priority assigned to receive the messages from MCP802x */
//#define DE2_UART_3_PRIORITY	5
/* UART 4 RX interrupt priority assigned to receive the messages from MCP802x */
//#define DE2_UART_4_PRIORITY	5
/* Counts in microseconds used for software timer to prevent improper UART
   communications*/
#define TIMER_OUT_COUNTS 1000
/* Defines buffer size for storing configuration and status messages */
#define DE2_BUFFER_SIZE 16

/* Mask to set the high temp warning bit in T_MCP802x_WARNING_FLAGS */
#define HIGH_TEMP_125_WARN_MASK 0x01 
/* Mask to set buck regulator over current warning bit in 
   T_MCP802x_WARNING_FLAGS */
#define BUCK_REG_OVERCURRENT_WARN_MASK 0x20 
/* Mask to set buck regulator under voltage warning bit in 
   T_MCP802x_WARNING_FLAGS */
#define BUCK_REG_UNDERVOLT_WARN_MASK 0x80 
/* Mask to set 5V LDO over current warning bit in 
   T_MCP802x_WARNING_FLAGS */
#define LDO_5V_OVERCURRENT_WARN_MASK 0x01  
/* Mask to set 12 V LDO over current warning bit in 
   T_MCP802x_WARNING_FLAGS */
#define LDO_12V_OVERCURRENT_WARN_MASK 0x02 

/* Mask to set over temperature fault bit in T_MCP802x_FAULT_FLAGS */
#define OVER_TEMP_160_FAULT_MASK 0x02 
/* Mask to set input under voltage fault bit in T_MCP802x_FAULT_FLAGS */
#define INPUT_UNDERVOLT_FAULT_MASK 0x04 
/* Mask to set input over voltage fault bit in T_MCP802x_FAULT_FLAGS */
#define INPUT_OVERVOLT_FAULT_MASK 0x10 
/* Mask to set buck regulator under voltage fault bit in T_MCP802x_FAULT_FLAGS */
#define BUCK_REG_UNDERVOLT_FAULT_MASK 0x80 
/* Mask to set  under voltage MOSFET fault bit in T_MCP802x_FAULT_FLAGS */
#define EXT_MOSFET_ULVO_FAULT_MASK 0x04 
/* Mask to set MOSFET over current fault bit in T_MCP802x_FAULT_FLAGS */
#define EXT_MOSFET_OVER_CURRENT_FAULT_MASK 0x08 
/* Mask to set BrownOut Reset fault bit in T_MCP802x_FAULT_FLAGS */
#define BROWNOUT_RESET_MASK 0x10 

/* Mask to set the register bits in UART status register to
   enable UART RX interrupt to occur on reception of 4 characters (Full Buffer)*/
#define UART_INT_RX_BUF_FUL_BITS 3
/* Mask to set the register bits in UART status register to
   enable UART RX interrupt to occur on reception of 3 characters (3/4 Buffer)*/
#define UART_INT_RX_3_4_FUL_BITS 2
/* Mask to set the register bits in UART status register to
   enable UART RX interrupt to occur on reception of 1 character */
#define UART_INT_RX_CHAR_BITS 0


/* *********DE2 Protocol Commands To MCP 802x and MESSAGES From MCP802x *******/
/* Set Configuration Register 0 for MCP802x */
#define CMD_SET_CFG_0 	0X81  
/* Get Configuration Register 0 from MCP802x */
#define CMD_GET_CFG_0 	0X82  

/* Set Configuration Register 1 for MCP802x */
#define CMD_SET_CFG_1 	0X83 
/* Get Configuration Register 1 from MCP802x */ 
#define CMD_GET_CFG_1 	0X84  

/* Set Configuration Register 2 for MCP802x */
#define CMD_SET_CFG_2 	0X87 
/* Get Configuration Register 2 from MCP802x */ 
#define CMD_GET_CFG_2 	0X88  

/* Get Status Register 0 from MCP802x */
#define CMD_STATUS_0 	0X85  
/* STATUS Register 1 Command from MCP802x (Unsolicited)*/
#define CMD_STATUS_1	0X86  

/* Set Configuration Register 0 Not Acknowledged (Response)*/
#define SET_CFG_0_NACK 	0X01  
/* Set Configuration Register 0 Acknowledged (Response)*/
#define SET_CFG_0_ACK	0X41  

/* Set Configuration Register 1 Not Acknowledged (Response) */
#define SET_CFG_1_NACK 	0X03 
/* Set Configuration Register 1 Acknowledged (Response)*/ 
#define SET_CFG_1_ACK	0X43  

/* Set Configuration Register 2 Not Acknowledged (Response)*/
#define SET_CFG_2_NACK 	0X07 
/* Set Configuration Register 2 Acknowledged (Response)*/ 
#define SET_CFG_2_ACK	0X47  

/* Get Configuration Register 0 Response Not Acknowledged (Response) */
#define GET_CFG_0_NACK 	0X02  
/* Get Configuration Register 0 Response Acknowledged (Response) */
#define GET_CFG_0_ACK	0X42  

/* Get Configuration Register 1 Response Not Acknowledged (Response) */
#define GET_CFG_1_NACK 	0X04  
/* Get Configuration Register 1 Response Acknowledged (Response) */
#define GET_CFG_1_ACK	0X44  

/* Get Configuration Register 2 Response Not Acknowledged (Response) */
#define GET_CFG_2_NACK 	0X08  
/* Get Configuration Register 2 Response Acknowledged (Response) */
#define GET_CFG_2_ACK	0X48  

/* Status Register 0 Response Not Acknowledged (Response)*/
#define STATUS_0_NACK 	0X05  
/* Status Register 0 Response Acknowledged (Response)*/
#define STATUS_0_ACK	0X45  

/* Status Register 1 Response Not Acknowledged (Response)*/
#define STATUS_1_NACK 	0X06  
/* Status Register 1 Response Acknowledged (Response)*/
#define STATUS_1_ACK	0X46  


/* Defines for DE2 SET_CFG_0 Command Options */

/* 0.250V External MOSFET Over current Limit */
#define CFG_0_FET_OC_LIMIT_250mV    0xFC    
/* 0.500V External MOSFET Over current Limit */
#define CFG_0_FET_OC_LIMIT_500mV    0xFD    
/* 0.750V External MOSFET Over current Limit */
#define CFG_0_FET_OC_LIMIT_750mV    0xFE    
/* 1.000V External MOSFET Over current Limit */
#define CFG_0_FET_OC_LIMIT_1V       0xFF    

/* Enable External MOSFET Short Circuit Detection */
#define CFG_0_FET_SC_DETECT_EN      0xFB    
/* Disable External MOSFET Short Circuit Detection */
#define CFG_0_FET_SC_DETECT_DIS     0xFF    

/* Enable Under voltage Lockout */
#define CFG_0_UV_LOCKOUT_EN         0xF7    
/* Disable Under voltage Lockout */
#define CFG_0_UV_LOCKOUT_DIS        0xFF    

/* Disable Disconnect of 30K LIN Bus/Level Translator Pull-up When CE=0 */
#define CFG_0_30K_PULLUP_DIS        0xBF    
/* Enable Disconnect of 30K LIN Bus/Level Translator Pull up When CE=0 */
#define CFG_0_30K_PULLUP_EN         0xFF    
/* Setting Reserved bits to 0 */
#define CFG_0_RESERVED              0x4F    

/* Defines for DE2 SET_CFG_2 Command Options */
/* Setting Reserved bits to 0 */
#define CFG_2_RESERVED              0x0F    

/* Driver Blanking Time = 4us */
#define CFG_2_BLANKING_TIME_4us     0xFC    
/* Driver Blanking Time = 2us */
#define CFG_2_BLANKING_TIME_2us     0xFD   
/* Driver Blanking Time = 1us  */ 
#define CFG_2_BLANKING_TIME_1us     0xFE   
/* Driver Blanking Time = 500 ns  */ 
#define CFG_2_BLANKING_TIME_500ns   0xFF    

/* Driver Dead Time ( For PWMH /PWML inputs) = 2us */
#define CFG_2_DEAD_TIME_2us         0xF3    
/* Driver Dead Time ( For PWMH /PWML inputs) = 1us */
#define CFG_2_DEAD_TIME_1us         0xF7    
/* Driver Dead Time ( For PWMH /PWML inputs) = 500 ns */
#define CFG_2_DEAD_TIME_500ns       0xFB    
 /* Driver Dead Time ( For PWMH /PWML inputs) = 250 ns */
#define CFG_2_DEAD_TIME_250ns       0xFF   

/* Motor Current Limit Value (i.e Internal Comparator Reference set through
   Programmable 8-bit DAC)
   DAC Value  = (ILIMIT_REF * RSHUNT* DIFFGAIN  - 0.991)/0.01377
   DAC_Value = ((ILIMIT_REF(in Amps) * RSHUNT(in Ohms) * 
                DIFFGAIN *1000)mV - 991mmv )/13.77mV/bit
 */
#define Calc_DAC_ILimit_Ref_Amps(iLimit_Ref_Amps)	\
		(uint8_t)(((iLimit_Ref_Amps*INVA_BUS_SHUNTRES_OHM*INVA_IBUS_AMP_GAIN*1000.0) -(VREF_EXT-991.0))/13.77)

#if defined DE2_FCY
    #if defined DE2_BAUDRATE
        #define DE2_BRG (DE2_FCY/(16*DE2_BAUDRATE))-1
    #else
        #error Cannot calculate BRG value. Please define DE2_BAUDRATE in MCP802x_DE2.h file
    #endif
#else
    #error Cannot calculate DE2_BRG value. Please define DE2_FCY in MCP802x_DE2.h file
#endif
/*(DE2_FCY/(16*(DE2_BRG+1))) */
#define DE2_BAUDRATE_ACTUAL	1 
/*((DE2_BAUDRATE_ACTUAL > DE2_BAUDRATE) ? DE2_BAUDRATE_ACTUAL - DE2_BAUDRATE : 
   DE2_BAUDRATE - DE2_BAUDRATE_ACTUAL)*/
#define DE2_BAUD_ERROR		1
#define DE2_BAUD_ERROR_PERCENT	(((DE2_BAUD_ERROR*100)+(DE2_BAUDRATE/2))/DE2_BAUDRATE)

#if (DE2_BAUD_ERROR_PERCENT > 1)
	#error The value loaded to the BRG register produces a baud rate error higher than 1%
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

/* DE2_Flags Parameter data type

  Description:
    This structure will host parameters related to DE2 Communication between
    MCP802x and dsPIC.
 */
typedef struct
{
    /* Flag to indicate status of ISR for UART1 Rx */
    unsigned messageReceived : 1; 
    /* Flag to indicate configuration message has being sent to MCP802x */
    unsigned configMessage : 1;
    /* Flag to indicate some fault bits is set in status messages from MCP802x */
    unsigned statusErrMessage : 1; 
    /* Flag to indicate STATUS_0 ACK (Response) has being received from MCP802x */
    unsigned status0AddressRx : 1; 
    /* Flag to indicate STATUS_1 ACK (Response) has being received from MCP802x */
    unsigned status1AddressRx : 1; 
    /* Flag to indicate error in CONFIG message response from MCP802x */
    unsigned configSettingErr : 1; 
    /* Flag to indicate error in STATUS message response from MCP802x */
    unsigned statusSettingErr : 1; 
    /* Flag to indicate status message request has being sent to MCP802x */
    unsigned statusMessage : 1; 
    unsigned unused : 8;
} T_DE2_FLAGS;

/* MCP802x_WARNING_FLAGS Parameter data type

  Description:
    This structure is used to store warning status flags related to DE2
    Communication between MCP802x and dsPIC.The flag set by MCP802x in this
    structure will not shut down MCP802x.
    These are warning status sent by MCP802x.
 */
typedef struct
{
    /* Flag to indicate high temp (TJ>125) warning from MCP802x*/
    unsigned highTemp125 : 1; 
    /* Flag to indicate buck regulator over current warning from MCP802x */
    unsigned buckRegOvercurr : 1; 
    /* Flag to indicate buck regulator output under voltage warning from MCP802x */
    unsigned buckRegUndervolt : 1; 
    /* Flag to indicate 5V LDO over current warning from MCP802x*/
    unsigned ldo5VOvercurr : 1; 
    /* Flag to indicate 12V LDO over current warning from MCP802x*/
    unsigned ldo12VOvercurr : 1; 
    unsigned unused : 11;
} T_MCP802x_WARNING_FLAGS;
/* T_MCP802x_FAULT_FLAGS Parameter data type

  Description:
    This structure is used to store fault indication flags related to DE2
    Communication between MCP802x and dsPIC.the flag set by MCP802x in
    this structure will shut down MCP802x. These are fault status
    sent by MCP802x*
 */
typedef struct
{
    /* Flag to indicate over temp (TJ>160) fault indication from MCP802x */
    unsigned overTemp160 : 1; 
    /* Flag to indicate input under voltage (VDD < 5.5V) from MCP802x */
    unsigned inputUndervolt : 1; 
    /* Flag to indicate Output over voltage (VDD > 32V) from MCP802x */
    unsigned inputOvervolt : 1; 
    /* Flag to indicate buck regulator output error voltage
    (<80% Brown out error) from MCP802x */
    unsigned buckRegUndervolt : 1; 
    /* Flag to indicate External MOSFET Under voltage Lock Out (UVLO)
    from MCP802x */
    unsigned extMosfetUvlo : 1; 
    /* Flag to indicate External MOSFET Over current Detection from MCP802x */
    unsigned extMosfetOverCurr : 1;
    /* Flag to indicate Brown-out Reset and Config Lost (Start-up default = 1)
    from MCP802x */
    unsigned brownOutReset : 1; 
    unsigned unused : 9;
} T_MCP802x_FAULT_FLAGS;

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
#ifdef DE2_UART_1

extern volatile T_DE2_FLAGS de2FlagsUart1;
extern  T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart1;
extern  T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart1;

#endif

#ifdef DE2_UART_2

extern volatile T_DE2_FLAGS de2FlagsUart2;
extern T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart2;
extern T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart2;

#endif

#ifdef DE2_UART_3

extern volatile T_DE2_FLAGS de2FlagsUart3;
extern T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart3;
extern T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart3;

#endif

#ifdef DE2_UART_4

extern volatile T_DE2_FLAGS de2FlagsUart4;
extern T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart4;
extern T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart4;

#endif

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
#ifdef DE2_UART_1

uint16_t De2StartUart1();
uint16_t ConfigureMcp802xUart1();
uint16_t GetStatusMcp802xUart1();
void De2CheckStatusUart1(void);
void De2ErrorCheckUart1(void);
void WaitUart1(void);
void De2ValidStatusUart1(void);

#endif

#ifdef DE2_UART_2

uint16_t De2StartUart2();
uint16_t ConfigureMcp802xUart2();
uint16_t GetStatusMcp802xUart2();
void De2CheckStatusUart2(void);
void De2ErrorCheckUart2(void);
void WaitUart2(void);
void De2ValidStatusUart2(void);

#endif

#ifdef DE2_UART_3

uint16_t De2StartUart3();
uint16_t ConfigureMcp802xUart3();
uint16_t GetStatusMcp802xUart3();
void De2CheckStatusUart3(void);
void De2ErrorCheckUart3(void);
void WaitUart3(void);
void De2ValidStatusUart3(void);

#endif

#ifdef DE2_UART_4

uint16_t De2StartUart4();
uint16_t ConfigureMcp802xUart4();
uint16_t GetStatusMcp802xUart4();
void De2CheckStatusUart4(void);
void De2ErrorCheckUart4(void);
void WaitUart4(void);
void De2ValidStatusUart4(void);
#endif

extern void StatusCheckMcp802x(uint16_t de2_UART_Module);
extern void ConfigureMcp8024(uint16_t de2_UART_Module);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of MCP8024x_DE2_H


/* EOF */

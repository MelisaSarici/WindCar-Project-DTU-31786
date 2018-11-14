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
#ifndef _UART_H
#define _UART_H

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

#define _UART_V2
/* List of SFRs for UART */
/* This list contains the SFRs with default (POR) values to be used for
   configuring UART */
/* The user can modify this based on the requirement */
#define UxMODE_VALUE            0x0000
#define UxSTA_VALUE             0x0110
#define UxTXREG_VALUE           0x0000
#define UxRXREG_VALUE           0x0000
#define UxBRG_VALUE             0x0000

#define getcUART1               ReadUART1
#define putcUART1               WriteUART1

#ifdef _U2RXIF

#define getcUART2               ReadUART2
#define putcUART2               WriteUART2

#endif

#ifdef _U3RXIF

#define getcUART3               ReadUART3
#define putcUART3               WriteUART3

#endif

#ifdef _U4RXIF

#define getcUART4               ReadUART4
#define putcUART4               WriteUART4

#endif

#if defined(_UART_V2)

/* defines for UxMODE register */

#define UART_EN                 0xFFFF  /* Module enable */
#define UART_DIS                0x7FFF  /* Module disable */

#define UART_IDLE_CON           0xDFFF  /* Work in IDLE mode */
#define UART_IDLE_STOP          0xFFFF  /* Stop all functions in IDLE mode*/

#define UART_IrDA_ENABLE        0xFFFF  /* IrDA encoder and decoder enabled*/
#define UART_IrDA_DISABLE       0xEFFF  /* IrDA encoder and decoder disabled */

#define UART_MODE_SIMPLEX       0xFFFF  /* UxRTS pin in Simplex mode */
#define UART_MODE_FLOW          0xF7FF  /* UxRTS pin in Flow Control mode*/

#define UART_UEN_11             0xFFFF  /*UxTX,UxRX and BCLK pins are enabled and used; UxCTS pin controlled by port latches*/
#define UART_UEN_10             0xFEFF  /*UxTX,UxRX, UxCTS and UxRTS pins are enabled and used*/
#define UART_UEN_01             0xFDFF  /*UxTX,UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by port latches*/
#define UART_UEN_00             0xFCFF  /*UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins controlled by port latches*/

#define UART_EN_WAKE            0xFFFF  /*Enable Wake-up on START bit Detect during SLEEP Mode bit*/
#define UART_DIS_WAKE           0xFF7F  /*Disable Wake-up on START bit Detect during SLEEP Mode bit*/

#define UART_EN_LOOPBACK        0xFFFF  /*Loop back enabled*/
#define UART_DIS_LOOPBACK       0xFFBF  /*Loop back disabled*/

#define UART_EN_ABAUD           0xFFFF  /*Enable baud rate measurement on the next character*/
#define UART_DIS_ABAUD          0xFFDF  /*Baud rate measurement disabled or completed*/

#define UART_UXRX_IDLE_ZERO     0xFFFF  /* UxRX Idle state is zero */
#define UART_UXRX_IDLE_ONE      0xFFEF  /* UxRx Idle state is one */

#define UART_BRGH_FOUR          0xFFFF  /* BRG generates 4 clocks per bit period */
#define UART_BRGH_SIXTEEN       0xFFF7  /* BRG generates 16 clocks per bit period */

#define UART_NO_PAR_9BIT        0xFFFF  /*No parity 9 bit*/
#define UART_ODD_PAR_8BIT       0xFFFD  /*odd parity 8 bit*/
#define UART_EVEN_PAR_8BIT      0xFFFB  /*even parity 8 bit*/
#define UART_NO_PAR_8BIT        0xFFF9  /*no parity 8 bit*/

#define UART_2STOPBITS          0xFFFF  /*2 stop bits*/
#define UART_1STOPBIT           0xFFFE  /*1 stop bit*/

/* defines for UART Status register */

#define UART_INT_TX_BUF_EMPTY   0xDFFF  /* Interrupt on TXBUF becoming empty */
#define UART_INT_TX_LAST_CH     0x7FFF  /* Interrupt when last character shifted out*/
#define UART_INT_TX             0x5FFF  /* Interrupt on transfer of every character to TSR */

#define UART_IrDA_POL_INV_ONE   0xFFFF  /*IrDA encoded, UxTX Idle state is '1' */
#define UART_IrDA_POL_INV_ZERO  0xBFFF  /* IrDA encoded, UxTX Idel state is '0' */

#define UART_SYNC_BREAK_ENABLED   0xFFFF  /* Send sync break on next transmission */
#define UART_SYNC_BREAK_DISABLED  0xF7FF  /* Sync break transmission disabled or completed */

#define UART_TX_ENABLE          0xFFFF  /* Transmit enable */
#define UART_TX_DISABLE         0xFBFF  /* Transmit disable */

#define UART_TX_BUF_FUL         0xFFFF  /* Transmit buffer is full */
#define UART_TX_BUF_NOT_FUL     0xFDFF  /* Transmit buffer is not full */

#define UART_INT_RX_BUF_FUL     0xFFFF  /* Interrupt on RXBUF full */
#define UART_INT_RX_3_4_FUL     0xFFBF  /* Interrupt on RXBUF 3/4 full */
#define UART_INT_RX_CHAR        0xFF7F  /* Interrupt on every char received */

#define UART_ADR_DETECT_EN      0xFFFF  /* address detect enable */
#define UART_ADR_DETECT_DIS     0xFFDF  /* address detect disable */

#define UART_RX_OVERRUN_CLEAR   0xFFFD  /* Rx buffer Over run status bit clear */

/* defines for UART Interrupt configuartion */
#define UART_RX_INT_EN          0xFFFF  /*Receive interrupt enabled*/
#define UART_RX_INT_DIS         0xFFF7  /*Receive interrupt disabled*/

#define UART_RX_INT_PR0         0xFFF8  /*Priority RX interrupt 0*/
#define UART_RX_INT_PR1         0xFFF9  /*Priority RX interrupt 1*/
#define UART_RX_INT_PR2         0xFFFA  /*Priority RX interrupt 2*/
#define UART_RX_INT_PR3         0xFFFB  /*Priority RX interrupt 3*/
#define UART_RX_INT_PR4         0xFFFC  /*Priority RX interrupt 4*/
#define UART_RX_INT_PR5         0xFFFD  /*Priority RX interrupt 5*/
#define UART_RX_INT_PR6         0xFFFE  /*Priority RX interrupt 6*/
#define UART_RX_INT_PR7         0xFFFF  /*Priority RX interrupt 7*/

#define UART_TX_INT_EN          0xFFFF  /*transmit interrupt enabled*/
#define UART_TX_INT_DIS         0xFF7F  /*transmit interrupt disabled*/

#define UART_TX_INT_PR0         0xFF8F  /*Priority TX interrupt 0*/
#define UART_TX_INT_PR1         0xFF9F  /*Priority TX interrupt 1*/
#define UART_TX_INT_PR2         0xFFAF  /*Priority TX interrupt 2*/
#define UART_TX_INT_PR3         0xFFBF  /*Priority TX interrupt 3*/
#define UART_TX_INT_PR4         0xFFCF  /*Priority TX interrupt 4*/
#define UART_TX_INT_PR5         0xFFDF  /*Priority TX interrupt 5*/
#define UART_TX_INT_PR6         0xFFEF  /*Priority TX interrupt 6*/
#define UART_TX_INT_PR7         0xFFFF  /*Priority TX interrupt 7*/

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART1 */
#define EnableIntU1RX                    _U1RXIE = 1
#define EnableIntU1TX                    _U1TXIE = 1

#define DisableIntU1RX                   _U1RXIE = 0
#define DisableIntU1TX                   _U1TXIE = 0

#define SetPriorityIntU1RX(priority)     _U1RXIP = priority
#define SetPriorityIntU1TX(priority)     _U1TXIP = priority

void putsUART1(uint16_t *buffer) __attribute__ ((section (".libperi")));

void WriteUART1(uint16_t data) __attribute__ ((section (".libperi")));

void CloseUART1(void) __attribute__ ((section (".libperi")));

void ConfigIntUART1(uint16_t config) __attribute__ ((section (".libperi")));

char DataRdyUART1(void) __attribute__ ((section (".libperi")));

uint16_t getsUART1(uint16_t length,uint16_t *buffer,
                       uint16_t uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART1(uint16_t config1,uint16_t config2, uint16_t ubrg) __attribute__ ((section (".libperi")));

uint16_t ReadUART1(void) __attribute__ ((section (".libperi")));

char BusyUART1(void) __attribute__ ((section (".libperi")));

#ifdef _U2RXIF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART2 */
#define EnableIntU2RX                    _U2RXIE = 1
#define EnableIntU2TX                    _U2TXIE = 1

#define DisableIntU2RX                   _U2RXIE = 0
#define DisableIntU2TX                   _U2TXIE = 0

#define SetPriorityIntU2RX(priority)     _U2RXIP = priority
#define SetPriorityIntU2TX(priority)     _U2TXIP = priority

void putsUART2(uint16_t *buffer) __attribute__ ((section (".libperi")));

void WriteUART2(uint16_t data) __attribute__ ((section (".libperi")));

void CloseUART2(void) __attribute__ ((section (".libperi")));

void ConfigIntUART2(uint16_t config) __attribute__ ((section (".libperi")));

char DataRdyUART2(void) __attribute__ ((section (".libperi")));

uint16_t getsUART2(uint16_t length,uint16_t *buffer,
                       uint16_t uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART2(uint16_t config1,uint16_t config2, uint16_t ubrg) __attribute__ ((section (".libperi")));

uint16_t ReadUART2(void) __attribute__ ((section (".libperi")));

char BusyUART2(void) __attribute__ ((section (".libperi")));

#endif

#ifdef _U3RXIF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART2 */
#define EnableIntU3RX                    _U3RXIE = 1
#define EnableIntU3TX                    _U3TXIE = 1

#define DisableIntU3RX                   _U3RXIE = 0
#define DisableIntU3TX                   _U3TXIE = 0

#define SetPriorityIntU3RX(priority)     _U3RXIP = priority
#define SetPriorityIntU3TX(priority)     _U3TXIP = priority

void putsUART3(uint16_t *buffer) __attribute__ ((section (".libperi")));

void WriteUART3(uint16_t data) __attribute__ ((section (".libperi")));

void CloseUART3(void) __attribute__ ((section (".libperi")));

void ConfigIntUART3(uint16_t config) __attribute__ ((section (".libperi")));

char DataRdyUART3(void) __attribute__ ((section (".libperi")));

uint16_t getsUART3(uint16_t length,uint16_t *buffer,
                       uint16_t uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART3(uint16_t config1,uint16_t config2, uint16_t ubrg) __attribute__ ((section (".libperi")));

uint16_t ReadUART3(void) __attribute__ ((section (".libperi")));

char BusyUART3(void) __attribute__ ((section (".libperi")));

#endif

#ifdef _U4RXIF

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART2 */
#define EnableIntU4RX                    _U4RXIE = 1
#define EnableIntU4TX                    _U4TXIE = 1

#define DisableIntU4RX                   _U4RXIE = 0
#define DisableIntU4TX                   _U4TXIE = 0

#define SetPriorityIntU4RX(priority)     _U4RXIP = priority
#define SetPriorityIntU4TX(priority)     _U4TXIP = priority

void putsUART4(uint16_t *buffer) __attribute__ ((section (".libperi")));

void WriteUART4(uint16_t data) __attribute__ ((section (".libperi")));

void CloseUART4(void) __attribute__ ((section (".libperi")));

void ConfigIntUART4(uint16_t config) __attribute__ ((section (".libperi")));

char DataRdyUART4(void) __attribute__ ((section (".libperi")));

uint16_t getsUART4(uint16_t length,uint16_t *buffer,
                       uint16_t uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART4(uint16_t config1,uint16_t config2, uint16_t ubrg) __attribute__ ((section (".libperi")));

uint16_t ReadUART4(void) __attribute__ ((section (".libperi")));

char BusyUART4(void) __attribute__ ((section (".libperi")));

#endif

#endif

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of UART_H


/* EOF */

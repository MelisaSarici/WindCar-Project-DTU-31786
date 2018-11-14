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
#include "mcp802x_de2.h"
#include <string.h>
#include "general.h"
#include "user_parms.h"
#include <stdint.h>
#include "sys_config.h"
#include "periph.h"

/* Variable to store value of configuration register setting send to MCP8024 */
volatile uint16_t cmdOption = 0;  
/* Holds the value of UART config register */
volatile uint16_t de2UartModeValue = 0; 
/* Holds the information regarding UART	TX & RX interrupt modes */
volatile uint16_t de2UartStaValue = 0; 
/* Used as a software watchdog timer used to recover from improper UART 
   communication */ 
volatile uint16_t timerOut = 0; 


#ifdef DE2_UART_1
/* Stores MCP802x Configuration data */
uint8_t de2RxBufferUart1[DE2_BUFFER_SIZE]; 
/* Pointer to de2RxBufferUart1 */
uint8_t *pDe2RxBufferIndexUart1 = de2RxBufferUart1; 
/* Pointer to start of array for DE2RXbufffer */
uint8_t * pDe2RxBufferIndexUart1Low = de2RxBufferUart1; 
/* Pointer to end of array for DE2Rx buffer */
uint8_t * pDe2RxBufferIndexUart1High = (de2RxBufferUart1 + DE2_BUFFER_SIZE); 
/* MCP8024 status data */
uint8_t driverStatusUart1[DE2_BUFFER_SIZE]; 
/* Pointer to driverstatusuart1 buffer */
uint8_t * pDe2StatusIndexUart1 = driverStatusUart1;
/* Pointer to start of array driverstatusUart1 */ 
uint8_t * pDe2StatusIndexUart1Low = driverStatusUart1; 
/* Pointer to end of array for driver status */
uint8_t * pDe2StatusIndexUart1High = (driverStatusUart1 + DE2_BUFFER_SIZE); 
/* Variables to store status received from MCP802x */
uint8_t status0Uart1 = 0, status1Uart1 = 0; 
/* Flags indicating various process in MCP802x communication */
volatile T_DE2_FLAGS de2FlagsUart1; 
/* Flags indicating warning faults received from MCO802x. 
   This faults will not switch off motor drivers on MCP802x */
T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart1;
/* Flags indicating error faults received from MCO802x. 
   This faults will switch off motor drivers on MCP802x */
T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart1;

// *****************************************************************************
/* Function:
    De2StartUart1()

  Summary:
    Routine to initialise UART1 for DE2 Communication

  Description:
    Routine to initialise UART1 for DE2 Communication

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    return 0 if no errors

  Remarks:
    None.
 */
uint16_t De2StartUart1(void)
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART1 module */
    CloseUART1();
	
    /* Configure UART1 receive and transmit interrupt */
    ConfigIntUART1(UART_RX_INT_EN & (UART_RX_INT_PR0 + DE2_UART_1_PRIORITY) & 
	               UART_TX_INT_DIS);

    /* Configure UART1 module to transmit 8 bit data with one stop bit. */
    de2UartModeValue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
            UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE &
            UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
            UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;


    de2UartStaValue = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO &
            UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR &
            UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
			
    /*Turn on UART 1 module and configure Rx interrupt with every character*/
    OpenUART1(de2UartModeValue, de2UartStaValue, DE2_BRG);

    return 0;
}
// *****************************************************************************
/* Function:
    ConfigureMcp802xUart1()

  Summary:
    Routine to configure MCP802x series of driver through DE2 protocol

  Description:
    Routine to configure MCP802x series of driver through DE2 protocol

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t ConfigureMcp802xUart1(void)
{
    /* Dummy Rx Buffer read */
    ReadUART1();
    /* Wait before transmission and clear DE2flags */	
    WaitUart1(); 
	
    if (de2FlagsUart1.messageReceived)
    {
        return 1;
    }
    /* Configure UART 1 Rx interrupt to occur on reception of 4 character
   (Full Buffer)*/
    U1STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart1.configMessage = TRUE; 
    /* Configuration Register 0 of MCP802x */
    cmdOption = CFG_0_RESERVED & CFG_0_30K_PULLUP_EN & 
        CFG_0_UV_LOCKOUT_EN & CFG_0_FET_SC_DETECT_EN & CFG_0_FET_OC_LIMIT_1V;

    /* This message is sent to MCP802x to configure MCP802x */
    WriteUART1(CMD_SET_CFG_0); 
    WriteUART1(cmdOption);
	
   /* Wait till acknowledgement is received from MCP802x */
    while (!de2FlagsUart1.messageReceived) 
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
               communication */
            return 1;    
                          
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart1 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart1 - 2) != SET_CFG_0_ACK)
    {
        return 1;
    }
    
    /* Wait before transmission and clear DE2flags */
    WaitUart1(); 
    if (de2FlagsUart1.messageReceived)
    {
        return 1;
    }
    /* Configure UART 1 Rx interrupt to occur on reception of 4 character
   (Full Buffer)*/
    U1STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS;
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart1.configMessage = TRUE; 
    /* Configuration Register 1 of MCP802x */
    cmdOption = Calc_DAC_ILimit_Ref_Amps(INVA_OVER_CURRENT_AMPS);
    
    /* This message is sent to configure the motor current limit reference 
    DAC */
    WriteUART1(CMD_SET_CFG_1); 
    WriteUART1(cmdOption);
    while (!de2FlagsUart1.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
	    /* Return to indicate that there is problem with UART
	    communication */
            return 1;    
                          
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart1 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart1 - 2) != SET_CFG_1_ACK)
    {
        return 1;
    }
    /* Wait before transmission and clear DE2flags */
    WaitUart1(); 
    if (de2FlagsUart1.messageReceived)
    {
        return 1;
    }
    /* Configure UART 1 Rx interrupt to occur on reception of 4 character
       (Full Buffer)*/
    U1STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart1.configMessage = TRUE; 
	
    /* Configuration Register 2 of MCP802x */
    cmdOption = CFG_2_RESERVED & CFG_2_BLANKING_TIME_4us & CFG_2_DEAD_TIME_2us;
    /* This message is sent to retrieve the motor current limit reference 
    DAC Configuration register*/
    WriteUART1(CMD_SET_CFG_2);

    WriteUART1(cmdOption);
    while (!de2FlagsUart1.messageReceived)
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
	    /* Return to indicate that there is problem with UART
	    communication */
            return 1;    
                          
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    if (*(pDe2RxBufferIndexUart1 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart1 - 2) != SET_CFG_2_ACK)
    {
        return 1;
    }

    /* Reset the flag to indicate end of configuration setting for MCP8024 */
    de2FlagsUart1.configMessage = FALSE;
    /* Reset the flag to indicate end of UART communication for configuration
       messages*/
    de2FlagsUart1.messageReceived = FALSE;
    /* Configure UART 1 Rx interrupt to occur on reception of 1 character */
    U1STAbits.URXISEL = UART_INT_RX_CHAR_BITS;

    return 0;
}
// *****************************************************************************
/* Function:
    GetStatusMcp802xUart1()

  Summary:
    Routine to get Status from Driver MCP802x

  Description:
    Routine to get Status from Driver MCP802x

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t GetStatusMcp802xUart1(void)
{
    /* Dummy Rx Buffer read */
    ReadUART1(); 
    /* Wait before transmission and clear DE2flags */
    WaitUart1(); 
    if (de2FlagsUart1.messageReceived)
    {
        return 1;
    }
    /* Configure UART 1 Rx interrupt to occur on reception of 3 character
       3/4 Buffer full) */
    U1STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS; 
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart1.statusMessage = TRUE;
    /* Request for status_0 register of MCP802x */
    WriteUART1(CMD_STATUS_0); 
    while (!de2FlagsUart1.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
	    /* Return to indicate that there is problem with UART
            communication */
            return 1;    
                          
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Wait before transmission and clear DE2flags */
    WaitUart1(); 
    if (de2FlagsUart1.messageReceived)
    {
        return 1;
    }
    /* Configure UART 1 Rx interrupt to occur on reception of 3 character
   (3/4 Buffer full) */
    U1STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart1.statusMessage = TRUE;
    /* Request for status_1 register of MCP802x */
    WriteUART1(CMD_STATUS_1); 
    while (!de2FlagsUart1.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                   
        }

    }
    /* Reset the timer */
    timerOut = 0; 
    de2FlagsUart1.messageReceived = FALSE;
    /* Empty driver status buffer to prevent overflow of buffer */
    U1STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    memset(driverStatusUart1, 0, sizeof (driverStatusUart1));
    /* Set pointer to start of array */
    pDe2StatusIndexUart1 = pDe2StatusIndexUart1Low; 
    /* Configure UART 1 Rx interrupt to occur on reception of 1 character*/
    U1STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    return 0;
}
// *****************************************************************************
/* Function:
    U1RXInterrupt()

  Summary:
    UART1 Interrupt service routine

  Description:
    Routine processes and stores the received UART1 data in appropriate buffer

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    de2FlagsUart1.messageReceived = FALSE;
    _U1RXIF = FALSE;

    if (de2FlagsUart1.configMessage)
    {
        while (DataRdyUART1())
        {
            /* If the pointer to Rx buffer has reached top of buffer break
	    from this routine */
            if (pDe2RxBufferIndexUart1 == pDe2RxBufferIndexUart1High)
            {
                break;
            }
            /* Stores configuration data received from MCP802x in DE2Rx Buffer */
            *(pDe2RxBufferIndexUart1++) = ReadUART1();

        }
	/* Reset the flag to indicate completion of config message 
        communication setting to buffer */
        de2FlagsUart1.configMessage = FALSE;

    }
    else if (de2FlagsUart1.statusMessage)
    {
        while (DataRdyUART1())
        {
	    /* If the pointer to statusrRxbuffer has reached top of buffer
	    break from this routine*/
            if (pDe2StatusIndexUart1 == pDe2StatusIndexUart1High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART 
	    Buffer */
            *(pDe2StatusIndexUart1++) = ReadUART1();

        }
        /* Check the status message received from MCP802x and store in 
        appropriate status indication variable */
        De2CheckStatusUart1();
        /* Reset the flag to indicate completion of storing status message 
        received to buffer */
        de2FlagsUart1.statusMessage=FALSE;

    }
    /* This routine is to process unsolicited status message received
       from MCP8024 */
    else
    {

        while (DataRdyUART1())
        {
	    /* If the pointer to statusrRxbuffer has reached top of buffer break
            from this routine */
            if (pDe2StatusIndexUart1 == pDe2StatusIndexUart1High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART 
	       Buffer */
            *(pDe2StatusIndexUart1++) = ReadUART1();

        }
	/* Check for flag to indicate whether proper status 0  
           address/acknowledgement is received from MCP8024 */
        if (de2FlagsUart1.status0AddressRx)
        {
	    /* Store the status 0 register received form MCP8024 into appropriate
	       variable */
            status0Uart1 = *(pDe2StatusIndexUart1 - 1);
            /* Reset the flag to indicate status 0 register is stored
               in appropriate variable */
            de2FlagsUart1.status0AddressRx = FALSE;
            /* Set the flag if the fault or warning status message is received 
	       from MCP8024 */
            if (status0Uart1 != 0x00)
            {
                de2FlagsUart1.statusErrMessage = TRUE;

            }
        }
	/* Check for flag to indicate whether proper status 1 
           address/acknowledgement is received from MCP8024 */
        else if (de2FlagsUart1.status1AddressRx)
        {
	    /* Store the status 1 register received form MCP8024 into appropriate
	       variable */
            status1Uart1 = *(pDe2StatusIndexUart1 - 1);

            de2FlagsUart1.status1AddressRx = FALSE;
            if (status1Uart1 != 0x00)
            {
		/* Set the flag if the fault or warning status message is received 
	           from MCP8024 */
                de2FlagsUart1.statusErrMessage = TRUE;

            }
        }
        else
        {
	    /* Validate the unsolicited message received form MCP8024  */
            De2ValidStatusUart1();

        }
    }
    /* Set the flag to indicate the end of interrupt service routine for Uart1 */
    de2FlagsUart1.messageReceived = TRUE;
}

// *****************************************************************************
/* Function:
    De2ValidStatusUart1()

  Summary:
    Routine to validate and process unsolicited message received from MCP8024

  Description:
    Function validates and process unsolicited message received from MCP8024 
	through UART1

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2ValidStatusUart1(void)
{
    if (*(pDe2StatusIndexUart1 - 1) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart1 - 1) == CMD_STATUS_0)
    {
        /* Set the flag if the unsolicited message received indicates that
            next character will contain status 0 register of MCP8024 */
        de2FlagsUart1.status0AddressRx = TRUE;
    }
    else if ((*(pDe2StatusIndexUart1 - 1) == STATUS_1_ACK) || 
	    (*(pDe2StatusIndexUart1 - 1) == CMD_STATUS_1))
    {
	/* Set the flag if the unsolicited message received indicates that next 
           character will contain status 1 register of MCP8024 */
        de2FlagsUart1.status1AddressRx = TRUE;
    }
    /* If the unsolicited contains garbage empty the statusRxbuffer */
    else
    {
	 /* Empty driver status buffer to prevent overflow of buffer */
         memset(driverStatusUart1, 0, sizeof (driverStatusUart1));
	 /* Set the pointer to start of array */
         pDe2StatusIndexUart1 = pDe2StatusIndexUart1Low; 
    }
}
// *****************************************************************************
/* Function:
    De2CheckStatusUart1()

  Summary:
    Routine to process received message from UART1 and store it in appropriate 
	status indication variable

  Description:
    Routine to process received message from UART1 and store it in appropriate 
	status indication variable

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2CheckStatusUart1(void)
{
    /* Check if valid response acknowledgement or command to host for status 
       register 0 is received*/
    if (*(pDe2StatusIndexUart1 - 2) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart1 - 2) == CMD_STATUS_0)
    {
	/* Stores status register 0 in appropriate variable */
        status0Uart1 = *(pDe2StatusIndexUart1 - 1); 
        if (status0Uart1 != 0x00)
        {
            /* Set the flag if the fault or warning status message is
            received from MCP8024 */
            de2FlagsUart1.statusErrMessage = TRUE;
        }
    }
    else if ((*(pDe2StatusIndexUart1 - 2) == STATUS_1_ACK) || 
			 (*(pDe2StatusIndexUart1 - 2) == CMD_STATUS_1))
    {
        /* Check if valid response acknowledgement or command to host for
        status register 1 is received */
        status1Uart1 = *(pDe2StatusIndexUart1 - 1); 
	/* Stores status register 1 in appropriate variable */
        if (status1Uart1 != 0x00)
        {
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            de2FlagsUart1.statusErrMessage = TRUE;
        }
    }
    else
    {
	/* Empty status buffer if non-valid message received */
        memset(driverStatusUart1, 0, sizeof (driverStatusUart1));
	/* Pointer to start of array */
        pDe2StatusIndexUart1 = pDe2StatusIndexUart1Low; 
    }
}
// *****************************************************************************
/* Function:
    De2ErrorCheckUart1()

  Summary:
	Routine to process status messages from MCP802x and set error flag 
	accordingly

  Description:
	Routine to process status messages from MCP802x and set error flag 
	accordingly

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    Warning message will not cause MCP802x to switch off gate driver. 
    Fault message will cause mcp802x to switch off gate driver 
 */
void De2ErrorCheckUart1(void)
{
    if (status0Uart1 & HIGH_TEMP_125_WARN_MASK)
    {
	/* High temp (TJ>125) warning from MCP802x */
        mcp802xWarningFlagsUart1.highTemp125 = TRUE;
    }
    if (status0Uart1 & OVER_TEMP_160_FAULT_MASK)
    {
	/* Over temp (TJ>160) fault indication from MCP802x */
        mcp802xFaultFlagsUart1.overTemp160 = TRUE;
    }
    if (status0Uart1 & INPUT_UNDERVOLT_FAULT_MASK)
    {
	/* Input under voltage fault (VDD < 5.5V) */
        mcp802xFaultFlagsUart1.inputUndervolt = TRUE;
    }
    if (status0Uart1 & INPUT_OVERVOLT_FAULT_MASK)
    {
	/* Output over voltage fault (VDD > 32V) */
        mcp802xFaultFlagsUart1.inputOvervolt = TRUE;
    }
    if (status0Uart1 & BUCK_REG_OVERCURRENT_WARN_MASK)
    {
	/* Buck regulator over current warning */
        mcp802xWarningFlagsUart1.buckRegOvercurr = TRUE;
    }
    if (status0Uart1 & BUCK_REG_UNDERVOLT_WARN_MASK)
    {
	/* Buck regulator output under voltage warning */
        mcp802xWarningFlagsUart1.buckRegUndervolt = TRUE;
    }
    if (status0Uart1 & BUCK_REG_UNDERVOLT_FAULT_MASK)
    {
	/* Buck regulator output under voltage (<80% Brown out error)*/
        mcp802xFaultFlagsUart1.buckRegUndervolt = TRUE;
    }
    if (status1Uart1 & LDO_5V_OVERCURRENT_WARN_MASK)
    {
        /* 5V LDO over current warning */
        mcp802xWarningFlagsUart1.ldo5VOvercurr = TRUE;
    }
    if (status1Uart1 & LDO_12V_OVERCURRENT_WARN_MASK)
    {
	/* 12V LDO over current warning */
        mcp802xWarningFlagsUart1.ldo12VOvercurr = TRUE;
    }
    if (status1Uart1 & EXT_MOSFET_ULVO_FAULT_MASK)
    {
	/* External MOSFET Under voltage Lock Out (UVLO) fault */
        mcp802xFaultFlagsUart1.extMosfetUvlo = TRUE;
    }
    if (status1Uart1 & EXT_MOSFET_OVER_CURRENT_FAULT_MASK)
    {
        /* External MOSFET Over current Detection fault */
        mcp802xFaultFlagsUart1.extMosfetOverCurr = TRUE;
    }
    if (status1Uart1 & BROWNOUT_RESET_MASK)
    {
	/* Brown-out Reset and CONFIG Lost (Start-up default = 1)*/
        mcp802xFaultFlagsUart1.brownOutReset = TRUE;
    }
}
// *****************************************************************************
/* Function:
    WaitUart1()

  Summary:
	Routine to set UART1 Rx interrupt to occur on reception of 1 character and 
	wait in between configuration and status messages processing

  Description:
	Routine to set UART1 Rx interrupt to occur on reception of 1 character and 
	wait in between configuration and status messages processing

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    None.
 */
void WaitUart1(void)
{
    /* Reset the flag to enter wait state */
    de2FlagsUart1.configMessage = FALSE; 
    /* Reset the flag to indicate no request is made by dsPIC to MCP8024 */
    de2FlagsUart1.messageReceived = FALSE; 
    /* Reset the flag to enter wait state */
    de2FlagsUart1.statusMessage = FALSE; 
    /* Configure UART 1 Rx interrupt to occur on reception of 1 character */
    U1STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Wait */
    LongDelay(10); 
}
#endif


#ifdef DE2_UART_2
/* Stores MCP802x Configuration data */
uint8_t de2RxBufferUart2[DE2_BUFFER_SIZE]; 
/* Pointer to de2RxBufferUart2 */
uint8_t *pDe2RxBufferIndexUart2 = de2RxBufferUart2; 
/* Pointer to start of array for DE2RXbufffer */
uint8_t * pDe2RxBufferIndexUart2Low = de2RxBufferUart2; 
/* Pointer to end of array for DE2Rx buffer */
uint8_t * pDe2RxBufferIndexUart2High = (de2RxBufferUart2 + DE2_BUFFER_SIZE); 
/* MCP8024 status data */
uint8_t driverStatusUart2[DE2_BUFFER_SIZE]; 
/* Pointer to driverstatusuart2 buffer */
uint8_t * pDe2StatusIndexUart2 = driverStatusUart2;
/* Pointer to start of array driverstatusUart2 */ 
uint8_t * pDe2StatusIndexUart2Low = driverStatusUart2; 
/* Pointer to end of array for driver status */
uint8_t * pDe2StatusIndexUart2High = (driverStatusUart2 + DE2_BUFFER_SIZE); 
/* Variables to store status received from MCP802x */
uint8_t status0Uart2 = 0, status1Uart2 = 0; 
/* Flags indicating various process in MCP802x communication */
volatile T_DE2_FLAGS de2FlagsUart2; 
/* Flags indicating warning faults received from MCO802x. 
   This faults will not switch off motor drivers on MCP802x */
T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart2;
/* Flags indicating error faults received from MCO802x. 
   This faults will switch off motor drivers on MCP802x */
T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart2;

// *****************************************************************************
/* Function:
    De2StartUart2()

  Summary:
    Routine to initialise UART2 for DE2 Communication

  Description:
    Routine to initialise UART2 for DE2 Communication

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    return 0 if no errors

  Remarks:
    None.
 */
uint16_t De2StartUart2(void)
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART2 module */
    CloseUART2();
	
    /* Configure UART2 receive and transmit interrupt */
    ConfigIntUART2(UART_RX_INT_EN & (UART_RX_INT_PR0 + DE2_UART_2_PRIORITY) & 
	               UART_TX_INT_DIS);

    /* Configure UART2 module to transmit 8 bit data with one stop bit. */
    de2UartModeValue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                       UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE &
                       UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                       UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;


    de2UartStaValue = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO &
                      UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR &
                      UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
			
    /*Turn on UART 2 module and configure Rx interrupt with every character*/
    OpenUART2(de2UartModeValue, de2UartStaValue, DE2_BRG);

    return 0;
}
// *****************************************************************************
/* Function:
    ConfigureMcp802xUart2()

  Summary:
    Routine to configure MCP802x series of driver through DE2 protocol

  Description:
    Routine to configure MCP802x series of driver through DE2 protocol

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t ConfigureMcp802xUart2(void)
{
    /* Dummy Rx Buffer read */
    ReadUART2();
    /* Wait before transmission and clear DE2flags */	
    WaitUart2(); 
	
    if (de2FlagsUart2.messageReceived)
    {
        return 1;
    }
    /* Configure UART 2 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U2STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart2.configMessage = TRUE; 
    /* Configuration Register 0 of MCP802x */
    cmdOption = CFG_0_RESERVED & CFG_0_30K_PULLUP_EN & 
                CFG_0_UV_LOCKOUT_EN & CFG_0_FET_SC_DETECT_EN &
                CFG_0_FET_OC_LIMIT_1V;

    /* This message is sent to MCP802x to configure MCP802x */
    WriteUART2(CMD_SET_CFG_0); 
    WriteUART2(cmdOption);
	
    /* Wait till acknowledgement is received from MCP802x */
    while (!de2FlagsUart2.messageReceived) 
    {
        /* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
               communication */
            return 1;                     
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart2 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart2 - 2) != SET_CFG_0_ACK)
    {
        return 1;
    }
    
    /* Wait before transmission and clear DE2flags */
    WaitUart2(); 
    if (de2FlagsUart2.messageReceived)
    {
        return 1;
    }
    /* Configure UART 2 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U2STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS;
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart2.configMessage = TRUE; 
    /* Configuration Register 1 of MCP802x */
    cmdOption = Calc_DAC_ILimit_Ref_Amps(INVA_OVER_CURRENT_AMPS);
    
    /* This message is sent to configure the motor current limit reference
    DAC */
    WriteUART2(CMD_SET_CFG_1); 
    WriteUART2(cmdOption);
    while (!de2FlagsUart2.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                   
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart2 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart2 - 2) != SET_CFG_1_ACK)
    {
        return 1;
    }
    /* Wait before transmission and clear DE2flags */
    WaitUart2(); 
    if (de2FlagsUart2.messageReceived)
    {
        return 1;
    }
    /* configure UART 2 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U2STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart2.configMessage = TRUE; 
	
    /* Configuration Register 2 of MCP802x */
    cmdOption = CFG_2_RESERVED & CFG_2_BLANKING_TIME_4us & CFG_2_DEAD_TIME_2us;
    /* This message is sent to retrieve the motor current limit reference
    DAC Configuration register*/
    WriteUART2(CMD_SET_CFG_2);

    WriteUART2(cmdOption);
    while (!de2FlagsUart2.messageReceived)
    {
        /* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                   
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    if (*(pDe2RxBufferIndexUart2 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart2 - 2) != SET_CFG_2_ACK)
    {
        return 1;
    }

    /* Reset the flag to indicate end of configuration setting for MCP8024 */
    de2FlagsUart2.configMessage = FALSE;
    /* Reset the flag to indicate end of UART communication for configuration
    messages*/
    de2FlagsUart2.messageReceived = FALSE;
    /* Configure UART 2 Rx interrupt to occur on reception of 1 character */
    U2STAbits.URXISEL = UART_INT_RX_CHAR_BITS;

    return 0;
}
// *****************************************************************************
/* Function:
    GetStatusMcp802xUart2()

  Summary:
    Routine to get Status from Driver MCP802x

  Description:
    Routine to get Status from Driver MCP802x

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t GetStatusMcp802xUart2(void)
{
    /* Dummy Rx Buffer read */
    ReadUART2(); 
    /* Wait before transmission and clear DE2flags */
    WaitUart2(); 
    if (de2FlagsUart2.messageReceived)
    {
        return 1;
    }
    /* Configure UART 2 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U2STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS; 
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart2.statusMessage = TRUE;
    /* Request for status_0 register of MCP802x */
    WriteUART2(CMD_STATUS_0); 
    while (!de2FlagsUart2.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                     
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Wait before transmission and clear DE2flags */
    WaitUart2(); 
    if (de2FlagsUart2.messageReceived)
    {
        return 1;
    }
    /* Configure UART 2 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U2STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart2.statusMessage = TRUE;
    /* Request for status_1 register of MCP802x */
    WriteUART2(CMD_STATUS_1); 
    while (!de2FlagsUart2.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                      
        }
    }
    /* Reset the timer */
    timerOut = 0; 
    de2FlagsUart2.messageReceived = FALSE;
    /* Empty driver status buffer to prevent overflow of buffer */
    U2STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    memset(driverStatusUart2, 0, sizeof (driverStatusUart2));
    /* Set pointer to start of array */
    pDe2StatusIndexUart2 = pDe2StatusIndexUart2Low; 
    /* Configure UART 2 Rx interrupt to occur on reception of 1 character*/
    U2STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    return 0;
}
// *****************************************************************************
/* Function:
    U2RXInterrupt()

  Summary:
    UART2 Interrupt service routine

  Description:
    Routine processes and stores the received UART2 data in appropriate buffer

  Precondition:
    None.

  Parameters:
	None.

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    de2FlagsUart2.messageReceived = FALSE;
    _U2RXIF = FALSE;

    if (de2FlagsUart2.configMessage)
    {
        while (DataRdyUART2())
        {
            /* If the pointer to Rx buffer has reached top of buffer break
            from this routine */
            if (pDe2RxBufferIndexUart2 == pDe2RxBufferIndexUart2High)
            {
                break;
            }
            /* Stores configuration data received from MCP802x in DE2Rx Buffer */
            *(pDe2RxBufferIndexUart2++) = ReadUART2();

        }
        /* Reset the flag to indicate completion of config message
        communication setting to buffer */
        de2FlagsUart2.configMessage = FALSE;
    }
    else if (de2FlagsUart2.statusMessage)
    {
        while (DataRdyUART2())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer
            break from this routine*/
            if (pDe2StatusIndexUart2 == pDe2StatusIndexUart2High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart2++) = ReadUART2();

        }
        /* Check the status message received from MCP802x and store in
        appropriate status indication variable */
        De2CheckStatusUart2();
        /* Reset the flag to indicate completion of storing status message
        received to buffer */
        de2FlagsUart2.statusMessage=FALSE;
    }
    /* This routine is to process unsolicited status message received
    from MCP8024 */
    else
    {
        while (DataRdyUART2())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer break
            from this routine */
            if (pDe2StatusIndexUart2 == pDe2StatusIndexUart2High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart2++) = ReadUART2();
        }
        /* Check for flag to indicate whether proper status 0
        address/acknowledgement is received from MCP8024 */
        if (de2FlagsUart2.status0AddressRx)
        {
            /* Store the status 0 register received form MCP8024 into appropriate
            variable */
            status0Uart2 = *(pDe2StatusIndexUart2 - 1);
            /* Reset the flag to indicate status 0 register is stored in appropriate
            variable */
            de2FlagsUart2.status0AddressRx = FALSE;
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            if (status0Uart2 != 0x00)
            {
                de2FlagsUart2.statusErrMessage = TRUE;
            }
        }
        /* Check for flag to indicate whether proper status 2
        address/acknowledgement is received from MCP8024 */
        else if (de2FlagsUart2.status1AddressRx)
        {
            /* Store the status 2 register received form MCP8024 into appropriate
            variable */
            status1Uart2 = *(pDe2StatusIndexUart2 - 1);

            de2FlagsUart2.status1AddressRx = FALSE;
            if (status1Uart2 != 0x00)
            {
                /* Set the flag if the fault or warning status message is received
                from MCP8024 */
                de2FlagsUart2.statusErrMessage = TRUE;
            }
        }
        else
        {
            /* Validate the unsolicited message received form MCP8024  */
            De2ValidStatusUart2();

        }
    }
    /* Set the flag to indicate the end of interrupt service routine for Uart1 */
    de2FlagsUart2.messageReceived = TRUE;
}

// *****************************************************************************
/* Function:
    De2ValidStatusUart2()

  Summary:
    Routine to validate and process unsolicited message received from MCP8024

  Description:
    Function validates and process unsolicited message received from MCP8024 
    through UART2

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2ValidStatusUart2(void)
{
    if (*(pDe2StatusIndexUart2 - 1) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart2 - 1) == CMD_STATUS_0)
    {
        /* Set the flag if the unsolicited message received indicates that
        next character will contain status 0 register of MCP8024 */
        de2FlagsUart2.status0AddressRx = TRUE;
    }
    else if ((*(pDe2StatusIndexUart2 - 1) == STATUS_1_ACK) || 
            (*(pDe2StatusIndexUart2 - 1) == CMD_STATUS_1))
    {
        /* Set the flag if the unsolicited message received indicates that next
        character will contain status 1 register of MCP8024 */
        de2FlagsUart2.status1AddressRx = TRUE;
    }
    /* If the unsolicited contains garbage empty the statusRxbuffer */
    else
    {
        /* Empty driver status buffer to prevent overflow of buffer */
         memset(driverStatusUart2, 0, sizeof (driverStatusUart2));
         /* Set the pointer to start of array */
         pDe2StatusIndexUart2 = pDe2StatusIndexUart2Low; 
    }
}
// *****************************************************************************
/* Function:
    De2CheckStatusUart2()

  Summary:
    Routine to process received message from UART2 and store it in appropriate 
	status indication variable

  Description:
    Routine to process received message from UART2 and store it in appropriate 
	status indication variable

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2CheckStatusUart2(void)
{
    /* Check if valid response acknowledgement or command to host for status
    register 0 is received*/
    if (*(pDe2StatusIndexUart2 - 2) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart2 - 2) == CMD_STATUS_0)
    {
        /* Stores status register 0 in appropriate variable */
        status0Uart2 = *(pDe2StatusIndexUart2 - 1); 
        if (status0Uart2 != 0x00)
        {
            /* Set the flag if the fault or warning status message is
            received from MCP8024 */
            de2FlagsUart2.statusErrMessage = TRUE;
        }
    }
    else if ((*(pDe2StatusIndexUart2 - 2) == STATUS_1_ACK) || 
             (*(pDe2StatusIndexUart2 - 2) == CMD_STATUS_1))
    {
        /* Check if valid response acknowledgement or command to host for
        status register 1 is received */
        status1Uart2 = *(pDe2StatusIndexUart2 - 1); 
        /* Stores status register 1 in appropriate variable */
        if (status1Uart2 != 0x00)
        {
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            de2FlagsUart2.statusErrMessage = TRUE;
        }
    }
    else
    {
        /* Empty status buffer if non-valid message received */
        memset(driverStatusUart2, 0, sizeof (driverStatusUart2));
        /* Pointer to start of array */
        pDe2StatusIndexUart2 = pDe2StatusIndexUart2Low; 
    }
}
// *****************************************************************************
/* Function:
    De2ErrorCheckUart2()

  Summary:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Description:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Warning message will not cause MCP802x to switch off gate driver. 
    Fault message will cause mcp802x to switch off gate driver 
 */
void De2ErrorCheckUart2(void)
{
    if (status0Uart2 & HIGH_TEMP_125_WARN_MASK)
    {
	/* High temp (TJ>125) warning from MCP802x */
        mcp802xWarningFlagsUart2.highTemp125 = TRUE;
    }
    if (status0Uart2 & OVER_TEMP_160_FAULT_MASK)
    {
	/* Over temp (TJ>160) fault indication from MCP802x */
        mcp802xFaultFlagsUart2.overTemp160 = TRUE;
    }
    if (status0Uart2 & INPUT_UNDERVOLT_FAULT_MASK)
    {
	/* Input under voltage fault (VDD < 5.5V) */
        mcp802xFaultFlagsUart2.inputUndervolt = TRUE;
    }
    if (status0Uart2 & INPUT_OVERVOLT_FAULT_MASK)
    {
	/* Output over voltage fault (VDD > 32V) */
        mcp802xFaultFlagsUart2.inputOvervolt = TRUE;
    }
    if (status0Uart2 & BUCK_REG_OVERCURRENT_WARN_MASK)
    {
	/* Buck regulator over current warning */
        mcp802xWarningFlagsUart2.buckRegOvercurr = TRUE;
    }
    if (status0Uart2 & BUCK_REG_UNDERVOLT_WARN_MASK)
    {
	/* Buck regulator output under voltage warning */
        mcp802xWarningFlagsUart2.buckRegUndervolt = TRUE;
    }
    if (status0Uart2 & BUCK_REG_UNDERVOLT_FAULT_MASK)
    {
	/* Buck regulator output under voltage (<80% Brown out error)*/
        mcp802xFaultFlagsUart2.buckRegUndervolt = TRUE;
    }
    if (status1Uart2 & LDO_5V_OVERCURRENT_WARN_MASK)
    {
        /* 5V LDO over current warning */
        mcp802xWarningFlagsUart2.ldo5VOvercurr = TRUE;
    }
    if (status1Uart2 & LDO_12V_OVERCURRENT_WARN_MASK)
    {
	/* 12V LDO over current warning */
        mcp802xWarningFlagsUart2.ldo12VOvercurr = TRUE;
    }
    if (status1Uart2 & EXT_MOSFET_ULVO_FAULT_MASK)
    {
	/* External MOSFET Under voltage Lock Out (UVLO) fault */
        mcp802xFaultFlagsUart2.extMosfetUvlo = TRUE;
    }
    if (status1Uart2 & EXT_MOSFET_OVER_CURRENT_FAULT_MASK)
    {
        /* External MOSFET Over current Detection fault */
        mcp802xFaultFlagsUart2.extMosfetOverCurr = TRUE;
    }
    if (status1Uart2 & BROWNOUT_RESET_MASK)
    {
	/* Brown-out Reset and CONFIG Lost (Start-up default = 1)*/
        mcp802xFaultFlagsUart2.brownOutReset = TRUE;
    }
}
// *****************************************************************************
/* Function:
    WaitUart2()

  Summary:
    Routine to set Uart2 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Description:
    Routine to set Uart2 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WaitUart2(void)
{
    /* Reset the flag to enter wait state */
    de2FlagsUart2.configMessage = FALSE; 
    /* Reset the flag to indicate no request is made by dsPIC to MCP8024 */
    de2FlagsUart2.messageReceived = FALSE; 
    /* Reset the flag to enter wait state */
    de2FlagsUart2.statusMessage = FALSE; 
    /* Configure UART 2 Rx interrupt to occur on reception of 1 character */
    U2STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Wait */
    LongDelay(10); 
}
#endif

#ifdef DE2_UART_3
/* Stores MCP802x Configuration data */
uint8_t de2RxBufferUart3[DE2_BUFFER_SIZE]; 
/* Pointer to de2RxBufferUart3 */
uint8_t *pDe2RxBufferIndexUart3 = de2RxBufferUart3; 
/* Pointer to start of array for DE2RXbufffer */
uint8_t * pDe2RxBufferIndexUart3Low = de2RxBufferUart3; 
/* Pointer to end of array for DE2Rx buffer */
uint8_t * pDe2RxBufferIndexUart3High = (de2RxBufferUart3 + DE2_BUFFER_SIZE); 
/* MCP8024 status data */
uint8_t driverStatusUart3[DE2_BUFFER_SIZE]; 
/* Pointer to driverstatusuart3 buffer */
uint8_t * pDe2StatusIndexUart3 = driverStatusUart3;
/* Pointer to start of array driverstatusUart3 */ 
uint8_t * pDe2StatusIndexUart3Low = driverStatusUart3; 
/* Pointer to end of array for driver status */
uint8_t * pDe2StatusIndexUart3High = (driverStatusUart3 + DE2_BUFFER_SIZE); 
/* Variables to store status received from MCP802x */
uint8_t status0Uart3 = 0, status1Uart3 = 0; 
/* Flags indicating various process in MCP802x communication */
volatile T_DE2_FLAGS de2FlagsUart3; 
/* Flags indicating warning faults received from MCO802x. 
   This faults will not switch off motor drivers on MCP802x */
T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart3;
/* Flags indicating error faults received from MCO802x. 
   This faults will switch off motor drivers on MCP802x */
T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart3;

// *****************************************************************************
/* Function:
    De2StartUart3()

  Summary:
    Routine to initialise UART3 for DE2 Communication

  Description:
    Routine to initialise UART3 for DE2 Communication

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    return 0 if no errors

  Remarks:
    None.
 */
uint16_t De2StartUart3(void)
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART3 module */
    CloseUART3();
	
    /* Configure UART3 receive and transmit interrupt */
    ConfigIntUART3(UART_RX_INT_EN & (UART_RX_INT_PR0 + DE2_UART_3_PRIORITY) & 
	               UART_TX_INT_DIS);

    /* Configure UART3 module to transmit 8 bit data with one stop bit. */
    de2UartModeValue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
            UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE &
            UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
            UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;


    de2UartStaValue = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO &
            UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR &
            UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
			
    /*Turn on UART 3 module and configure Rx interrupt with every character*/
    OpenUART3(de2UartModeValue, de2UartStaValue, DE2_BRG);

    return 0;
}
// *****************************************************************************
/* Function:
    ConfigureMcp802xUart3()

  Summary:
    Routine to configure MCP802x series of driver through DE2 protocol

  Description:
    Routine to configure MCP802x series of driver through DE2 protocol

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t ConfigureMcp802xUart3(void)
{
    /* Dummy Rx Buffer read */
    ReadUART3();
    /* Wait before transmission and clear DE2flags */	
    WaitUart3(); 
	
    if (de2FlagsUart3.messageReceived)
    {
        return 1;
    }
    /* Configure UART 3 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U3STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart3.configMessage = TRUE; 
    /* Configuration Register 0 of MCP802x */
    cmdOption = CFG_0_RESERVED & CFG_0_30K_PULLUP_EN & 
        CFG_0_UV_LOCKOUT_EN & CFG_0_FET_SC_DETECT_EN & CFG_0_FET_OC_LIMIT_1V;

    /* This message is sent to MCP802x to configure MCP802x */
    WriteUART3(CMD_SET_CFG_0); 
    WriteUART3(cmdOption);
	
    /* Wait till acknowledgement is received from MCP802x */
    while (!de2FlagsUart3.messageReceived) 
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                     
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart3 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart3 - 2) != SET_CFG_0_ACK)
    {
        return 1;
    }
    
    /* Wait before transmission and clear DE2flags */
    WaitUart3(); 
    if (de2FlagsUart3.messageReceived)
    {
        return 1;
    }
    /* Configure UART 3 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U3STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS;
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart3.configMessage = TRUE; 
    /* Configuration Register 1 of MCP802x */
    cmdOption = Calc_DAC_ILimit_Ref_Amps(INVB_OVER_CURRENT_AMPS);
    
    /* This message is sent to configure the motor current limit reference
    DAC */
    WriteUART3(CMD_SET_CFG_1); 
    WriteUART3(cmdOption);
    while (!de2FlagsUart3.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                     
        }

    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart3 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart3 - 2) != SET_CFG_1_ACK)
    {
        return 1;
    }
    /* Wait before transmission and clear DE2flags */
    WaitUart3(); 
    if (de2FlagsUart3.messageReceived)
    {
        return 1;
    }
    /* configure UART 3 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U3STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart3.configMessage = TRUE; 
	
    /* Configuration Register 2 of MCP802x */
    cmdOption = CFG_2_RESERVED & CFG_2_BLANKING_TIME_4us & CFG_2_DEAD_TIME_2us;
    /* This message is sent to retrieve the motor current limit reference
    DAC Configuration register*/
    WriteUART3(CMD_SET_CFG_2);

    WriteUART3(cmdOption);
    while (!de2FlagsUart3.messageReceived)
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                      
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    if (*(pDe2RxBufferIndexUart3 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart3 - 2) != SET_CFG_2_ACK)
    {
        return 1;
    }

    /* Check for valid response from MCP802x */
    de2FlagsUart3.configMessage = FALSE;
    /* Reset the flag to indicate end of configuration setting for MCP8024 */
    de2FlagsUart3.messageReceived = FALSE;
    /* Reset the flag to indicate end of UART communication for configuration
    messages*/
    U3STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Configure UART 3 Rx interrupt to occur on reception of 1 character */
    return 0;
}
// *****************************************************************************
/* Function:
    GetStatusMcp802xUart3()

  Summary:
    Routine to get Status from Driver MCP802x

  Description:
    Routine to get Status from Driver MCP802x

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t GetStatusMcp802xUart3(void)
{
    /* Dummy Rx Buffer read */
    ReadUART3(); 
    /* Wait before transmission and clear DE2flags */
    WaitUart3(); 
    if (de2FlagsUart3.messageReceived)
    {
        return 1;
    }
    /* Configure UART 3 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U3STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS; 
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart3.statusMessage = TRUE;
    /* Request for status_0 register of MCP802x */
    WriteUART3(CMD_STATUS_0); 
    while (!de2FlagsUart3.messageReceived)
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                   
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    /* Wait before transmission and clear DE2flags */
    WaitUart3(); 
    if (de2FlagsUart3.messageReceived)
    {
        return 1;
    }
    /* Configure UART 3 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U3STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart3.statusMessage = TRUE;
    /* Request for status_1 register of MCP802x */
    WriteUART3(CMD_STATUS_1); 
    while (!de2FlagsUart3.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                
        }
    }
    /* Reset the timer */
    timerOut = 0; 
    de2FlagsUart3.messageReceived = FALSE;
    /* Empty driver status buffer to prevent overflow of buffer */
    U3STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    memset(driverStatusUart3, 0, sizeof (driverStatusUart3));
    /* Set pointer to start of array */
    pDe2StatusIndexUart3 = pDe2StatusIndexUart3Low; 
    /* Configure UART 3 Rx interrupt to occur on reception of 1 character*/
    U3STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    return 0;
}
// *****************************************************************************
/* Function:
    U3RXInterrupt()

  Summary:
    UART3 Interrupt service routine

  Description:
    Routine processes and stores the received UART3 data in appropriate buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U3RXInterrupt(void)
{
    de2FlagsUart3.messageReceived = FALSE;
    _U3RXIF = FALSE;

    if (de2FlagsUart3.configMessage)
    {
        while (DataRdyUART3())
        {
            /* If the pointer to Rx buffer has reached top of buffer break
            from this routine */
            if (pDe2RxBufferIndexUart3 == pDe2RxBufferIndexUart3High)
            {
                break;
            }
            /* Stores configuration data received from MCP802x in DE2Rx Buffer */
            *(pDe2RxBufferIndexUart3++) = ReadUART3();

        }
        /* Reset the flag to indicate completion of config message
        communication setting to buffer */
        de2FlagsUart3.configMessage = FALSE;
    }
    else if (de2FlagsUart3.statusMessage)
    {
        while (DataRdyUART3())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer
            break from this routine*/
            if (pDe2StatusIndexUart3 == pDe2StatusIndexUart3High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart3++) = ReadUART3();

        }
        /* Check the status message received from MCP802x and store in
        appropriate status indication variable */
        De2CheckStatusUart3();
        /* Reset the flag to indicate completion of storing status message
        received to buffer */
        de2FlagsUart3.statusMessage=FALSE;
    }
    /* This routine is to process unsolicited status message received
    from MCP8024 */
    else
    {

        while (DataRdyUART3())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer break
            from this routine */
            if (pDe2StatusIndexUart3 == pDe2StatusIndexUart3High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart3++) = ReadUART3();

        }
        /* Check for flag to indicate whether proper status 0
        address/acknowledgement is received from MCP8024 */
        if (de2FlagsUart3.status0AddressRx)
        {
            /* Store the status 0 register received form MCP8024 into appropriate
            variable */
            status0Uart3 = *(pDe2StatusIndexUart3 - 1);
            /* Reset the flag to indicate status 0 register is stored in appropriate
            variable */
            de2FlagsUart3.status0AddressRx = FALSE;
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            if (status0Uart3 != 0x00)
            {
                de2FlagsUart3.statusErrMessage = TRUE;
            }
        }
        /* Check for flag to indicate whether proper status 1
        address/acknowledgement is received from MCP8024 */
        else if (de2FlagsUart3.status1AddressRx)
        {
            /* Store the status 1 register received form MCP8024 into appropriate
            variable */
            status1Uart3 = *(pDe2StatusIndexUart3 - 1);

            de2FlagsUart3.status1AddressRx = FALSE;
            if (status1Uart3 != 0x00)
            {
                /* Set the flag if the fault or warning status message is received
                from MCP8024 */
                de2FlagsUart3.statusErrMessage = TRUE;
            }
        }
        else
        {
            /* Validate the unsolicited message received form MCP8024  */
            De2ValidStatusUart3();
        }
    }
    /* Set the flag to indicate the end of interrupt service routine for Uart3 */
    de2FlagsUart3.messageReceived = TRUE;
}

// *****************************************************************************
/* Function:
    De2ValidStatusUart3()

  Summary:
    Routine to validate and process unsolicited message received from MCP8024

  Description:
    Function validates and process unsolicited message received from MCP8024
    through UART3

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2ValidStatusUart3(void)
{
    if (*(pDe2StatusIndexUart3 - 1) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart3 - 1) == CMD_STATUS_0)
    {
        /* Set the flag if the unsolicited message received indicates that
        next character will contain status 0 register of MCP8024 */
        de2FlagsUart3.status0AddressRx = TRUE;
    }
    else if ((*(pDe2StatusIndexUart3 - 1) == STATUS_1_ACK) || 
            (*(pDe2StatusIndexUart3 - 1) == CMD_STATUS_1))
    {
        /* Set the flag if the unsolicited message received indicates that next
        character will contain status 1 register of MCP8024 */
        de2FlagsUart3.status1AddressRx = TRUE;
    }
    /* If the unsolicited contains garbage empty the statusRxbuffer */
    else
    {
	/* Empty driver status buffer to prevent overflow of buffer */
         memset(driverStatusUart3, 0, sizeof (driverStatusUart3));
        /* Set the pointer to start of array */
         pDe2StatusIndexUart3 = pDe2StatusIndexUart3Low; 
    }
}
// *****************************************************************************
/* Function:
    De2CheckStatusUart3()

  Summary:
    Routine to process received message from UART3 and store it in appropriate
    status indication variable

  Description:
    Routine to process received message from UART3 and store it in appropriate
    status indication variable

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2CheckStatusUart3(void)
{
    /* Check if valid response acknowledgement or command to host for status
    register 0 is received*/
    if (*(pDe2StatusIndexUart3 - 2) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart3 - 2) == CMD_STATUS_0)
    {
	/* Stores status register 0 in appropriate variable */
        status0Uart3 = *(pDe2StatusIndexUart3 - 1); 
        if (status0Uart3 != 0x00)
        {
            /* Set the flag if the fault or warning status message is
            received from MCP8024 */
            de2FlagsUart3.statusErrMessage = TRUE;
        }
    }
    else if ((*(pDe2StatusIndexUart3 - 2) == STATUS_1_ACK) || 
            (*(pDe2StatusIndexUart3 - 2) == CMD_STATUS_1))
    {
        /* Check if valid response acknowledgement or command to host for
        status register 1 is received */
        status1Uart3 = *(pDe2StatusIndexUart3 - 1); 
	/* Stores status register 1 in appropriate variable */
        if (status1Uart3 != 0x00)
        {
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            de2FlagsUart3.statusErrMessage = TRUE;
        }
    }
    else
    {
	/* Empty status buffer if non-valid message received */
        memset(driverStatusUart3, 0, sizeof (driverStatusUart3));
	/* Pointer to start of array */
        pDe2StatusIndexUart3 = pDe2StatusIndexUart3Low; 
    }
}
// *****************************************************************************
/* Function:
    De2ErrorCheckUart3()

  Summary:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Description:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Warning message will not cause MCP802x to switch off gate driver. 
    Fault message will cause mcp802x to switch off gate driver 
 */
void De2ErrorCheckUart3(void)
{
    if (status0Uart3 & HIGH_TEMP_125_WARN_MASK)
    {
	/* High temp (TJ>125) warning from MCP802x */
        mcp802xWarningFlagsUart3.highTemp125 = TRUE;
    }
    if (status0Uart3 & OVER_TEMP_160_FAULT_MASK)
    {
	/* Over temp (TJ>160) fault indication from MCP802x */
        mcp802xFaultFlagsUart3.overTemp160 = TRUE;
    }
    if (status0Uart3 & INPUT_UNDERVOLT_FAULT_MASK)
    {
	/* Input under voltage fault (VDD < 5.5V) */
        mcp802xFaultFlagsUart3.inputUndervolt = TRUE;
    }
    if (status0Uart3 & INPUT_OVERVOLT_FAULT_MASK)
    {
	/* Output over voltage fault (VDD > 32V) */
        mcp802xFaultFlagsUart3.inputOvervolt = TRUE;
    }
    if (status0Uart3 & BUCK_REG_OVERCURRENT_WARN_MASK)
    {
	/* Buck regulator over current warning */
        mcp802xWarningFlagsUart3.buckRegOvercurr = TRUE;
    }
    if (status0Uart3 & BUCK_REG_UNDERVOLT_WARN_MASK)
    {
	/* Buck regulator output under voltage warning */
        mcp802xWarningFlagsUart3.buckRegUndervolt = TRUE;
    }
    if (status0Uart3 & BUCK_REG_UNDERVOLT_FAULT_MASK)
    {
        /* Buck regulator output under voltage (<80% Brown out error)*/
        mcp802xFaultFlagsUart3.buckRegUndervolt = TRUE;
    }
    if (status1Uart3 & LDO_5V_OVERCURRENT_WARN_MASK)
    {
        /* 5V LDO over current warning */
        mcp802xWarningFlagsUart3.ldo5VOvercurr = TRUE;
    }
    if (status1Uart3 & LDO_12V_OVERCURRENT_WARN_MASK)
    {
	/* 12V LDO over current warning */
        mcp802xWarningFlagsUart3.ldo12VOvercurr = TRUE;
    }
    if (status1Uart3 & EXT_MOSFET_ULVO_FAULT_MASK)
    {
        /* External MOSFET Under voltage Lock Out (UVLO) fault */
        mcp802xFaultFlagsUart3.extMosfetUvlo = TRUE;
    }
    if (status1Uart3 & EXT_MOSFET_OVER_CURRENT_FAULT_MASK)
    {
	/* External MOSFET Over current Detection fault */
        mcp802xFaultFlagsUart3.extMosfetOverCurr = TRUE;
    }
    if (status1Uart3 & BROWNOUT_RESET_MASK)
    {
	/* Brown-out Reset and CONFIG Lost (Start-up default = 1)*/
        mcp802xFaultFlagsUart3.brownOutReset = TRUE;
    }
}
// *****************************************************************************
/* Function:
    WaitUart3()

  Summary:
    Routine to set UART3 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Description:
    Routine to set UART3 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WaitUart3(void)
{
    /* Reset the flag to enter wait state */
    de2FlagsUart3.configMessage = FALSE; 
    /* Reset the flag to indicate no request is made by dsPIC to MCP8024 */
    de2FlagsUart3.messageReceived = FALSE; 
    /* Reset the flag to enter wait state */
    de2FlagsUart3.statusMessage = FALSE; 
    /* Configure UART 3 Rx interrupt to occur on reception of 1 character */
    U3STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Wait */
    LongDelay(10); 
}
#endif

#ifdef DE2_UART_4
/* Stores MCP802x Configuration data */
uint8_t de2RxBufferUart4[DE2_BUFFER_SIZE]; 
/* Pointer to de2RxBufferUart4 */
uint8_t *pDe2RxBufferIndexUart4 = de2RxBufferUart4; 
/* Pointer to start of array for DE2RXbufffer */
uint8_t * pDe2RxBufferIndexUart4Low = de2RxBufferUart4; 
/* Pointer to end of array for DE2Rx buffer */
uint8_t * pDe2RxBufferIndexUart4High = (de2RxBufferUart4 + DE2_BUFFER_SIZE); 
/* MCP8024 status data */
uint8_t driverStatusUart4[DE2_BUFFER_SIZE]; 
/* Pointer to driverstatusuart4 buffer */
uint8_t * pDe2StatusIndexUart4 = driverStatusUart4;
/* Pointer to start of array driverstatusUart4 */ 
uint8_t * pDe2StatusIndexUart4Low = driverStatusUart4; 
/* Pointer to end of array for driver status */
uint8_t * pDe2StatusIndexUart4High = (driverStatusUart4 + DE2_BUFFER_SIZE); 
/* Variables to store status received from MCP802x */
uint8_t status0Uart4 = 0, status1Uart4 = 0; 
/* Flags indicating various process in MCP802x communication */
volatile T_DE2_FLAGS de2FlagsUart4; 
/* Flags indicating warning faults received from MCO802x. 
   This faults will not switch off motor drivers on MCP802x */
T_MCP802x_WARNING_FLAGS mcp802xWarningFlagsUart4;
/* Flags indicating error faults received from MCO802x. 
   This faults will switch off motor drivers on MCP802x */
T_MCP802x_FAULT_FLAGS mcp802xFaultFlagsUart4;

// *****************************************************************************
/* Function:
    De2StartUart4()

  Summary:
    Routine to initialise UART4 for DE2 Communication

  Description:
    Routine to initialise UART4 for DE2 Communication

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    return 0 if no errors

  Remarks:
    None.
 */
uint16_t De2StartUart4(void)
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART4 module */
    CloseUART4();
	
    /* Configure UART4 receive and transmit interrupt */
    ConfigIntUART4(UART_RX_INT_EN & (UART_RX_INT_PR0 + DE2_UART_4_PRIORITY) & 
	           UART_TX_INT_DIS);

    /* Configure UART4 module to transmit 8 bit data with one stop bit. */
    de2UartModeValue =  UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                        UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE &
                        UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                        UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;


    de2UartStaValue = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO &
                    UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR &
                    UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;
			
    /*Turn on UART 4 module and configure Rx interrupt with every character*/
    OpenUART4(de2UartModeValue, de2UartStaValue, DE2_BRG);

    return 0;
}
// *****************************************************************************
/* Function:
    ConfigureMcp802xUart4()

  Summary:
    Routine to configure MCP802x series of driver through DE2 protocol

  Description:
    Routine to configure MCP802x series of driver through DE2 protocol

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t ConfigureMcp802xUart4(void)
{
    /* Dummy Rx Buffer read */
    ReadUART4();
    /* Wait before transmission and clear DE2flags */	
    WaitUart4(); 
	
    if (de2FlagsUart4.messageReceived)
    {
        return 1;
    }
    /* Configure UART 4 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U4STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart4.configMessage = TRUE; 
    /* Configuration Register 0 of MCP802x */
    cmdOption = CFG_0_RESERVED & CFG_0_30K_PULLUP_EN & 
                CFG_0_UV_LOCKOUT_EN & CFG_0_FET_SC_DETECT_EN &
                CFG_0_FET_OC_LIMIT_1V;

    /* This message is sent to MCP802x to configure MCP802x */
    WriteUART4(CMD_SET_CFG_0); 
    WriteUART4(cmdOption);
	
    /* Wait till acknowledgement is received from MCP802x */
    while (!de2FlagsUart4.messageReceived) 
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                     
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart4 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart4 - 2) != SET_CFG_0_ACK)
    {
        return 1;
    }
    
    /* Wait before transmission and clear DE2flags */
    WaitUart4(); 
    if (de2FlagsUart4.messageReceived)
    {
        return 1;
    }
    /* Configure UART 4 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U4STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS;
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart4.configMessage = TRUE; 
    /* Configuration Register 1 of MCP802x */
    cmdOption = Calc_DAC_ILimit_Ref_Amps(INVA_OVER_CURRENT_AMPS);
    
    /* This message is sent to configure the motor current limit reference
    DAC */
    WriteUART4(CMD_SET_CFG_1); 
    WriteUART4(cmdOption);
    while (!de2FlagsUart4.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                  
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    /* Check for valid response from MCP802x */
    if (*(pDe2RxBufferIndexUart4 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart4 - 2) != SET_CFG_1_ACK)
    {
        return 1;
    }
    /* Wait before transmission and clear DE2flags */
    WaitUart4(); 
    if (de2FlagsUart4.messageReceived)
    {
        return 1;
    }
    /* configure UART 4 Rx interrupt to occur on reception of 4 character
    (Full Buffer)*/
    U4STAbits.URXISEL = UART_INT_RX_BUF_FUL_BITS; 
    /* Set the flag to indicate config message has being sent to MCP802x */
    de2FlagsUart4.configMessage = TRUE; 
	
    /* Configuration Register 2 of MCP802x */
    cmdOption = CFG_2_RESERVED & CFG_2_BLANKING_TIME_4us & CFG_2_DEAD_TIME_2us;
    /* This message is sent to retrieve the motor current limit reference
    DAC Configuration register*/
    WriteUART4(CMD_SET_CFG_2);

    WriteUART4(cmdOption);
    while (!de2FlagsUart4.messageReceived)
    {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                 
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    if (*(pDe2RxBufferIndexUart4 - 1) != cmdOption || 
	*(pDe2RxBufferIndexUart4 - 2) != SET_CFG_2_ACK)
    {
        return 1;
    }

    /* Check for valid response from MCP802x */
    de2FlagsUart4.configMessage = FALSE;
    /* Reset the flag to indicate end of configuration setting for MCP8024 */
    de2FlagsUart4.messageReceived = FALSE;
    /* Reset the flag to indicate end of UART communication for configuration
    messages*/
    U4STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Configure UART 4 Rx interrupt to occur on reception of 1 character */

    return 0;
}
// *****************************************************************************
/* Function:
    GetStatusMcp802xUart4()

  Summary:
    Routine to get Status from Driver MCP802x

  Description:
    Routine to get Status from Driver MCP802x

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Return 1 on occurrence of error

  Remarks:
    None.
 */
uint16_t GetStatusMcp802xUart4(void)
{
    /* Dummy Rx Buffer read */
    ReadUART4(); 
    /* Wait before transmission and clear DE2flags */
    WaitUart4(); 
    if (de2FlagsUart4.messageReceived)
    {
        return 1;
    }
    /* Configure UART 4 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U4STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS; 
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart4.statusMessage = TRUE;
    /* Request for status_0 register of MCP802x */
    WriteUART4(CMD_STATUS_0); 
    while (!de2FlagsUart4.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;               
        }
    }
    /* Reset the timer */
    timerOut = 0;  
    /* Wait before transmission and clear DE2flags */
    WaitUart4(); 
    if (de2FlagsUart4.messageReceived)
    {
        return 1;
    }
    /* Configure UART 4 Rx interrupt to occur on reception of 3 character
    (3/4 Buffer full) */
    U4STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    /* Set the flag to indicate status request has being sent to MCP802x */
    de2FlagsUart4.statusMessage = TRUE;
    /* Request for status_1 register of MCP802x */
    WriteUART4(CMD_STATUS_1); 
    while (!de2FlagsUart4.messageReceived)
     {
	/* Timer to prevent improper UART communication */
        timerOut++;        
        if (timerOut >= CountsinMicrosecond(TIMER_OUT_COUNTS))
        {
            /* Return to indicate that there is problem with UART
            communication */
            return 1;                    
        }
    }
    /* Reset the timer */
    timerOut = 0; 
    de2FlagsUart4.messageReceived = FALSE;
    /* Empty driver status buffer to prevent overflow of buffer */
    U4STAbits.URXISEL = UART_INT_RX_3_4_FUL_BITS;
    memset(driverStatusUart4, 0, sizeof (driverStatusUart4));
    /* Set pointer to start of array */
    pDe2StatusIndexUart4 = pDe2StatusIndexUart4Low; 
    /* Configure UART 4 Rx interrupt to occur on reception of 1 character*/
    U4STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    return 0;
}
// *****************************************************************************
/* Function:
    U4RXInterrupt()

  Summary:
    UART4 Interrupt service routine

  Description:
    Routine processes and stores the received UART4 data in appropriate buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U4RXInterrupt(void)
{
    de2FlagsUart4.messageReceived = FALSE;
    _U4RXIF = FALSE;

    if (de2FlagsUart4.configMessage)
    {
        while (DataRdyUART4())
        {
            /* If the pointer to Rx buffer has reached top of buffer break
            from this routine */
            if (pDe2RxBufferIndexUart4 == pDe2RxBufferIndexUart4High)
            {
                break;
            }
            /* Stores configuration data received from MCP802x in DE2Rx Buffer */
            *(pDe2RxBufferIndexUart4++) = ReadUART4();

        }
        /* Reset the flag to indicate completion of config message
        communication setting to buffer */
        de2FlagsUart4.configMessage = FALSE;

    }
    else if (de2FlagsUart4.statusMessage)
    {
        while (DataRdyUART4())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer
            break from this routine*/
            if (pDe2StatusIndexUart4 == pDe2StatusIndexUart4High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart4++) = ReadUART4();

        }
        /* Check the status message received from MCP802x and store in
        appropriate status indication variable */
        De2CheckStatusUart4();
        /* Reset the flag to indicate completion of storing status message
        received to buffer */
        de2FlagsUart4.statusMessage=FALSE;

    }
    /* This routine is to process unsolicited status message received
    from MCP8024 */
    else
    {

        while (DataRdyUART4())
        {
            /* If the pointer to statusrRxbuffer has reached top of buffer break
            from this routine */
            if (pDe2StatusIndexUart4 == pDe2StatusIndexUart4High)
            {
                break;
            }
            /* Stores status data received from MCP802x in DE2_status_UART
            Buffer */
            *(pDe2StatusIndexUart4++) = ReadUART4();

        }
        /* Check for flag to indicate whether proper status 0
        address/acknowledgement is received from MCP8024 */
        if (de2FlagsUart4.status0AddressRx)
        {
            /* Store the status 0 register received form MCP8024 into appropriate
            variable */
            status0Uart4 = *(pDe2StatusIndexUart4 - 1);
            /* Reset the flag to indicate status 0 register is stored in appropriate
            variable */
            de2FlagsUart4.status0AddressRx = FALSE;
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            if (status0Uart4 != 0x00)
            {
                de2FlagsUart4.statusErrMessage = TRUE;
            }
        }
        /* Check for flag to indicate whether proper status 1
        address/acknowledgement is received from MCP8024 */
        else if (de2FlagsUart4.status1AddressRx)
        {
            /* Store the status 1 register received form MCP8024 into appropriate
            variable */
            status1Uart4 = *(pDe2StatusIndexUart4 - 1);

            de2FlagsUart4.status1AddressRx = FALSE;
            if (status1Uart4 != 0x00)
            {
                /* Set the flag if the fault or warning status message is received
                from MCP8024 */
                de2FlagsUart4.statusErrMessage = TRUE;
            }
        }
        else
        {
            /* Validate the unsolicited message received form MCP8024  */
            De2ValidStatusUart4();
        }
    }
    /* Set the flag to indicate the end of interrupt service routine for Uart4 */
    de2FlagsUart4.messageReceived = TRUE;
}

// *****************************************************************************
/* Function:
    De2ValidStatusUart4()

  Summary:
    Routine to validate and process unsolicited message received from MCP8024

  Description:
    Function validates and process unsolicited message received from MCP8024
    through UART4

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2ValidStatusUart4(void)
{
    if (*(pDe2StatusIndexUart4 - 1) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart4 - 1) == CMD_STATUS_0)
    {
        /* Set the flag if the unsolicited message received indicates that
        next character will contain status 0 register of MCP8024 */
        de2FlagsUart4.status0AddressRx = TRUE;
    }
    else if ((*(pDe2StatusIndexUart4 - 1) == STATUS_1_ACK) || 
            (*(pDe2StatusIndexUart4 - 1) == CMD_STATUS_1))
    {
        /* Set the flag if the unsolicited message received indicates that next
        character will contain status 1 register of MCP8024 */
        de2FlagsUart4.status1AddressRx = TRUE;
    }
    /* If the unsolicited contains garbage empty the statusRxbuffer */
    else
    {
	/* Empty driver status buffer to prevent overflow of buffer */
         memset(driverStatusUart4, 0, sizeof (driverStatusUart4));
	/* Set the pointer to start of array */
         pDe2StatusIndexUart4 = pDe2StatusIndexUart4Low; 
    }
}
// *****************************************************************************
/* Function:
    De2CheckStatusUart4()

  Summary:
    Routine to process received message from UART4 and store it in appropriate
    status indication variable

  Description:
    Routine to process received message from UART4 and store it in appropriate
    status indication variable

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void De2CheckStatusUart4(void)
{
    /* Check if valid response acknowledgement or command to host for status
    register 0 is received*/
    if (*(pDe2StatusIndexUart4 - 2) == STATUS_0_ACK || 
	*(pDe2StatusIndexUart4 - 2) == CMD_STATUS_0)
    {
	/* Stores status register 0 in appropriate variable */
        status0Uart4 = *(pDe2StatusIndexUart4 - 1); 
        if (status0Uart4 != 0x00)
        {
            /* Set the flag if the fault or warning status message is
            received from MCP8024 */
            de2FlagsUart4.statusErrMessage = TRUE;
        }
    }
    else if ((*(pDe2StatusIndexUart4 - 2) == STATUS_1_ACK) || 
            (*(pDe2StatusIndexUart4 - 2) == CMD_STATUS_1))
    {
        /* Check if valid response acknowledgement or command to host for
        status register 1 is received */
        status1Uart4 = *(pDe2StatusIndexUart4 - 1); 
	/* Stores status register 1 in appropriate variable */
        if (status1Uart4 != 0x00)
        {
            /* Set the flag if the fault or warning status message is received
            from MCP8024 */
            de2FlagsUart4.statusErrMessage = TRUE;
        }
    }
    else
    {
	/* Empty status buffer if non-valid message received */
        memset(driverStatusUart4, 0, sizeof (driverStatusUart4));
	/* Pointer to start of array */
        pDe2StatusIndexUart4 = pDe2StatusIndexUart4Low; 
    }
}
// *****************************************************************************
/* Function:
    De2ErrorCheckUart4()

  Summary:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Description:
    Routine to process status messages from MCP802x and set error flag
    accordingly

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Warning message will not cause MCP802x to switch off gate driver. 
    Fault message will cause mcp802x to switch off gate driver 
 */
void De2ErrorCheckUart4(void)
{
    if (status0Uart4 & HIGH_TEMP_125_WARN_MASK)
    {
	/* High temp (TJ>125) warning from MCP802x */
        mcp802xWarningFlagsUart4.highTemp125 = TRUE;
    }
    if (status0Uart4 & OVER_TEMP_160_FAULT_MASK)
    {
	/* Over temp (TJ>160) fault indication from MCP802x */
        mcp802xFaultFlagsUart4.overTemp160 = TRUE;
    }
    if (status0Uart4 & INPUT_UNDERVOLT_FAULT_MASK)
    {
	/* Input under voltage fault (VDD < 5.5V) */
        mcp802xFaultFlagsUart4.inputUndervolt = TRUE;
    }
    if (status0Uart4 & INPUT_OVERVOLT_FAULT_MASK)
    {
	/* Output over voltage fault (VDD > 32V) */
        mcp802xFaultFlagsUart4.inputOvervolt = TRUE;
    }
    if (status0Uart4 & BUCK_REG_OVERCURRENT_WARN_MASK)
    {
	/* Buck regulator over current warning */
        mcp802xWarningFlagsUart4.buckRegOvercurr = TRUE;
    }
    if (status0Uart4 & BUCK_REG_UNDERVOLT_WARN_MASK)
    {
	/* Buck regulator output under voltage warning */
        mcp802xWarningFlagsUart4.buckRegUndervolt = TRUE;
    }
    if (status0Uart4 & BUCK_REG_UNDERVOLT_FAULT_MASK)
    {
        /* Buck regulator output under voltage (<80% Brown out error)*/
        mcp802xFaultFlagsUart4.buckRegUndervolt = TRUE;
    }
    if (status1Uart4 & LDO_5V_OVERCURRENT_WARN_MASK)
    {
	/* 5V LDO over current warning */
        mcp802xWarningFlagsUart4.ldo5VOvercurr = TRUE;
    }
    if (status1Uart4 & LDO_12V_OVERCURRENT_WARN_MASK)
    {
	/* 12V LDO over current warning */
        mcp802xWarningFlagsUart4.ldo12VOvercurr = TRUE;
    }
    if (status1Uart4 & EXT_MOSFET_ULVO_FAULT_MASK)
    {
	/* External MOSFET Under voltage Lock Out (UVLO) fault */
        mcp802xFaultFlagsUart4.extMosfetUvlo = TRUE;
    }
    if (status1Uart4 & EXT_MOSFET_OVER_CURRENT_FAULT_MASK)
    {
	/* External MOSFET Over current Detection fault */
        mcp802xFaultFlagsUart4.extMosfetOverCurr = TRUE;
    }
    if (status1Uart4 & BROWNOUT_RESET_MASK)
    {
	/* Brown-out Reset and CONFIG Lost (Start-up default = 1)*/
        mcp802xFaultFlagsUart4.brownOutReset = TRUE;
    }
}
// *****************************************************************************
/* Function:
    WaitUart4()

  Summary:
    Routine to set UART4 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Description:
    Routine to set UART4 Rx interrupt to occur on reception of 1 character and
    wait in between configuration and status messages processing

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void WaitUart4(void)
{
    /* Reset the flag to enter wait state */
    de2FlagsUart4.configMessage = FALSE; 
    /* Reset the flag to indicate no request is made by dsPIC to MCP8024 */
    de2FlagsUart4.messageReceived = FALSE; 
    /* Reset the flag to enter wait state */
    de2FlagsUart4.statusMessage = FALSE; 
    /* Configure UART 1 Rx interrupt to occur on reception of 1 character */
    U4STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
    /* Wait */
    LongDelay(10); 
}
#endif

/******************************************************************************
 * Function:    StatusCheckMcp802x()
 *
 * Output:	None
 *
 * Overview:	Routine to check status message from MCP8024
 *
 *******************************************************************************/
void StatusCheckMcp802x(uint16_t de2_UART_Module)
{
    #ifdef DE2_UART_1
    if(de2_UART_Module == DE2_UART_1)
    {
	/* Execute if appropriate Status register is received from MCP802x*/
        if (de2FlagsUart1.statusErrMessage)
        {
            /* Check for status register from MCP802x */
            GetStatusMcp802xUart1();
            /* Set the appropriate error indication flag */
            De2ErrorCheckUart1();
            /* Reset the flag to indicate the end of processing status messages
               received from MCP8024 */
            de2FlagsUart1.statusErrMessage = 0;
        }
    }
    #endif
    #ifdef DE2_UART_2
    if(de2_UART_Module == DE2_UART_2)
    {
	/* Execute if appropriate Status register is received from MCP802x*/
        if (de2FlagsUart2.statusErrMessage)
        {
            /* Check for status register from MCP802x */
            GetStatusMcp802xUart2();
            /* Set the appropriate error indication flag */
            De2ErrorCheckUart2();
            /* Reset the flag to indicate the end of processing status messages
               received from MCP8024 */
            de2FlagsUart2.statusErrMessage = 0;
        }
    }
    #endif
    #ifdef DE2_UART_3
    if(de2_UART_Module == DE2_UART_3)
    {
	/* Execute if appropriate Status register is received from MCP802x*/
        if (de2FlagsUart3.statusErrMessage)
        {
            /* Check for status register from MCP802x */
            GetStatusMcp802xUart3();
            /* Set the appropriate error indication flag */
            De2ErrorCheckUart3();
            /* Reset the flag to indicate the end of processing status messages
               received from MCP8024 */
            de2FlagsUart3.statusErrMessage = 0;
        }
    }
    #endif
    #ifdef DE2_UART_4
    if(de2_UART_Module == DE2_UART_4)
    {
	/* Execute if appropriate Status register is received from MCP802x*/
        if (de2FlagsUart4.statusErrMessage)
        {
            /* Check for status register from MCP802x */
            GetStatusMcp802xUart4();
            /* Set the appropriate error indication flag */
            De2ErrorCheckUart4();
            /* Reset the flag to indicate the end of processing status messages
               received from MCP8024 */
            de2FlagsUart4.statusErrMessage = 0;
        }
    }
    #endif
}

/******************************************************************************
 * Function:    ConfigureMcp8024()
 *
 * Output:	None
 *
 * Overview:	Routine to initialise MCP8024 parameters through DE2 protocol
 *
 *******************************************************************************/

void ConfigureMcp8024(uint16_t de2UARTModule)
{
    #ifdef DE2_UART_1
    if(de2UARTModule == DE2_UART_1)
    {
	/* Setting up UART1 for DE2 communication with MCP8024 */
        De2StartUart1();
	/* Wait */
        LongDelay(500);  
	/* Setting up configuration register in MCP8024 */
        de2FlagsUart1.configSettingErr = ConfigureMcp802xUart1(); 
        if (de2FlagsUart1.configSettingErr)
        {
            /* Error in Configuring Configuration register of mcp8024 */
            U1STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
            while (1)
            {
                TOGGLE_LED1;
                /* Code will be stuck here and LED1 will blink to indicate error
                   has occurred in UART communication while setting configuration
                   bits for MCP802x*/
                DelayinMicrosecond(2);
            }
        }
    }
    #endif
    #ifdef DE2_UART_2
    if(de2UARTModule == DE2_UART_2)
    {
	/* Setting up UART2 for DE2 communication with MCP8024 */
        De2StartUart2();
	/* Wait */
        LongDelay(500);  
	/* Setting up configuration register in MCP8024 */
        de2FlagsUart2.configSettingErr = ConfigureMcp802xUart2(); 
        if (de2FlagsUart2.configSettingErr)
        {
            /* Error in Configuring Configuration register of mcp8024 */
            U2STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
            while (1)
            {
                TOGGLE_LED1;
                /* Code will be stuck here and LED1 will blink to indicate error
                   has occurred in UART communication while setting configuration
                   bits for MCP802x */
                DelayinMicrosecond(2);
            }
        }
    }
    #endif
    #ifdef DE2_UART_3
    if(de2UARTModule == DE2_UART_3)
    {
	/* Setting up UART3 for DE2 communication with MCP8024 */
        De2StartUart3();
        /* Wait */
        LongDelay(500);  
	/* Setting up configuration register in MCP8024 */
        de2FlagsUart3.configSettingErr = ConfigureMcp802xUart3(); 
        if (de2FlagsUart3.configSettingErr)
        {
            /* Error in Configuring Configuration register of mcp8024 */
            U3STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
            while (1)
            {
                TOGGLE_LED2;
                /* Code will be stuck here and LED2 will blink to indicate error
                   has occurred in UART communication while setting configuration
                   bits for MCP802x*/
                DelayinMicrosecond(2);
            }
        }
    }
    #endif
    #ifdef DE2_UART_4
    if(de2UARTModule == DE2_UART_4)
    {
	/* Setting up UART4 for DE2 communication with MCP8024 */
        De2StartUart4();
	/* Wait */
        LongDelay(500);  
	/* Setting up configuration register in MCP8024 */
        de2FlagsUart4.configSettingErr = ConfigureMcp802xUart4(); 
        if (de2FlagsUart4.configSettingErr)
        {
            /* Error in Configuring Configuration register of mcp8024 */
            U4STAbits.URXISEL = UART_INT_RX_CHAR_BITS;
            while (1)
            {
                TOGGLE_LED2;
                /* Code will be stuck here and LED2 will blink to indicate error
                   has occurred in UART communication while setting configuration
                   bits for MCP802x*/
                DelayinMicrosecond(2);
            }
        }
    }
    #endif
}


/* EOF */
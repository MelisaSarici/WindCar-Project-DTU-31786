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

#include "uart.h"


#ifdef _U1RXIF
// *****************************************************************************
/* Function:
    BusyUART1()

  Summary:
    Routine to check if the transmission is progress

  Description:
    This returns status whether the transmission is in progress or not,
    by checking Status bit TRMT

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Info about whether transmission is in progress.

  Remarks:
    None.
 */
char BusyUART1(void)
{
    return (!U1STAbits.TRMT);
}
// *****************************************************************************
/* Function:
    CloseUART1()

  Summary:
    Routine to disable the UART module

  Description:
    This function disables the UART and clears the interrupt enable and flag
    bits

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void CloseUART1(void)
{
    U1MODEbits.UARTEN = 0;

    _U1RXIE = 0;
    _U1TXIE = 0;

    _U1RXIF = 0;
    _U1TXIF = 0;
}
// *****************************************************************************
/* Function:
    ConfigIntUART1()

  Summary:
    Function to configure UART Rx and Tx Interrupts

  Description:
    This function sets priority for  RX and TX interrupt and enable/disables
    the interrupt

  Precondition:
    None.

  Parameters:
    unsigned int config enable/disable and priority.

  Returns:
    None.

  Remarks:
    None.
 */
void ConfigIntUART1(uint16_t config)
{
    /* clear IF flags */
    _U1RXIF = 0;
    _U1TXIF = 0;

    /* set priority */
    _U1RXIP = 0x0007 & config;
    _U1TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U1RXIE = (0x0008 & config) >> 3;
    _U1TXIE = (0x0080 & config) >> 7;
}
// *****************************************************************************
/* Function:
    DataRdyUART1()

  Summary:
    Function to check whether data is ready and it can be read

  Description:
    This function checks whether there is any data that can be read from the
    input buffer, by checking URXDA bit

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    char if any data available in buffer.

  Remarks:
    None.
 */
char DataRdyUART1(void)
{
    return (U1STAbits.URXDA);
}
// *****************************************************************************
/* Function:
    getsUART1()

  Summary:
    Function to read data from recieve buffer if available

  Description:
    This function gets a string of data of specified length if available in the
    UxRXREG buffer into the buffer specified

  Precondition:
    None.

  Parameters:
    unsigned int length the length expected
    unsigned int *buffer  the received data to be recorded to this array
    unsigned int uart_data_wait timeout value

  Returns:
    unsigned int number of data bytes yet to be received.

  Remarks:
    None.
 */
uint16_t getsUART1(uint16_t length, uint16_t *buffer,uint16_t uart_data_wait)
{
    int16_t wait = 0;
    char *temp_ptr = (char *) buffer;

    while (length) /* read till length is 0 */
    {
        while (!DataRdyUART1())
        {
            /*wait for more data */
            if (wait < uart_data_wait)
            {
                wait++;
            }
            /*Time out- Return words/bytes to be read */
            else
            {
                return (length);
            }
        }
        wait = 0;
        /* check if TX/RX is 8bits or 9bits */
        if (U1MODEbits.PDSEL == 3)
        {
            /* data word from HW buffer to SW buffer */
            *buffer++ = U1RXREG;
        }
        else
        {
            /* data byte from HW buffer to SW buffer */
            *temp_ptr++ = U1RXREG & 0xFF;
        }

        length--;
    }
    /* number of data yet to be received i.e.,0 */
    return (length);
}

// *****************************************************************************
/* Function:
    OpenUART1()

  Summary:
    Function to configure UART module

  Description:
    This function configures the UART mode,UART Interrupt modes and the Baud
    Rate

  Precondition:
    None.

  Parameters:
    unsigned int config1 operation setting
    unsigned int config2 TX & RX interrupt modes
    unsigned int ubrg baud rate setting

  Returns:
    None.

  Remarks:
    None.
 */
void OpenUART1(uint16_t config1, uint16_t config2, uint16_t ubrg)
{
    /* baud rate */
    U1BRG = ubrg;
    /* operation settings */
    U1MODE = config1;
    /* TX & RX interrupt modes */
    U1STA = config2;
}

// *****************************************************************************
/* Function:
    putsUART1()

  Summary:
    Function to put data to be tranmitted into transmit Buffer

  Description:
    This function puts the data string to be transmitted into the transmit
    buffer (till NULL character)

  Precondition:
    None.

  Parameters:
    unsigned int * address of the string buffer to be transmitted

  Returns:
    None.

  Remarks:
    None.
 */
void putsUART1(uint16_t *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */
    /* check if TX is 8bits or 9bits */
    if (U1MODEbits.PDSEL == 3)
    {
        while (*buffer != '\0')
        {
             /* wait if the buffer is full */
            while (U1STAbits.UTXBF);
            /* transfer data word to TX reg */
            U1TXREG = *buffer++;
        }
    }
    else
    {
        while (*temp_ptr != '\0')
        {
            /* wait if the buffer is full */
            while (U1STAbits.UTXBF);
            /* transfer data byte to TX reg */
            U1TXREG = *temp_ptr++;
        }
    }
}
// *****************************************************************************
/* Function:
    ReadUART1()

  Summary:
    This function returns the contents of UxRXREG buffer

  Description:
    This function returns the contents of UxRXREG buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    unsigned int value from UxRXREG receive buffer.

  Remarks:
    None.
 */
uint16_t ReadUART1(void)
{
    if (U1MODEbits.PDSEL == 3)
    {
        return (U1RXREG);
    }
    else
    {
        return (U1RXREG & 0xFF);
    }
}
// *****************************************************************************
/* Function:
    WriteUART1()

  Summary:
    This function writes data into the UxTXREG

  Description:
    This function writes data into the UxTXREG

  Precondition:
    None.

  Parameters:
    unsigned int data the data to be written.

  Returns:
    None.

  Remarks:
    None.
 */
void WriteUART1(uint16_t data)
{
    if (U1MODEbits.PDSEL == 3)
    {
        U1TXREG = data;
    }
    else
    {
        U1TXREG = data & 0xFF;
    }
}

#endif
#ifdef _U2RXIF

// *****************************************************************************
/* Function:
    BusyUART2()

  Summary:
    Routine to check if the transmission is progress

  Description:
    This returns status whether the transmission is in progress or not,
    by checking Status bit TRMT

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Info about whether transmission is in progress.

  Remarks:
    None.
 */
char BusyUART2(void)
{
    return (!U2STAbits.TRMT);
}
// *****************************************************************************
/* Function:
    CloseUART2()

  Summary:
    Routine to disable the UART module

  Description:
    This function disables the UART and clears the interrupt enable and flag
    bits

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void CloseUART2(void)
{
    U2MODEbits.UARTEN = 0;

    _U2RXIE = 0;
    _U2TXIE = 0;

    _U2RXIF = 0;
    _U2TXIF = 0;
}
// *****************************************************************************
/* Function:
    ConfigIntUART2()

  Summary:
    Function to configure UART Rx and Tx Interrupts

  Description:
    This function sets priority for  RX and TX interrupt and enable/disables
    the interrupt

  Precondition:
    None.

  Parameters:
    unsigned int config enable/disable and priority.

  Returns:
    None.

  Remarks:
    None.
 */
void ConfigIntUART2(uint16_t config)
{
    /* clear IF flags */
    _U2RXIF = 0;
    _U2TXIF = 0;

    /* set priority */
    _U2RXIP = 0x0007 & config;
    _U2TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U2RXIE = (0x0008 & config) >> 3;
    _U2TXIE = (0x0080 & config) >> 7;
}
// *****************************************************************************
/* Function:
    DataRdyUART2()

  Summary:
    Function to check whether data is ready and it can be read

  Description:
    This function checks whether there is any data that can be read from the
    input buffer, by checking URXDA bit

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    char if any data available in buffer.

  Remarks:
    None.
 */
char DataRdyUART2(void)
{
    return (U2STAbits.URXDA);
}
// *****************************************************************************
/* Function:
    getsUART2()

  Summary:
    Function to read data from recieve buffer if available

  Description:
    This function gets a string of data of specified length if available in the
    UxRXREG buffer into the buffer specified

  Precondition:
    None.

  Parameters:
    unsigned int length the length expected
    unsigned int *buffer  the received data to be recorded to this array
    unsigned int uart_data_wait timeout value

  Returns:
    unsigned int number of data bytes yet to be received.

  Remarks:
    None.
 */
uint16_t getsUART2(uint16_t length, uint16_t *buffer,uint16_t uart_data_wait)
{
    int16_t wait = 0;
    char *temp_ptr = (char *) buffer;

    while (length) /* read till length is 0 */
    {
        while (!DataRdyUART2())
        {
            /*wait for more data */
            if (wait < uart_data_wait)
            {
                wait++;
            }
            /*Time out- Return words/bytes to be read */
            else
            {
                return (length);
            }
        }
        wait = 0;
        /* check if TX/RX is 8bits or 9bits */
        if (U2MODEbits.PDSEL == 3)
        {
            /* data word from HW buffer to SW buffer */
            *buffer++ = U2RXREG;
        }
        else
        {
            /* data byte from HW buffer to SW buffer */
            *temp_ptr++ = U2RXREG & 0xFF;
        }

        length--;
    }
    /* number of data yet to be received i.e.,0 */
    return (length);
}

// *****************************************************************************
/* Function:
    OpenUART2()

  Summary:
    Function to configure UART module

  Description:
    This function configures the UART mode,UART Interrupt modes and the Baud
    Rate

  Precondition:
    None.

  Parameters:
    unsigned int config1 operation setting
    unsigned int config2 TX & RX interrupt modes
    unsigned int ubrg baud rate setting

  Returns:
    None.

  Remarks:
    None.
 */
void OpenUART2(uint16_t config1, uint16_t config2, uint16_t ubrg)
{
    /* baud rate */
    U2BRG = ubrg;
    /* operation settings */
    U2MODE = config1;
    /* TX & RX interrupt modes */
    U2STA = config2;
}

// *****************************************************************************
/* Function:
    putsUART2()

  Summary:
    Function to put data to be tranmitted into transmit Buffer

  Description:
    This function puts the data string to be transmitted into the transmit
    buffer (till NULL character)

  Precondition:
    None.

  Parameters:
    unsigned int * address of the string buffer to be transmitted

  Returns:
    None.

  Remarks:
    None.
 */
void putsUART2(uint16_t *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */
    /* check if TX is 8bits or 9bits */
    if (U2MODEbits.PDSEL == 3)
    {
        while (*buffer != '\0')
        {
             /* wait if the buffer is full */
            while (U2STAbits.UTXBF);
            /* transfer data word to TX reg */
            U2TXREG = *buffer++;
        }
    }
    else
    {
        while (*temp_ptr != '\0')
        {
            /* wait if the buffer is full */
            while (U2STAbits.UTXBF);
            /* transfer data byte to TX reg */
            U2TXREG = *temp_ptr++;
        }
    }
}
// *****************************************************************************
/* Function:
    ReadUART2()

  Summary:
    This function returns the contents of UxRXREG buffer

  Description:
    This function returns the contents of UxRXREG buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    unsigned int value from UxRXREG receive buffer.

  Remarks:
    None.
 */
uint16_t ReadUART2(void)
{
    if (U2MODEbits.PDSEL == 3)
    {
        return (U2RXREG);
    }
    else
    {
        return (U2RXREG & 0xFF);
    }
}
// *****************************************************************************
/* Function:
    WriteUART2()

  Summary:
    This function writes data into the UxTXREG

  Description:
    This function writes data into the UxTXREG

  Precondition:
    None.

  Parameters:
    unsigned int data the data to be written.

  Returns:
    None.

  Remarks:
    None.
 */
void WriteUART2(uint16_t data)
{
    if (U2MODEbits.PDSEL == 3)
    {
        U2TXREG = data;
    }
    else
    {
        U2TXREG = data & 0xFF;
    }
}

#endif
#ifdef _U3RXIF

// *****************************************************************************
/* Function:
    BusyUART3()

  Summary:
    Routine to check if the transmission is progress

  Description:
    This returns status whether the transmission is in progress or not,
    by checking Status bit TRMT

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Info about whether transmission is in progress.

  Remarks:
    None.
 */
char BusyUART3(void)
{
    return (!U3STAbits.TRMT);
}
// *****************************************************************************
/* Function:
    CloseUART3()

  Summary:
    Routine to disable the UART module

  Description:
    This function disables the UART and clears the interrupt enable and flag
    bits

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void CloseUART3(void)
{
    U3MODEbits.UARTEN = 0;

    _U3RXIE = 0;
    _U3TXIE = 0;

    _U3RXIF = 0;
    _U3TXIF = 0;
}
// *****************************************************************************
/* Function:
    ConfigIntUART3()

  Summary:
    Function to configure UART Rx and Tx Interrupts

  Description:
    This function sets priority for  RX and TX interrupt and enable/disables
    the interrupt

  Precondition:
    None.

  Parameters:
    unsigned int config enable/disable and priority.

  Returns:
    None.

  Remarks:
    None.
 */
void ConfigIntUART3(uint16_t config)
{
    /* clear IF flags */
    _U3RXIF = 0;
    _U3TXIF = 0;

    /* set priority */
    _U3RXIP = 0x0007 & config;
    _U3TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U3RXIE = (0x0008 & config) >> 3;
    _U3TXIE = (0x0080 & config) >> 7;
}
// *****************************************************************************
/* Function:
    DataRdyUART3()

  Summary:
    Function to check whether data is ready and it can be read

  Description:
    This function checks whether there is any data that can be read from the
    input buffer, by checking URXDA bit

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    char if any data available in buffer.

  Remarks:
    None.
 */
char DataRdyUART3(void)
{
    return (U3STAbits.URXDA);
}
// *****************************************************************************
/* Function:
    getsUART3()

  Summary:
    Function to read data from recieve buffer if available

  Description:
    This function gets a string of data of specified length if available in the
    UxRXREG buffer into the buffer specified

  Precondition:
    None.

  Parameters:
    unsigned int length the length expected
    unsigned int *buffer  the received data to be recorded to this array
    unsigned int uart_data_wait timeout value

  Returns:
    unsigned int number of data bytes yet to be received.

  Remarks:
    None.
 */
uint16_t getsUART3(uint16_t length, uint16_t *buffer,uint16_t uart_data_wait)
{
    int16_t wait = 0;
    char *temp_ptr = (char *) buffer;

    while (length) /* read till length is 0 */
    {
        while (!DataRdyUART3())
        {
            /*wait for more data */
            if (wait < uart_data_wait)
            {
                wait++;
            }
            /*Time out- Return words/bytes to be read */
            else
            {
                return (length);
            }
        }
        wait = 0;
        /* check if TX/RX is 8bits or 9bits */
        if (U3MODEbits.PDSEL == 3)
        {
            /* data word from HW buffer to SW buffer */
            *buffer++ = U3RXREG;
        }
        else
        {
            /* data byte from HW buffer to SW buffer */
            *temp_ptr++ = U3RXREG & 0xFF;
        }

        length--;
    }
    /* number of data yet to be received i.e.,0 */
    return (length);
}

// *****************************************************************************
/* Function:
    OpenUART3()

  Summary:
    Function to configure UART module

  Description:
    This function configures the UART mode,UART Interrupt modes and the Baud
    Rate

  Precondition:
    None.

  Parameters:
    unsigned int config1 operation setting
    unsigned int config2 TX & RX interrupt modes
    unsigned int ubrg baud rate setting

  Returns:
    None.

  Remarks:
    None.
 */
void OpenUART3(uint16_t config1, uint16_t config2, uint16_t ubrg)
{
    /* baud rate */
    U3BRG = ubrg;
    /* operation settings */
    U3MODE = config1;
    /* TX & RX interrupt modes */
    U3STA = config2;
}

// *****************************************************************************
/* Function:
    putsUART3()

  Summary:
    Function to put data to be tranmitted into transmit Buffer

  Description:
    This function puts the data string to be transmitted into the transmit
    buffer (till NULL character)

  Precondition:
    None.

  Parameters:
    unsigned int * address of the string buffer to be transmitted

  Returns:
    None.

  Remarks:
    None.
 */
void putsUART3(uint16_t *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */
    /* check if TX is 8bits or 9bits */
    if (U3MODEbits.PDSEL == 3)
    {
        while (*buffer != '\0')
        {
             /* wait if the buffer is full */
            while (U3STAbits.UTXBF);
            /* transfer data word to TX reg */
            U3TXREG = *buffer++;
        }
    }
    else
    {
        while (*temp_ptr != '\0')
        {
            /* wait if the buffer is full */
            while (U3STAbits.UTXBF);
            /* transfer data byte to TX reg */
            U3TXREG = *temp_ptr++;
        }
    }
}
// *****************************************************************************
/* Function:
    ReadUART3()

  Summary:
    This function returns the contents of UxRXREG buffer

  Description:
    This function returns the contents of UxRXREG buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    unsigned int value from UxRXREG receive buffer.

  Remarks:
    None.
 */
uint16_t ReadUART3(void)
{
    if (U3MODEbits.PDSEL == 3)
    {
        return (U3RXREG);
    }
    else
    {
        return (U3RXREG & 0xFF);
    }
}
// *****************************************************************************
/* Function:
    WriteUART3()

  Summary:
    This function writes data into the UxTXREG

  Description:
    This function writes data into the UxTXREG

  Precondition:
    None.

  Parameters:
    unsigned int data the data to be written.

  Returns:
    None.

  Remarks:
    None.
 */
void WriteUART3(uint16_t data)
{
    if (U3MODEbits.PDSEL == 3)
    {
        U3TXREG = data;
    }
    else
    {
        U3TXREG = data & 0xFF;
    }
}

#endif
#ifdef _U4RXIF

// *****************************************************************************
/* Function:
    BusyUART4()

  Summary:
    Routine to check if the transmission is progress

  Description:
    This returns status whether the transmission is in progress or not,
    by checking Status bit TRMT

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Info about whether transmission is in progress.

  Remarks:
    None.
 */
char BusyUART4(void)
{
    return (!U4STAbits.TRMT);
}
// *****************************************************************************
/* Function:
    CloseUART4()

  Summary:
    Routine to disable the UART module

  Description:
    This function disables the UART and clears the interrupt enable and flag
    bits

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 */
void CloseUART4(void)
{
    U4MODEbits.UARTEN = 0;

    _U4RXIE = 0;
    _U4TXIE = 0;

    _U4RXIF = 0;
    _U4TXIF = 0;
}
// *****************************************************************************
/* Function:
    ConfigIntUART4()

  Summary:
    Function to configure UART Rx and Tx Interrupts

  Description:
    This function sets priority for  RX and TX interrupt and enable/disables
    the interrupt

  Precondition:
    None.

  Parameters:
    unsigned int config enable/disable and priority.

  Returns:
    None.

  Remarks:
    None.
 */
void ConfigIntUART4(uint16_t config)
{
    /* clear IF flags */
    _U4RXIF = 0;
    _U4TXIF = 0;

    /* set priority */
    _U4RXIP = 0x0007 & config;
    _U4TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U4RXIE = (0x0008 & config) >> 3;
    _U4TXIE = (0x0080 & config) >> 7;
}
// *****************************************************************************
/* Function:
    DataRdyUART4()

  Summary:
    Function to check whether data is ready and it can be read

  Description:
    This function checks whether there is any data that can be read from the
    input buffer, by checking URXDA bit

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    char if any data available in buffer.

  Remarks:
    None.
 */
char DataRdyUART4(void)
{
    return (U4STAbits.URXDA);
}
// *****************************************************************************
/* Function:
    getsUART4()

  Summary:
    Function to read data from recieve buffer if available

  Description:
    This function gets a string of data of specified length if available in the
    UxRXREG buffer into the buffer specified

  Precondition:
    None.

  Parameters:
    unsigned int length the length expected
    unsigned int *buffer  the received data to be recorded to this array
    unsigned int uart_data_wait timeout value

  Returns:
    unsigned int number of data bytes yet to be received.

  Remarks:
    None.
 */
uint16_t getsUART4(uint16_t length, uint16_t *buffer,uint16_t uart_data_wait)
{
    int16_t wait = 0;
    char *temp_ptr = (char *) buffer;

    while (length) /* read till length is 0 */
    {
        while (!DataRdyUART4())
        {
            /*wait for more data */
            if (wait < uart_data_wait)
            {
                wait++;
            }
            /*Time out- Return words/bytes to be read */
            else
            {
                return (length);
            }
        }
        wait = 0;
        /* check if TX/RX is 8bits or 9bits */
        if (U4MODEbits.PDSEL == 3)
        {
            /* data word from HW buffer to SW buffer */
            *buffer++ = U4RXREG;
        }
        else
        {
            /* data byte from HW buffer to SW buffer */
            *temp_ptr++ = U4RXREG & 0xFF;
        }

        length--;
    }
    /* number of data yet to be received i.e.,0 */
    return (length);
}

// *****************************************************************************
/* Function:
    OpenUART4()

  Summary:
    Function to configure UART module

  Description:
    This function configures the UART mode,UART Interrupt modes and the Baud
    Rate

  Precondition:
    None.

  Parameters:
    unsigned int config1 operation setting
    unsigned int config2 TX & RX interrupt modes
    unsigned int ubrg baud rate setting

  Returns:
    None.

  Remarks:
    None.
 */
void OpenUART4(uint16_t config1, uint16_t config2, uint16_t ubrg)
{
    /* baud rate */
    U4BRG = ubrg;
    /* operation settings */
    U4MODE = config1;
    /* TX & RX interrupt modes */
    U4STA = config2;
}

// *****************************************************************************
/* Function:
    putsUART4()

  Summary:
    Function to put data to be tranmitted into transmit Buffer

  Description:
    This function puts the data string to be transmitted into the transmit
    buffer (till NULL character)

  Precondition:
    None.

  Parameters:
    unsigned int * address of the string buffer to be transmitted

  Returns:
    None.

  Remarks:
    None.
 */
void putsUART4(uint16_t *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */
    /* check if TX is 8bits or 9bits */
    if (U4MODEbits.PDSEL == 3)
    {
        while (*buffer != '\0')
        {
             /* wait if the buffer is full */
            while (U4STAbits.UTXBF);
            /* transfer data word to TX reg */
            U4TXREG = *buffer++;
        }
    } 
    else
    {
        while (*temp_ptr != '\0')
        {
            /* wait if the buffer is full */
            while (U4STAbits.UTXBF);
            /* transfer data byte to TX reg */
            U4TXREG = *temp_ptr++;
        }
    }
}
// *****************************************************************************
/* Function:
    ReadUART4()

  Summary:
    This function returns the contents of UxRXREG buffer

  Description:
    This function returns the contents of UxRXREG buffer

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    unsigned int value from UxRXREG receive buffer.

  Remarks:
    None.
 */
uint16_t ReadUART4(void)
{
    if (U4MODEbits.PDSEL == 3)
    {
        return (U4RXREG);
    }
    else
    {
        return (U4RXREG & 0xFF);
    }
}
// *****************************************************************************
/* Function:
    WriteUART4()

  Summary:
    This function writes data into the UxTXREG

  Description:
    This function writes data into the UxTXREG

  Precondition:
    None.

  Parameters:
    unsigned int data the data to be written.

  Returns:
    None.

  Remarks:
    None.
 */
void WriteUART4(uint16_t data)
{
    if (U4MODEbits.PDSEL == 3)
    {
        U4TXREG = data;
    }
    else
    {
        U4TXREG = data & 0xFF;
    }
}

#endif


/* EOF */
 /******************************************************************************
 * UART_COM_PORT.c
 *
 *  Created on: 21 d’abr. 2022
 *      Author: ainar
 *
 * MSP432 UART - PC with 24MHz SMCLK
 *
 * Description: This demo receives and transmits data via a PC serial port.
 * The auto-clock enable feature is used by the eUSCI and SMCLK is turned off
 * when the UART is idle and turned on when a receive edge is detected.
 * Note that level shifter hardware is needed to shift between RS232 and MSP
 * voltage levels.
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *            |                 |
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----> PC
 *            |                 |
 *            |                 |
 *            |     P1.2/UCA0RXD|<---- PC
 *            |                 |
 *
 * Author: Timothy Logan
 *******************************************************************************/

/* DriverLib Includes */
#include <driverlib.h>

/* Standard Includes */
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "UART_COM_PORT.h"


/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate.
 *
 * For baud rate 9600 and SMCLK = 24000000, set BRDIV=156, UCxBRF=4, UCxBRS=0 <-- our case
 * For baud rate 115200 and SMCLK = 24000000, set BRDIV=13, UCxBRF=0, UCxBRS=37
 * For baud rate 230400 and SMCLK = 24000000, set BRDIV=6, UCxBRF=8, UCxBRS=32
 * For baud rate 460800 and SMCLK = 24000000, set BRDIV=3, UCxBRF=4, UCxBRS=2
 * For baud rate 921600 and SMCLK = 24000000, set BRDIV=1, UCxBRF=10, UCxBRS=0
 */

const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                     // SMCLK Clock Source
    156,                                                // BRDIV = 78
    4,                                                  // UCxBRF = 2
    0,                                                  // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                             // No Parity
    EUSCI_A_UART_MSB_FIRST,                             // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,                          // One stop bit
    EUSCI_A_UART_MODE,                                  // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION       // Oversampling
};

void init_uart(void)
{
    /* initialize UART */
    /* Selecting P1.2 and P1.3 in UART mode*/
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               UART_RX | UART_TX, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    UART_enableModule(EUSCI_A0_BASE);
}


 /******************************************************************************
 * UART_COM_PORT.c
 *
 *  Created on: 21 d’abr. 2022
 *      Author: ainar
 *
 * MSP432 UART - PC echo
 *
 * Description: This demo echoes back characters received via a PC serial port.
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
 *       RST -|     P1.3/UCA0TXD|----> PC (echo)
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
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *software-dl.ti.com/.../index.html
 */
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                     // SMCLK Clock Source
    78,                                                 // BRDIV = 78
    2,                                                  // UCxBRF = 2
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


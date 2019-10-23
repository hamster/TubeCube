/**
 * \file
 *
 * \brief USART basic driver.
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \defgroup doc_driver_usart_basic USART Basic
 * \ingroup doc_driver_usart
 *
 * \section doc_driver_usart_basic_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <compiler.h>
#include <clock_config.h>
#include <usart_basic.h>
#include <atomic.h>

/**
 * \brief Initialize USART interface
 * If module is configured to disabled state, the clock to the USART is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the USART init was successful
 * \retval 1 the USART init was not successful
 */
int8_t USART_0_init()
{

	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600); /* set baud rate register */

	// USART0.CTRLA = 0 << USART_ABEIE_bp /* Auto-baud Error Interrupt Enable: disabled */
	//		 | 0 << USART_DREIE_bp /* Data Register Empty Interrupt Enable: disabled */
	//		 | 0 << USART_LBME_bp /* Loop-back Mode Enable: disabled */
	//		 | USART_RS485_OFF_gc /* RS485 Mode disabled */
	//		 | 0 << USART_RXCIE_bp /* Receive Complete Interrupt Enable: disabled */
	//		 | 0 << USART_RXSIE_bp /* Receiver Start Frame Interrupt Enable: disabled */
	//		 | 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

	USART0.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
	               | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
	               | 1 << USART_RXEN_bp     /* Reciever enable: enabled */
	               | USART_RXMODE_NORMAL_gc /* Normal mode */
	               | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
	               | 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

	// USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
	//		 | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
	//		 | USART_PMODE_DISABLED_gc /* No Parity */
	//		 | USART_SBMODE_1BIT_gc; /* 1 stop bit */

	// USART0.DBGCTRL = 0 << USART_DBGRUN_bp; /* Debug Run: disabled */

	// USART0.EVCTRL = 0 << USART_IREI_bp; /* IrDA Event Input Enable: disabled */

	// USART0.RXPLCTRL = 0x0 << USART_RXPL_gp; /* Receiver Pulse Length: 0x0 */

	// USART0.TXPLCTRL = 0x0 << USART_TXPL_gp; /* Transmit pulse length: 0x0 */

	return 0;
}

/**
 * \brief Enable RX and TX in USART_0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX and TX enable-bits in the USART control register
 *
 * \return Nothing
 */
void USART_0_enable()
{
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * \brief Enable RX in USART_0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the RX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART_0_enable_rx()
{
	USART0.CTRLB |= USART_RXEN_bm;
}

/**
 * \brief Enable TX in USART_0
 * 1. If supported by the clock system, enables the clock to the USART
 * 2. Enables the USART module by setting the TX enable-bit in the USART control register
 *
 * \return Nothing
 */
void USART_0_enable_tx()
{
	USART0.CTRLB |= USART_TXEN_bm;
}

/**
 * \brief Disable USART_0
 * 1. Disables the USART module by clearing the enable-bit(s) in the USART control register
 * 2. If supported by the clock system, disables the clock to the USART
 *
 * \return Nothing
 */
void USART_0_disable()
{
	USART0.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * \brief Get recieved data from USART_0
 *
 * \return Data register from USART_0 module
 */
uint8_t USART_0_get_data()
{
	return USART0.RXDATAL;
}

/**
 * \brief Check if the usart can accept data to be transmitted
 *
 * \return The status of USART TX data ready check
 * \retval false The USART can not receive data to be transmitted
 * \retval true The USART can receive data to be transmitted
 */
bool USART_0_is_tx_ready()
{
	return (USART0.STATUS & USART_DREIF_bm);
}

/**
 * \brief Check if the USART has received data
 *
 * \return The status of USART RX data ready check
 * \retval true The USART has received data
 * \retval false The USART has not received data
 */
bool USART_0_is_rx_ready()
{
	return (USART0.STATUS & USART_RXCIF_bm);
}

/**
 * \brief Check if USART_0 data is transmitted
 *
 * \return Receiver ready status
 * \retval true  Data is not completely shifted out of the shift register
 * \retval false Data completely shifted out if the USART shift register
 */
bool USART_0_is_tx_busy()
{
	return (!(USART0.STATUS & USART_TXCIF_bm));
}

/**
 * \brief Read one character from USART_0
 *
 * Function will block if a character is not available.
 *
 * \return Data read from the USART_0 module
 */
uint8_t USART_0_read()
{
	while (!(USART0.STATUS & USART_RXCIF_bm))
		;
	return USART0.RXDATAL;
}

/**
 * \brief Write one character to USART_0
 *
 * Function will block until a character can be accepted.
 *
 * \param[in] data The character to write to the USART
 *
 * \return Nothing
 */
void USART_0_write(const uint8_t data)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
		;
	USART0.TXDATAL = data;
}

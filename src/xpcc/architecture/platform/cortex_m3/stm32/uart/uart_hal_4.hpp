// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2011, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Roboterclub Aachen e.V. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------
/*
 * WARNING: This file is generated automatically, do not edit!
 * Please modify the corresponding *.in file instead and rebuild this file. 
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_STM32__UARTHAL_4_HPP
#define XPCC_STM32__UARTHAL_4_HPP

#include <stdint.h>
#include "uart_base.hpp"

namespace xpcc
{
	namespace stm32
	{
		/**
		 * @brief		Universal asynchronous receiver transmitter (UARTHAL4)
		 * 
		 * Not available on the low- and medium density devices.
		 * 
		 * Very basic implementation that exposes more hardware features than
		 * the regular Usart classes.
		 * 
		 * @ingroup		stm32
		 */
		class UartHal4 : public UartBase
		{
		public:
			enum Interrupt
			{
				INTERRUPT_CHARACTER_MATCH = USART_CR1_CMIE,
				// called when the transmit register is empty (i.e. the byte
				// has been transfered to the shift out register)
				INTERRUPT_TX_EMPTY		= USART_CR1_TXEIE,
				// called when the byte was actually transmitted
				INTERRUPT_TX_COMPLETE	= USART_CR1_TCIE,
				INTERRUPT_RX_NOT_EMPTY	= USART_CR1_RXNEIE,
				INTERRUPT_PARITY_ERROR	= USART_CR1_PEIE,
			};

			enum InterruptFlag
			{
				FLAG_CHARACTER_MATCH	= USART_ISR_CMF,
				FLAG_TX_EMPTY			= USART_ISR_TXE,
				FLAG_TX_COMPLETE		= USART_ISR_TC,
				FLAG_RX_NOT_EMPTY		= USART_ISR_RXNE,
				FLAG_PARITY_ERROR		= USART_ISR_PE,
			};

			enum class Parity : uint32_t
			{
				Disabled 	= 0,
				Even 		= USART_CR1_PCE,
				Odd  		= USART_CR1_PCE | USART_CR1_PS,
			};

			enum ErrorFlag
			{
				ERROR_OVERRUN = USART_ISR_ORE,
				// set if a de-synchronization,
				// excessive noise or a break character is detected
				ERROR_FRAMING = USART_ISR_FE,
				ERROR_PARITY = USART_ISR_PE,
			};

		public:

			/*
			 * Enables the clock, resets the hardware and sets the UE bit
			 */
			static void
			enable();

			/*
			 * Disables the hw module (by disabling its clock line)
			 */
			static void
			disable();

			/*
			 * Initiaize Uart Hall Peripheral
			 *
			 * Enables clocks, the UART peripheral (but neither TX nor RX)
			 * Sets baudrate and parity.
			 */
			static void
			initialize(uint32_t baudrate, Parity parity = Parity::Disabled)
			{
				enable();
				// DIRTY HACK: disable and reenable uart to be able to set
				//             baud rate as well as parity
				UART4->CR1 &= ~USART_CR1_UE;	// Uart Disable
				setBaudrate(baudrate);
				setParity(parity);
				UART4->CR1 |=  USART_CR1_UE;	// Uart Reenable
			}

			enum Mapping
			{
				REMAP_PC10_PC11,	///< TX mapped to PC10, RX mapped to PC11
#if defined(STM32F2XX) || defined(STM32F4XX)
				REMAP_PA0_PA1,		///< TX mapped to PA0, RX mapped to PA1
#endif
			};

			/**
			 * Configure the IO Pins for UartHal4
			 */
			static void
			configurePins(Mapping mapping);

		private:	// This methods can only be executed while UART is
					// disabled.
					// At the moment this is only the case in the initialize
					// method.
			/*
			 * Set Baudrate
			 *
			 * Remember to enable the periheral before setting the baudrate
			 */
			static void
			setBaudrate(uint32_t baudrate);

			/*
			 * Disable Parity or Enable Odd/Even Parity
			 *
			 * This method assumes 8 databit + 1 parity bit
			 */
			static inline void
			setParity(Parity parity)
			{
				uint32_t flags = UART4->CR1;
				flags &= ~(USART_CR1_PCE | USART_CR1_PS);
				flags |= static_cast<uint32_t>(parity);
				// Parity Bit counts as 9th bit -> enable 9 data bits
				flags |= USART_CR1_M;
				UART4->CR1 = flags;
			}

		public:

			/*
			 * \brief	Write a single byte to the transmit register
			 *
			 * WARNING: this method does NOT do any sanity checks!!
			 * It is your responsibility to check if the register
			 * is empty!
			 */
			static inline void
			write(uint8_t data)
			{
			#if defined(STM32F3XX)
				UART4->TDR = data;
			#else
				UART4->DR = data;
			#endif
			}

			/**
			 * \brief	Returns the value of the receive register
			 *
			 * WARNING: this method does NOT do any sanity checks!!
			 * It is your responsibility to check if the register
			 * contains something useful!
			 */
			static inline uint8_t
			read()
			{
			#if defined(STM32F3XX)
				return UART4->RDR;
			#else
				return UART4->DR;
			#endif
			}



			/*
			 * Enable/Disable Transmitter
			 */
			static inline void
			setTransmitterEnable(bool enable)
			{
				if(enable)
					UART4->CR1 |=  USART_CR1_TE;
				else
					UART4->CR1 &= ~USART_CR1_TE;
			}

			/*
			 * Enable/Disable Receiver
			 */
			static inline void
			setReceiverEnable(bool enable)
			{
				if(enable)
					UART4->CR1 |=  USART_CR1_RE;
				else
					UART4->CR1 &= ~USART_CR1_RE;
			}

			/*
			 * Returns true if data has been received
			 */
			static inline bool
			isReceiveRegisterNotEmpty()
			{
			#if defined(STM32F3XX)
				return UART4->ISR & USART_ISR_RXNE;
			#else
				return UART4->SR & USART_SR_RXNE;
			#endif
			}

			/*
			 * Returns true if data can be written
			 */
			static inline bool
			isTransmitRegisterEmpty()
			{
			#if defined(STM32F3XX)
				return UART4->ISR & USART_ISR_TXE;
			#else
				return UART4->SR & USART_SR_TXE;
			#endif
			}

			static void
			enableInterruptVector(bool enable, uint32_t priority);

			static inline void
			enableInterrupt(Interrupt interrupt)
			{
				UART4->CR1 |= interrupt;
			}

			static inline void
			disableInterrupt(Interrupt interrupt)
			{
				UART4->CR1 &= ~interrupt;
			}

			static inline InterruptFlag
			getInterruptFlags()
			{
				return (InterruptFlag) UART4->ISR;
			}

			static inline ErrorFlag
			getErrorFlags()
			{
				return (ErrorFlag) UART4->ISR;
			}

			static inline void
			resetInterruptFlags(InterruptFlag flags)
			{
				// Not all flags can be cleared by writing to this reg
				const uint32_t mask = USART_ICR_PECF  | USART_ICR_FECF   |
					USART_ICR_NCF   | USART_ICR_ORECF | USART_ICR_IDLECF |
					USART_ICR_TCCF  | USART_ICR_LBDCF | USART_ICR_CTSCF  |
					USART_ICR_RTOCF | USART_ICR_EOBCF | USART_ICR_CMCF   |
					USART_ICR_WUCF;
				// Flags are cleared by writing a one to the flag position.
				// Writing a zero is (hopefully) ignored.
				UART4->ICR = (flags & mask);
			}
		};
	}
}

#endif // XPCC_STM32__UARTHAL_4_HPP
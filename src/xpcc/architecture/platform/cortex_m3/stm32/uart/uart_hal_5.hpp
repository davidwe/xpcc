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

#ifndef XPCC_STM32__UARTHAL_5_HPP
#define XPCC_STM32__UARTHAL_5_HPP

#include <stdint.h>
#include "uart_base.hpp"

namespace xpcc
{
	namespace stm32
	{
		/**
		 * @brief		Universal asynchronous receiver transmitter (UARTHAL5)
		 * 
		 * Not available on the low- and medium density devices.
		 * 
		 * Very badic implementation that exposes more hardware features than
		 * the regular Usart classes.
		 * 
		 * @ingroup		stm32
		 */
		class UartHal5 : public UartBase
		{

			enum Interrupt
			{
				INTERRUPT_CHARACTER_MATCH = USART_CR1_CMIE,
				// called when the transmit register is empty (i.e. the byte
				// has been tranfered to the shift out register)
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

			enum Parity
			{
				PARITY_DISABLED = 0,
				PARITY_EVEN = USART_CR1_PCE,
				PARITY_ODD  = USART_CR1_PCE | USART_CR1_PS,
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
			UartHal5(uint32_t baudrate)
			{
				setBaudrate(baudrate);
			}

			enum Mapping
			{
				REMAP_PC12_PD2,		///< TX mapped to PC12, RX mapped to PD2
			};

			/**
			 * Configure the IO Pins for UartHal5
			 */
			static void
			configurePins(Mapping mapping);

			/**
			 * \brief	Set baudrate
			 * \param	baudrate	desired baud rate
			 */
			static void
			setBaudrate(uint32_t baudrate);

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
				UART5->TDR = data;
			#else
				UART5->DR = data;
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
				return UART5->RDR;
			#else
				return UART5->DR;
			#endif
			}

			/*
			 * Disable Parity or Enable Odd/Even Parity
			 */
			static inline void
			setParity(Parity parity)
			{
				uint32_t flags = UART5->CR1;
				flags &= ~(USART_CR1_PCE | USART_CR1_PS);
				flags |= parity;
				UART5->CR1 = flags;
			}

			/*
			 * Enable/Disable Transmitter
			 */
			static inline void
			setTransmitterEnable(bool enable)
			{
				if(enable)
					UART5->CR1 |=  USART_CR1_TE;
				else
					UART5->CR1 &= ~USART_CR1_TE;
			}

			/*
			 * Enable/Disable Receiver
			 */
			static inline void
			setReceiverEnable(bool enable)
			{
				if(enable)
					UART5->CR1 |=  USART_CR1_RE;
				else
					UART5->CR1 &= ~USART_CR1_RE;
			}

			/*
			 * Returns true if data has been received
			 */
			static inline bool
			isReceiveRegisterNotEmpty()
			{
			#if defined(STM32F3XX)
				return UART5->ISR & USART_ISR_RXNE;
			#else
				return UART5->SR & USART_SR_RXNE;
			#endif
			}

			/*
			 * Returns true if data can be written
			 */
			static inline bool
			isTrasmitRegisterEmpty()
			{
			#if defined(STM32F3XX)
				return UART5->ISR & USART_ISR_TXE;
			#else
				return UART5->SR & USART_SR_TXE;
			#endif
			}

			static void
			enableInterruptVector(bool enable, uint32_t priority);

			static inline void
			enableInterrupt(Interrupt interrupt)
			{
				UART5->CR1 |= interrupt;
			}

			static inline void
			disableInterrupt(Interrupt interrupt)
			{
				UART5->CR1 &= ~interrupt;
			}

			static inline InterruptFlag
			getInterruptFlags()
			{
				return (InterruptFlag) UART5->ISR;
			}

			static inline ErrorFlag
			getErrorFlags()
			{
				return (ErrorFlag) UART5->ISR;
			}

			static inline void
			resetInterruptFlags(InterruptFlag flags)
			{
				// Flags are cleared by writing a one to the flag position.
				// Writing a zero is (hopefully) ignored.
				UART5->ISR = flags;
			}
		};
	}
}

#endif // XPCC_STM32__UARTHAL_5_HPP
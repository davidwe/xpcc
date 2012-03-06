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
 *
 * $Id: tmp102.hpp 738 2012-02-25 17:54:01Z salkinium $
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__TMP102_HPP
#define XPCC__TMP102_HPP

#include <stdint.h>
#include <xpcc/driver/connectivity/i2c/master.hpp>

namespace xpcc
{
	namespace tmp102
	{
		enum Register
		{
			REGISTER_TEMPERATURE = 0x00,
			REGISTER_CONFIGURATION = 0x01,
			REGISTER_T_LOW = 0x02,
			REGISTER_T_HIGH = 0x03
		};
		
		enum Temperature
		{
			TEMPERATURE_EXTENDED_MODE_bm = 0x01
		};
		
		enum Config1
		{// first byte
			CONFIGURATION_SHUTDOWN_MODE_bm = 0x01,
			CONFIGURATION_THERMOSTAT_MODE_bm = 0x02,
			CONFIGURATION_POLARITY_bm = 0x04,
			CONFIGURATION_FAULT_QUEUE_gm = 0x18,
			CONFIGURATION_FAULT_QUEUE_1_gc = 0x00,
			CONFIGURATION_FAULT_QUEUE_2_gc = 0x08,
			CONFIGURATION_FAULT_QUEUE_4_gc = 0x10,
			CONFIGURATION_FAULT_QUEUE_6_gc = 0x18,
			CONFIGURATION_CONVERTER_RESOLUTION_gm = 0x60,
			CONFIGURATION_CONVERTER_RESOLUTION_12bit_gc = 0x60,
			CONFIGURATION_ONE_SHOT_bm = 0x80
		};
		
		enum Config2
		{// second byte
			CONFIGURATION_EXTENDED_MODE_bm = 0x10,
			CONFIGURATION_ALERT_bm = 0x20,
			CONFIGURATION_CONVERSION_RATE_gm = 0xc0,
			CONFIGURATION_CONVERSION_RATE_0_25Hz_gc = 0x00,
			CONFIGURATION_CONVERSION_RATE_1Hz_gc = 0x40,
			CONFIGURATION_CONVERSION_RATE_4Hz_gc = 0x80,
			CONFIGURATION_CONVERSION_RATE_8Hz_gc = 0xc0
		};
	}
	
	/**
	 * \brief	TMP102 digital temperature sensor driver
	 *
	 * The TMP102 is a digital temperature sensor with a two-wire interface
	 * and measures temperature over a range of -40 to +125 deg Celsius with a
	 * resolution of 1/16 (0.0625) deg C and an accuracy of up to 0.5 deg C.
	 *
	 * The sensor has a default refresh rate of 4Hz but can be raised up to
	 * 30Hz by repeatedly manually starting a conversion (with 
	 * startConversion()), which lasts 26ms.
	 *
	 * To convert the raw data into degrees Celsius, cast the MSB and LSB into
	 * a signed 16bit integer, shift it right by 4 (or 3 in extended mode) and 
	 * devide by 16 (or use the getTemperature() method).
	 *
	 * If you are only interested in the integer value of the temperature,
	 * simply only use the MSB (getData()[0]) when not in extended mode.
	 *
	 * \see <a href="http://www.ti.com/lit/ds/symlink/tmp102.pdf">Datasheet</a>
	 *
	 * \ingroup temperature
	 * \author	Niklas Hauser
	 *
	 * \tparam TwiMaster Asynchronous Interface
	 */
	template < typename TwiMaster >
	class TMP102 : public xpcc::i2c::Delegate
	{
	public:
		/**
		 * Constructor, Default address is 0x48 (alternatives are 0x49, 0x4A and 0x4B)
		 */
		TMP102(uint8_t address=0x48);
		
		bool
		initialize(tmp102::Config1 msb=tmp102::CONFIGURATION_CONVERTER_RESOLUTION_12bit_gc,
				   tmp102::Config2 lsb=tmp102::CONFIGURATION_CONVERSION_RATE_4Hz_gc);
		
		/// starts a temperature conversion right now
		bool
		startConversion();
		
		/**
		 * read the Temperature registers and buffer the results
		 * sets isNewDataAvailable() to \c true
		 */
		bool
		readTemperature();
		
		/**
		 * \c true, when new data has been from the sensor and buffered,
		 * \c false, when the data has already been read, or data is being 
		 * copied into the buffer (by readAccelerationAverage()).
		 */
		bool
		isNewDataAvailable();
		
		/// \return pointer to 8bit array containing temperature
		uint8_t*
		getData();
		
		/// \return the temperature as a signed float in Celcius
		float
		getTemperature();
		
	private:
		/**
		 * this delegate function gets called after calling readTemperature()
		 * \return always \c false, since we do not want to continue using the bus
		 */
		void
		twiCompletion(const uint8_t *data, std::size_t index, bool reading);
		
		bool newData;
		uint8_t config;
		uint8_t data[2];
		uint8_t deviceAddress;
	};
	
}

#include "tmp102_impl.hpp"

#endif // XPCC__TMP102_HPP
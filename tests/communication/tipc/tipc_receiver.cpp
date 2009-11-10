// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2009, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * $Id: raw_tipc_receiver.cpp 88 2009-10-16 23:07:26Z dergraaf $
 */
// ----------------------------------------------------------------------------

#include <iostream>
#include <xpcc/debug/logger/logger.hpp>
#include <xpcc/debug/logger/backend/std/std_log_device.hpp>

#include <xpcc/communication/backend/tipc/tipc.hpp>

xpcc::log::DeviceStd device;

int
main()
{
	xpcc::log::setDevice( device );
	xpcc::log::setFilter(xpcc::log::DEBUG);

	xpcc::log::info << "########## XPCC TIPC RAW Test ##########" << xpcc::flush;

	xpcc::tipc::Tipc tipc;

	tipc.addReceiverId(0x10);
	tipc.addEventId(0x01);

	while(1) {
		if( tipc.isPacketAvailable() ) {
			const xpcc::Header& header =  tipc.getPacketHeader();
			const uint8_t* payload = tipc.getPacketPayload();

			XPCC_LOG_INFO << XPCC_FILE_INFO << "has ";
			XPCC_LOG_INFO << ((header.destination != 0) ? "ACTION" : "EVENT");
			XPCC_LOG_INFO << " from:" << (int)header.source;
			XPCC_LOG_INFO << " value:" << *(int*) payload;
			XPCC_LOG_INFO << xpcc::flush;

			if( header.destination != 0 ) {
				xpcc::Header ackHeader( xpcc::Header::REQUEST, true, header.source, header.destination, 0x01 );
				tipc.sendPacket(ackHeader);
			}

			tipc.dropPacket();
		}
		else {
			XPCC_LOG_DEBUG << XPCC_FILE_INFO << "has no packet" << xpcc::flush;
		}

		usleep(100000);
	}

	xpcc::log::info << "########## XPCC TIPC RAW Test END ##########" << xpcc::flush;
}
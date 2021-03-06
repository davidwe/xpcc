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
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_TIPC_RECEIVER_SOCKET_H_
#define XPCC_TIPC_RECEIVER_SOCKET_H_

#include "header.hpp"

#include <boost/scoped_array.hpp>
#include <stdint.h>

namespace xpcc {
	namespace tipc {
		/**
		 * \class		ReceiverSocket
		 * \brief		Provide the handling of the socket interface from TIPC.
		 * 
		 * \ingroup		tipc
		 * \version		$Id: tipc_receiver_socket.hpp 607 2011-09-13 19:51:03Z dergraaf $
		 * \author		Carsten Schmitt < >
		 */
		class ReceiverSocket {
			public:	
				ReceiverSocket();
				~ReceiverSocket();
		
				void 
				registerOnPacket(	unsigned int typeId,
									unsigned int lowerInstance,
									unsigned int upperInstance);
		
				/**
				 * \param transmitterPortId the id of the tipc port, that transmitted the message is returned
				 * \param tipcHeader the tipc header is returned
				 */
				bool 
				receiveHeader(
						uint32_t & transmitterPortId,
						tipc::Header & tipcHeader );
				
				bool 
				receivePayload(
						uint8_t* payloadPointer,
						size_t payloadLength);
				
				bool 
				popPayload();
		
			private:
				const int socketDescriptor_;
		};
	}
}

#endif /* XPCC_TIPC_RECEIVER_SOCKET_H_ */

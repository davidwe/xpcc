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
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef	XPCC_POSTMAN_HPP
#define	XPCC_POSTMAN_HPP

/**
 * \defgroup 	communication Communication
 * \brief 		Postman delivers messages to the Components.
 *
 * DESC DESC
 *
 * \version		$Id$
 */

#include "../backend/backend_interface.hpp"

namespace xpcc{
	class Postman
	{
	public:
		typedef enum {
			OK,
			NO_COMPONENT,
			NO_ACTION,
			WRONG_ACTION_PARAMETER,
			NO_EVENT,
			WRONG_EVENT_PARAMETER,
			NOT_IMPLEMENTED_YET_ERROR,
		} DeliverInfo;
		
		virtual DeliverInfo
		deliverPacket(const BackendInterface& backend) = 0;
		
		virtual DeliverInfo
		deliverPacket(const Header& header, SmartPointer& payload) = 0;
		
		virtual DeliverInfo
		deliverPacket(const Header& header) = 0;
		
		virtual bool
		isComponentAvaliable(const Header& header) const = 0;
		
	};
}
#endif // XPCC_POSTMAN_HPP
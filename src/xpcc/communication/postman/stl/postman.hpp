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

#ifndef	XPCC_STL_POSTMAN_HPP
#define	XPCC_STL_POSTMAN_HPP

/**
 * \defgroup 	communication Communication
 * \brief 		Default Postman can be used if no other more efficient
 * 				Postman is avalible.
 *
 * DESC DESC
 *
 * \version		$Id$
 */

#include "../postman.hpp"
#include "../../response_callback.hpp"

#include <map>

namespace xpcc
{
	class StlPostman : public Postman
	{
	public:
		typedef ::std::multimap<uint16_t, ResponseCallback> EventMap;
		typedef ::std::map<uint16_t, ResponseCallback > CallbackMap; ///< packetIdentifier -> callback
		typedef ::std::map<uint16_t, CallbackMap > RequestMap; ///< destination -> callbackMap

		StlPostman();

		StlPostman(const EventMap& eventMap, const RequestMap& requenstMap);
		
		~StlPostman();
		
		virtual DeliverInfo
		deliverPacket(const BackendInterface& interface);
		
		virtual DeliverInfo
		deliverPacket(const Header &header, SmartPointer& payload);
				
		virtual DeliverInfo
		deliverPacket(const Header &header);
		
		virtual bool
		isComponentAvaliable(const Header& header) const;

	private:
		EventMap eventMap;
		RequestMap requenstMap;
	};
}



#endif	// XPCC_STL_POSTMAN_HPP
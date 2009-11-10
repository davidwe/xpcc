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
 * $Id: smart_payload.hpp 77 2009-10-15 18:34:29Z thundernail $
 */
// ----------------------------------------------------------------------------

#include "smart_payload.hpp"


xpcc::SmartPayload::SmartPayload()
	: ptr(new uint8_t[2])
{
	ptr[0] = 1;
	ptr[1] = 0;
}

// ----------------------------------------------------------------------------

xpcc::SmartPayload::SmartPayload(const SmartPayload& other)
	: ptr(other.ptr)
{
	ptr[0]++;
}

// ----------------------------------------------------------------------------
		
xpcc::SmartPayload::~SmartPayload()
{
	if (--ptr[0] == 0) {
		delete ptr;
	}
}

// ----------------------------------------------------------------------------

const uint8_t *
xpcc::SmartPayload::getPointer() const {
	if( ptr[1] > 0 ) {
		return &ptr[2];
	}
	else {
		return 0;
	}
}
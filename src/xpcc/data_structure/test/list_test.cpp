// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2009, Roboterclub Aachen e.V.
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
 * $Id$
 */
// ----------------------------------------------------------------------------

#include <xpcc/data_structure/list.hpp>

#include "list_test.hpp"

typedef xpcc::List<int16_t> MyList;

void
ListTest::testAppendAndAt()
{
	MyList list;
	
	MyList::Node node1(1);
	MyList::Node node2(2);
	MyList::Node node3(3);
	
	list.append(&node1);
	list.append(&node2);
	list.append(&node3);
	
	TEST_ASSERT_EQUALS(list.at(0)->getValue(), 1);
	TEST_ASSERT_EQUALS(list.at(1)->getValue(), 2);
	TEST_ASSERT_EQUALS(list.at(2)->getValue(), 3);
	
	MyList::Node *node = list.at(5);
	TEST_ASSERT_TRUE(node == 0);
}

void
ListTest::testPrepend()
{
	MyList list;
	
	MyList::Node node1(1);
	MyList::Node node2(2);
	MyList::Node node3(3);
	
	list.prepend(&node3);
	list.prepend(&node2);
	list.prepend(&node1);
	
	TEST_ASSERT_EQUALS(list.at(0)->getValue(), 1);
	TEST_ASSERT_EQUALS(list.at(1)->getValue(), 2);
	TEST_ASSERT_EQUALS(list.at(2)->getValue(), 3);
}

void
ListTest::testInsertAfter()
{
	MyList list;
	
	MyList::Node node1(1);
	MyList::Node node2(2);
	MyList::Node node3(3);
	MyList::Node node4(4);
	
	list.append(&node1);
	
	list.insertAfter(&node1, &node4);
	list.insertAfter(&node1, &node2);
	list.insertAfter(&node2, &node3);
	
	TEST_ASSERT_EQUALS(list.at(0)->getValue(), 1);
	TEST_ASSERT_EQUALS(list.at(1)->getValue(), 2);
	TEST_ASSERT_EQUALS(list.at(2)->getValue(), 3);
	TEST_ASSERT_EQUALS(list.at(3)->getValue(), 4);
}

void
ListTest::testRemoveAndEmpty()
{
	MyList list;
	
	MyList::Node node1(1);
	MyList::Node node2(2);
	MyList::Node node3(3);
	
	TEST_ASSERT_TRUE(list.isEmpty());
	
	list.append(&node1);
	list.append(&node2);
	list.append(&node3);
	
	TEST_ASSERT_FALSE(list.isEmpty());
	
	list.remove(&node2);
	
	TEST_ASSERT_EQUALS(list.at(0)->getValue(), 1);
	TEST_ASSERT_EQUALS(list.at(1)->getValue(), 3);
	
	list.remove(&node3);
	
	TEST_ASSERT_EQUALS(list.at(0)->getValue(), 1);
	TEST_ASSERT_FALSE(list.isEmpty());
	
	list.remove(&node1);
	
	TEST_ASSERT_TRUE(list.isEmpty());
}

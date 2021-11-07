/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2018-2021, Japan Embedded Systems Technology Association(JASA)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "openEL_linkedList.hpp"

void * HalLinkedList_add__(HAL_LINKED_LIST_T *pNodeBgn, HAL_LINKED_LIST_T *pNodeAdd)
{
    HAL_LINKED_LIST_T *pNodeWk;

    if (nullptr == pNodeBgn)
    {
	pNodeAdd->pNext = nullptr;
	return pNodeAdd;
    }
    pNodeWk = pNodeBgn;
    while (nullptr != pNodeWk->pNext)
    {
	pNodeWk = pNodeWk->pNext;
    }
    pNodeAdd->pNext = nullptr;
    pNodeWk->pNext = pNodeAdd;

    return pNodeBgn;
}

void * HalLinkedList_remove__(HAL_LINKED_LIST_T *pNodeBgn, HAL_LINKED_LIST_T *pNodeRemove)
{
    HAL_LINKED_LIST_T *pNodeWk;
    HAL_LINKED_LIST_T *pNodePrev;
    HAL_LINKED_LIST_T nodeDummy;

    pNodePrev = &nodeDummy;
    pNodePrev->pNext = pNodeBgn;
    pNodeWk = pNodeBgn;
    while (nullptr != pNodeWk)
    {
	if (pNodeWk == pNodeRemove)
	{
	    pNodePrev->pNext = pNodeWk->pNext;
	    return nodeDummy.pNext;
	}
	pNodePrev = pNodeWk;
	pNodeWk = pNodeWk->pNext;
    }
    return pNodeBgn;
}


/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_DETAIL_OBJECTMEMORYSTACK_HPP_
#define DART_COMMON_DETAIL_OBJECTMEMORYSTACK_HPP_

#include <cstddef>

#include "dart/common/Macros.hpp"

namespace dart::common::detail {

//==============================================================================
template <typename T>
class ObjectMemoryStack
{
private:
  struct Node
  {
    union
    {
      Node* next;
      T* object;
    };
  };

public:
  ObjectMemoryStack()
  {
    static_assert(
        sizeof(T) >= sizeof(Node),
        "Object size should be greater than the size of node.");
  }

  ObjectMemoryStack(const ObjectMemoryStack& other) = delete;

  ObjectMemoryStack& operator=(const ObjectMemoryStack& other) = delete;

  void push(T* ptr)
  {
    Node* new_node = reinterpret_cast<Node*>(ptr);
    new_node->next = mHead;
    mHead = new_node;
    mSize++;
  }

  void pushSorted(T* object)
  {
    if (isEmpty())
    {
      push(object);
      return;
    }

    // Find location to add the new object
    DART_ASSERT(mHead != nullptr);
    Node* new_node = reinterpret_cast<Node*>(object);

    if (object < reinterpret_cast<T*>(mHead))
    {
      // Insert object above the current top
      new_node->next = mHead;
      mHead = new_node;
    }
    else
    {
      // Insert object below the iter
      Node* iter = mHead;
      while (true)
      {
        if (iter->next == nullptr || object < reinterpret_cast<T*>(iter->next))
        {
          break;
        }
        iter = iter->next;
      }
      new_node->next = iter->next;
      iter->next = new_node;
    }

    mSize++;
  }

  T* pop()
  {
    if (isEmpty())
    {
      return nullptr;
    }
    Node* top = mHead;
    mHead = mHead->next;
    mSize--;
    return reinterpret_cast<T*>(top);
  }

  size_t getSize() const
  {
    return mSize;
  }

  bool isEmpty() const
  {
    return (mHead == nullptr);
  }

  void clear()
  {
    mHead = nullptr;
    mSize = 0;
  }

  #ifndef NDEBUG
  void printList() const
  {
    Node* head = mHead;
    while (head)
    {
      std::cout << reinterpret_cast<T*>(head) << " ";
      head = head->next;
    }
  }
  #endif

private:
  Node* mHead{nullptr};
  size_t mSize{0};
};

} // namespace dart::common::detail

#endif

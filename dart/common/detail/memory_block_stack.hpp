/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#pragma once

#include "dart/common/macro.hpp"
#include "dart/common/memory_arena.hpp"

namespace dart::common {

class MemoryBlockStack final {
public:
  MemoryBlockStack() noexcept : m_head(nullptr)
  {
    // Do nothing
  }

  ~MemoryBlockStack() noexcept
  {
    // Do nothing
  }

  MemoryBlockStack(MemoryBlockStack&& other) noexcept
    : m_head(std::move(other.m_head))
  {
    // Do nothing
  }

  MemoryBlockStack& operator=(MemoryBlockStack&& other)
  {
    m_head = std::move(other.m_head);
  }

  void push(MemoryBlock block) noexcept
  {
    DART_ASSERT(block.size >= sizeof(Node));
    auto next = ::new (block.begin) Node(m_head, block.size);
    m_head = next;
  }

  MemoryBlock pop()
  {
    auto to_pop = m_head;
    m_head = m_head->prev;
    return MemoryBlock(to_pop, to_pop->usable_size + 0);
  }

  bool empty() const noexcept
  {
    return (m_head == nullptr);
  }

  bool contains(const void* ptr) const noexcept
  {
    auto address = static_cast<const char*>(ptr);
    for (auto curr = m_head; curr; curr = curr->prev) {
      auto mem = static_cast<char*>(static_cast<void*>(curr));
      if (address >= mem && address < mem + curr->usable_size) {
        return true;
      }
    }
    return false;
  }

  std::size_t size() const noexcept
  {
    std::size_t out = 0u;
    for (auto curr = m_head; curr != nullptr; curr = curr->prev) {
      ++out;
    }
    return out;
  }

private:
  struct Node {
    Node* prev;
    std::size_t usable_size;
    Node(Node* prev, std::size_t size) noexcept : prev(prev), usable_size(size)
    {
      // Do nothing
    }
  }

  Node* m_head;
};

} // namespace dart::common

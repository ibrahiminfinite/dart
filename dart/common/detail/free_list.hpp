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

#include <cstddef>
#include <cstdint>
#include <memory>

#include "dart/common/macro.hpp"
#include "dart/common/memory_alignment.hpp"

namespace dart::common::detail {

std::uintptr_t get_int(void* ptr) noexcept
{
  DART_ASSERT(ptr);
  std::uintptr_t out;
  std::memcpy(&out, ptr, sizeof(std::uintptr_t));
  return out;
}

void set_int(void* ptr, std::uintptr_t i)
{
  DART_ASSERT(ptr);
  std::memcpy(ptr, &i, sizeof(std::uintptr_t));
}

inline std::uintptr_t to_int(char* ptr) noexcept
{
  return reinterpret_cast<std::uintptr_t>(ptr);
}

inline char* from_int(std::uintptr_t i) noexcept
{
  return reinterpret_cast<char*>(i);
}

class FreeMemoryList {
public:
  FreeMemoryList(std::size_t node_size) noexcept
  {
  }
  //   FreeMemoryList(std::size_t, void* mem, std::size_t size) noexcept;

  void* allocate() noexcept
  {
    DART_ASSERT(!is_empty());
    --m_capacity;

    auto mem = m_first;
    m_first = from_int(get_int(m_first));
    return mem;
  }

  std::size_t get_node_size() const noexcept
  {
    return m_node_size;
  }

  std::size_t get_alignment() const noexcept
  {
    return alignment_for(m_node_size);
  }

  std::size_t get_capacity() const noexcept
  {
    return m_capacity;
  }

  bool is_empty() const noexcept
  {
    return (m_first == nullptr);
  }

private:
  char* m_first;
  std::size_t m_node_size;
  std::size_t m_capacity;
};

} // namespace dart::common::detail

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

#include "dart/common/linear_allocator.hpp"

#include "dart/common/heap_allocator.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/memory.hpp"

namespace dart::common {

//==============================================================================
LinearAllocator::LinearAllocator(
    std::size_t max_capacity, std::shared_ptr<Allocator> base_allocator)
  : m_max_capacity(max_capacity), m_base_allocator(std::move(base_allocator))
{
#ifndef NDEBUG
  if (max_capacity == 0) {
    DART_WARN(
        "Allocator with zero max capacity is not able to allocate any memory.");
  }
#endif

  // Set base allocator
  if (!m_base_allocator) {
    m_base_allocator = std::make_shared<HeapAllocator>();
  }
  DART_ASSERT(m_base_allocator, "Failed to create base allocator.");

  // Allocate memory chunk
  m_start_ptr = m_base_allocator->allocate(m_max_capacity);
  DART_ASSERT(
      max_capacity == 0 || m_start_ptr,
      "Failed to allocate using the base allocator.");
}

//==============================================================================
LinearAllocator::~LinearAllocator()
{
  m_base_allocator->deallocate(m_start_ptr, m_max_capacity);
}

//==============================================================================
void* LinearAllocator::allocate(std::size_t size, std::size_t alignment)
{
  // Allocating zero memory is not allowed
  if (size == 0) {
    return nullptr;
  }

  const std::size_t current_ptr
      = reinterpret_cast<std::size_t>(m_start_ptr) + m_offset;

  // Compute padding
  std::size_t padding = 0;
  if (alignment > 0 && m_offset % alignment != 0) {
    padding = get_padding(current_ptr, alignment);
  }

  // Check max capacity
  if (m_offset + padding + size > m_max_capacity) {
    DART_DEBUG(
        "Allocating {} with padding {} exceeds the max capacity {}. Returning "
        "nullptr.",
        size,
        padding,
        m_max_capacity);
    return nullptr;
  }

  m_offset += padding + size;

#ifndef NDEBUG
  m_peak = std::max(m_peak, m_offset);
#endif

  return reinterpret_cast<void*>(current_ptr + padding);
}

//==============================================================================
void LinearAllocator::deallocate(void* ptr, std::size_t size)
{
  DART_UNUSED(ptr, size);
}

//==============================================================================
std::size_t LinearAllocator::get_max_capacity() const
{
#if DART_ENABLE_THREAD_SAFE
  std::lock_guard<std::mutex> lock(m_mutex);
#endif
  return m_max_capacity;
}

//==============================================================================
std::size_t LinearAllocator::get_size() const
{
#if DART_ENABLE_THREAD_SAFE
  std::lock_guard<std::mutex> lock(m_mutex);
#endif
  return m_offset;
}

//==============================================================================
const void* LinearAllocator::get_begin_address() const
{
#if DART_ENABLE_THREAD_SAFE
  std::lock_guard<std::mutex> lock(m_mutex);
#endif
  return m_start_ptr;
}

} // namespace dart::common

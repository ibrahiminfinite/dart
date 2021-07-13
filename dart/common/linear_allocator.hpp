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

#if DART_ENABLE_THREAD_SAFE
  #include <mutex>
#endif

#include "dart/common/allocator.hpp"
#include "dart/common/memory.hpp"

namespace dart::common {

class DART_COMMON_API LinearAllocator final : public Allocator {
public:
  /// Constructor
  LinearAllocator(
      std::size_t max_capacity = 64,
      std::shared_ptr<Allocator> base_allocator = nullptr);

  /// Destructor
  ~LinearAllocator() override;

  // Documentation inherited
  [[nodiscard]] void* allocate(
      std::size_t size, std::size_t alignment = 0) override;

  /// This function does nothing. The allocated memory is released when this
  /// allocator is detructed.
  void deallocate(void* ptr, std::size_t size) override;

  /// Returns the maximum capacity of this allocator.
  std::size_t get_max_capacity() const;

  /// Returns the size of allocated memory.
  ///
  /// The return is the same as get_size_in_bytes() if T is void.
  std::size_t get_size() const;

  /// Returns the first address of this allocator uses.
  const void* get_begin_address() const;

private:
#if DART_ENABLE_THREAD_SAFE
  /// Mutex for thread safety
  mutable std::mutex m_mutex;
#endif

  /// The maximum size of memory that this allocator can allocate
  const std::size_t m_max_capacity;

  /// The Memory address of this allocator uses
  void* m_start_ptr = nullptr;

  /// The size of allocated memory or the offset from m_start_ptr.
  std::size_t m_offset = 0;

  /// The base allocator to allocate memory chunck
  std::shared_ptr<Allocator> m_base_allocator;

#ifndef NDEBUG
  // Statics for debugging
  std::size_t m_peak = 0;
#endif
};

} // namespace dart::common

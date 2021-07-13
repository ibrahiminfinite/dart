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
#include <memory>

#include "dart/common/export.hpp"

namespace dart::common {

/// Base class for memory allocators.
///
/// \note The derived classes should be thread safe when DART_ENABLE_THREAD_SAFE
/// is defined to 1
class DART_COMMON_API Allocator {
public:
  /// Constructor
  Allocator() = default;

  /// Destructor
  virtual ~Allocator() = default;

  /// Allocates memory of a given size in bytes and return a pointer to the
  /// allocated memory.
  [[nodiscard]] virtual void* allocate(
      std::size_t size, std::size_t alignment = 0)
      = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Releases previously allocated memory.
  virtual void deallocate(void* ptr, std::size_t size) = 0;
  // TODO(JS): Make this constexpr once migrated to C++20

  /// Creates an object.
  template <typename T, typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  /// Creates an object to an aligned memory.
  template <typename T, typename... Args>
  [[nodiscard]] T* aligned_construct(size_t alignment, Args&&... args) noexcept;

  /// Destroys an object created by this allocator.
  template <typename T>
  void destroy(T* object) noexcept;
};

} // namespace dart::common

#include "dart/common/detail/allocator_impl.hpp"

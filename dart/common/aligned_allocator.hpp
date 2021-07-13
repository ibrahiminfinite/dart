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
#include <cstdlib>
#include <memory>
#include <type_traits>

namespace dart::common {

/// A stateless memory allocator for aligned memory allocation.
///
/// This class is intended to be compatible with std::allocator, so that it can
/// be used with any STL containers for aligned memory allocation.
template <typename T = void, std::size_t Alignment = 32>
struct AlignedAllocator;

// Specialized for void
template <std::size_t Alignment>
struct AlignedAllocator<void, Alignment> {
  using pointer = void*;
  using const_pointer = const void*;
  using value_type = void;

  template <class U>
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };
};

/// A stateless memory allocator for aligned memory allocation.
///
/// This class is intended to be compatible with std::allocator, so that it can
/// be used with any STL containers for aligned memory allocation.
template <typename T, std::size_t Alignment>
struct AlignedAllocator {
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;

  template <class U>
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };

  /// Constructor
  AlignedAllocator() noexcept;

  /// Copy/move constructor
  template <class U>
  AlignedAllocator(AlignedAllocator<U, Alignment>&& /*other*/) noexcept;

  /// Returns the largest supported allocation size
  size_type max_size() const noexcept;

  /// Obtains the address of an object, even if operator& is overloaded.
  ///
  /// \param[in] x: The object to acquire address of
  pointer address(reference x) const noexcept;

  /// Obtains the address of an object, even if operator& is overloaded.
  ///
  /// \param[in] x: The object to acquire address of
  const_pointer address(const_reference x) const noexcept;

  /// Allocates uninitialized storage
  ///
  /// \param[in] n: The number of objects to allocate storage for
  /// \param[in] hint: Pointer to a nearby memory location
  [[nodiscard]] constexpr pointer allocate(
      size_type n,
      typename AlignedAllocator<void, Alignment>::const_pointer hint = nullptr);

  /// Deallocates storage
  ///
  /// \param[in] p: Pointer obtained from allocate()
  /// \param[in] size: Number of objects earlier passed to allocate()
  constexpr void deallocate(pointer p, size_type size) noexcept;

  /// Constructs an object in allocated storage
  ///
  /// \param[in] p: Pointer to allocated uninitialized storage
  /// \param[in] args: The constructor arguments to use
  template <class U, class... Args>
  void construct(U* p, Args&&... args);

  /// Destructs an object in allocated storage
  ///
  /// \param[in] p: Pointer to the object that is going to be destroyed
  void destroy(pointer p);

  /// Returns the alignment value
  [[nodiscard]] static constexpr std::size_t alignment();
};

// Specialized for const type
template <typename T, std::size_t Alignment>
struct AlignedAllocator<const T, Alignment> {
  using value_type = T;
  using pointer = const T*;
  using const_pointer = const T*;
  using reference = const T&;
  using const_reference = const T&;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using propagate_on_container_move_assignment = std::true_type;

  template <class U>
  struct rebind {
    typedef AlignedAllocator<U, Alignment> other;
  };

  /// Constructor
  AlignedAllocator() noexcept;

  /// Copy/move constructor
  template <class U>
  AlignedAllocator(AlignedAllocator<U, Alignment>&& /*other*/) noexcept;

  /// Returns the largest supported allocation size
  size_type max_size() const noexcept;

  /// Obtains the address of an object, even if operator& is overloaded.
  ///
  /// \param[in] x: The object to acquire address of
  const_pointer address(const_reference x) const noexcept;

  /// Allocates uninitialized storage
  ///
  /// \param[in] n: The number of objects to allocate storage for
  /// \param[in] hint: Pointer to a nearby memory location
  [[nodiscard]] constexpr pointer allocate(
      size_type n,
      typename AlignedAllocator<void, Alignment>::const_pointer hint = nullptr);

  /// Deallocates storage
  ///
  /// \param[in] p: Pointer obtained from allocate()
  /// \param[in] size: Number of objects earlier passed to allocate()
  constexpr void deallocate(pointer p, size_type size) noexcept;

  /// Constructs an object in allocated storage
  ///
  /// \param[in] p: Pointer to allocated uninitialized storage
  /// \param[in] args: The constructor arguments to use
  template <class U, class... Args>
  void construct(U* p, Args&&... args);

  /// Destructs an object in allocated storage
  ///
  /// \param[in] p: Pointer to the object that is going to be destroyed
  void destroy(pointer p);

  /// Returns the alignment value
  [[nodiscard]] static constexpr std::size_t alignment();
};

/// Compares two allocator instances
template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
bool operator==(
    const AlignedAllocator<T, TAlign>& allocator1,
    const AlignedAllocator<U, UAlign>& allocator2) noexcept;

/// Compares two allocator instances
template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
bool operator!=(
    const AlignedAllocator<T, TAlign>& allocator1,
    const AlignedAllocator<U, UAlign>& allocator2) noexcept;

} // namespace dart::common

#include "dart/common/detail/aligned_allocator_impl.hpp"

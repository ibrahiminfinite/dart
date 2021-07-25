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

#include <cstdlib>
#include <type_traits>

#include "dart/common/aligned_allocator.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"

namespace dart::common {

//==============================================================================
template <typename T, std::size_t Alignment>
AlignedAllocator<T, Alignment>::AlignedAllocator() noexcept
{
  // Do nothing
}

//==============================================================================
template <typename T, std::size_t Alignment>
template <class U>
AlignedAllocator<T, Alignment>::AlignedAllocator(
    AlignedAllocator<U, Alignment>&& /*other*/) noexcept
{
  // Do nothing
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<T, Alignment>::size_type
AlignedAllocator<T, Alignment>::max_size() const noexcept
{
  return (size_type(~0) - size_type(Alignment)) / sizeof(T);
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<T, Alignment>::pointer
AlignedAllocator<T, Alignment>::address(reference x) const noexcept
{
  return std::addressof(x);
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<T, Alignment>::const_pointer
AlignedAllocator<T, Alignment>::address(const_reference x) const noexcept
{
  return std::addressof(x);
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<T, Alignment>::
    pointer constexpr AlignedAllocator<T, Alignment>::allocate(
        size_type n, typename AlignedAllocator<void, Alignment>::const_pointer)
{
  if (n == 0) {
    return nullptr;
  }

  const size_type alignment = static_cast<size_type>(Alignment);
  void* ptr = std::aligned_alloc(alignment, n * sizeof(T));
  if (ptr == nullptr) {
    throw std::bad_alloc();
  }

  return reinterpret_cast<pointer>(ptr);
}

//==============================================================================
template <typename T, std::size_t Alignment>
constexpr void AlignedAllocator<T, Alignment>::deallocate(
    pointer p, size_type /*size*/) noexcept
{
  return std::free(p);
}

//==============================================================================
template <typename T, std::size_t Alignment>
template <class U, class... Args>
void AlignedAllocator<T, Alignment>::construct(U* p, Args&&... args)
{
  ::new (reinterpret_cast<void*>(p)) U(std::forward<Args>(args)...);
}

//==============================================================================
template <typename T, std::size_t Alignment>
void AlignedAllocator<T, Alignment>::destroy(pointer p)
{
  p->~T();
}

//==============================================================================
template <typename T, std::size_t Alignment>
constexpr std::size_t AlignedAllocator<T, Alignment>::alignment()
{
  return Alignment;
}

//==============================================================================
template <typename T, std::size_t Alignment>
AlignedAllocator<const T, Alignment>::AlignedAllocator() noexcept
{
  // Do nothing
}

//==============================================================================
template <typename T, std::size_t Alignment>
template <class U>
AlignedAllocator<const T, Alignment>::AlignedAllocator(
    AlignedAllocator<U, Alignment>&& /*other*/) noexcept
{
  // Do nothing
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<const T, Alignment>::size_type
AlignedAllocator<const T, Alignment>::max_size() const noexcept
{
  return (size_type(~0) - size_type(Alignment)) / sizeof(T);
}

//==============================================================================
template <typename T, std::size_t Alignment>
typename AlignedAllocator<const T, Alignment>::const_pointer
AlignedAllocator<const T, Alignment>::address(const_reference x) const noexcept
{
  return std::addressof(x);
}

//==============================================================================
template <typename T, std::size_t Alignment>
constexpr typename AlignedAllocator<const T, Alignment>::pointer
AlignedAllocator<const T, Alignment>::allocate(
    size_type n,
    typename AlignedAllocator<void, Alignment>::const_pointer /*hint*/)
{
  if (n == 0) {
    return nullptr;
  }

  const size_type alignment = static_cast<size_type>(Alignment);
  void* ptr = std::aligned_alloc(alignment, n * sizeof(T));
  if (ptr == nullptr) {
    throw std::bad_alloc();
  }

  return reinterpret_cast<pointer>(ptr);
}

//==============================================================================
template <typename T, std::size_t Alignment>
constexpr void AlignedAllocator<const T, Alignment>::deallocate(
    pointer p, size_type /*size*/) noexcept
{
  return std::free(p);
}

//==============================================================================
template <typename T, std::size_t Alignment>
template <class U, class... Args>
void AlignedAllocator<const T, Alignment>::construct(U* p, Args&&... args)
{
  ::new (reinterpret_cast<void*>(p)) U(std::forward<Args>(args)...);
}

//==============================================================================
template <typename T, std::size_t Alignment>
void AlignedAllocator<const T, Alignment>::destroy(pointer p)
{
  p->~T();
}

//==============================================================================
template <typename T, std::size_t Alignment>
constexpr std::size_t AlignedAllocator<const T, Alignment>::alignment()
{
  return Alignment;
}

//==============================================================================
template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
bool operator==(
    const AlignedAllocator<T, TAlign>& /*allocator1*/,
    const AlignedAllocator<U, UAlign>& /*allocator2*/) noexcept
{
  return TAlign == UAlign;
}

//==============================================================================
template <typename T, std::size_t TAlign, typename U, std::size_t UAlign>
bool operator!=(
    const AlignedAllocator<T, TAlign>& /*allocator1*/,
    const AlignedAllocator<U, UAlign>& /*allocator2*/) noexcept
{
  return TAlign != UAlign;
}

} // namespace dart::common

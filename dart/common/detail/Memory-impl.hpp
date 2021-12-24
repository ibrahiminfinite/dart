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

#ifndef DART_COMMON_DETAIL_MEMORY_IMPL_HPP_
#define DART_COMMON_DETAIL_MEMORY_IMPL_HPP_

#include <memory>

#include <Eigen/Core>

#include "dart/common/Macros.hpp"
#include "dart/config.hpp"

#if EIGEN_VERSION_AT_LEAST(3, 2, 1) && EIGEN_VERSION_AT_MOST(3, 2, 8)
  #include "dart/common/detail/AlignedAllocator.hpp"
#else
  #include <Eigen/StdVector>
#endif

namespace dart {
namespace common {

//==============================================================================
template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  using _Tp_nc = typename std::remove_const<_Tp>::type;

#if EIGEN_VERSION_AT_LEAST(3, 2, 1) && EIGEN_VERSION_AT_MOST(3, 2, 8)
  return std::allocate_shared<_Tp>(
      detail::aligned_allocator_cpp11<_Tp_nc>(),
      std::forward<_Args>(__args)...);
#else
  return std::allocate_shared<_Tp>(
      Eigen::aligned_allocator<_Tp_nc>(), std::forward<_Args>(__args)...);
#endif // EIGEN_VERSION_AT_LEAST(3,2,1) && EIGEN_VERSION_AT_MOST(3,2,8)
}

//==============================================================================
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::make_unique<T>(std::forward<Args>(args)...);
}

//==============================================================================
template <std::size_t Alignment>
constexpr std::size_t getPadding(void* basePointer)
{
  if constexpr (Alignment == 0)
  {
    return 0;
  }

  //
  // 0       (Alignment)  (2*Alignment)          (multiplier*Alignment)
  // +------------+-------------+-----...----+-------------+--------------
  //                                            ^          ^
  //                                            |          |
  //                                       base_address   aligned_address
  //                                            |--------->|
  //                                               padding
  //

  const std::size_t baseAddress = reinterpret_cast<std::size_t>(basePointer);
  const std::size_t multiplier = (baseAddress / Alignment) + 1;
  const std::size_t alignedAddress = multiplier * Alignment;
  assert(alignedAddress >= baseAddress);
  const std::size_t padding = alignedAddress - baseAddress;

  return padding;
}

//==============================================================================
constexpr std::size_t getPadding(void* basePointer, std::size_t alignment)
{
  if (alignment == 0)
  {
    return 0;
  }

  //
  // 0       (alignment)  (2*alignment)          (multiplier*alignment)
  // +------------+-------------+-----...----+-------------+--------------
  //                                            ^          ^
  //                                            |          |
  //                                       base_address   aligned_address
  //                                            |--------->|
  //                                               padding
  //

  const std::size_t baseAddress = reinterpret_cast<std::size_t>(basePointer);
  const std::size_t multiplier = (baseAddress / alignment) + 1;
  const std::size_t alignedAddress = multiplier * alignment;
  assert(alignedAddress >= baseAddress);
  const std::size_t padding = alignedAddress - baseAddress;

  return padding;
}

//==============================================================================
template <typename HeaderT>
constexpr std::size_t getPaddingWithHeader(
    void* baseAddress, std::size_t alignment)
{
  if (alignment == 0)
  {
    return sizeof(HeaderT);
  }

  std::size_t padding = getPadding(baseAddress, alignment);
  std::size_t neededSpace = sizeof(HeaderT);

  if (padding < neededSpace)
  {
    neededSpace -= padding;

    if (neededSpace % alignment == 0)
    {
      padding += alignment * (neededSpace / alignment);
    }
    else
    {
      padding += alignment * (1 + (neededSpace / alignment));
    }
  }

  DART_ASSERT(padding >= sizeof(HeaderT));

  return padding;
}

namespace literals {

//==============================================================================
constexpr std::size_t operator"" _KiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _KB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000);
}

//==============================================================================
constexpr std::size_t operator"" _MiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _MB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000);
}

//==============================================================================
constexpr std::size_t operator"" _GiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024 * 1024);
}

//==============================================================================
constexpr std::size_t operator"" _GB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000 * 1000);
}

} // namespace literals

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_MEMORY_IMPL_HPP_

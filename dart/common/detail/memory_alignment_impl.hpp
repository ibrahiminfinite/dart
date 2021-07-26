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

#include "dart/common/logging.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
constexpr bool is_power_of_two(T x)
{
  static_assert(std::is_integral_v<T>, "T should be an integer");
  static_assert(std::is_unsigned_v<T>, "T should be an unsigned integer");

  // First x in the below expression is for the case when x is 0
  return x && (!(x & (x - 1)));
}

//==============================================================================
template <typename T>
constexpr T next_power_of_2(T x)
{
  static_assert(std::is_integral_v<T>, "T should be an integer");
  static_assert(std::is_unsigned_v<T>, "T should be an unsigned integer");

  if (is_power_of_two(x)) {
    return x;
  }

  T count = 0;
  while (x != 0) {
    x >>= 1;
    count += 1;
  }
  return 1 << count;
}

//==============================================================================
constexpr bool is_valid_alignment(std::size_t alignment)
{
  return is_power_of_two(alignment);
}

//==============================================================================
template <typename T>
constexpr T log2ui(T x)
{
  // Adopted from https://www.py4u.net/discuss/63879
  static_assert(std::is_integral_v<T>, "T should be an unsigned integer");
  static_assert(std::is_unsigned_v<T>, "T should be an unsigned integer");
  DART_ASSERT(x > 0, "x shouldn't be zero");

  T i = 0;
  T k = sizeof(T) * CHAR_BIT;
  while (0 < (k /= 2)) {
    if (x >= T(1) << k) {
      i += k;
      x >>= k;
    }
  }
  return i;
}

//==============================================================================
template <typename T>
constexpr T log2iu_floor(T x)
{
  return log2ui(x);
}

//==============================================================================
template <typename T>
constexpr T log2ui_ceil(T x)
{
  return log2ui(x) + T(1) - is_power_of_two(x);
}

//==============================================================================
constexpr std::size_t alignment_for(std::size_t size) noexcept
{
  return size >= get_max_alignment() ? get_max_alignment()
                                     : next_power_of_2(size);
}

//==============================================================================
constexpr std::size_t get_padding(
    const std::size_t base_address, const std::size_t alignment)
{
  if (alignment == 0) {
    return 0;
  }
  const std::size_t multiplier = (base_address / alignment) + 1;
  const std::size_t aligned_address = multiplier * alignment;
  DART_ASSERT(aligned_address >= base_address);
  return aligned_address - base_address;
}

//==============================================================================
static_assert(is_valid_alignment(get_max_alignment()));

} // namespace dart::common

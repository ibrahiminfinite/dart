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
#include <type_traits>

// TODO(JS): Consider merging this file into memory.hpp

namespace dart::common {

constexpr std::size_t get_max_alignment()
{
  return alignof(std::max_align_t);
}

/// Returns whether a number is power of two
template <typename T>
constexpr bool is_power_of_two(T x);

/// Returns the next power of two given number
template <typename T>
constexpr T next_power_of_2(T x);

/// Returns whether an alignment is valid, which should be a power of two but
/// not zero.
constexpr bool is_valid_alignment(std::size_t alignment);

/// Returns log2 for unsigned integers.
template <typename T>
constexpr T log2ui(T x);

template <typename T>
constexpr T log2iu_floor(T x);

template <typename T>
constexpr T log2ui_ceil(T x);

std::size_t align_offset(
    std::uintptr_t address, std::size_t alignment) noexcept;

std::size_t align_offset(void* ptr, std::size_t alignment) noexcept;

bool is_aligned(void* ptr, std::size_t alignment) noexcept;

constexpr std::size_t alignment_for(std::size_t size) noexcept;

constexpr std::size_t get_padding(
    const std::size_t base_address, const std::size_t alignment);

} // namespace dart::common

#include "dart/common/detail/memory_alignment_impl.hpp"

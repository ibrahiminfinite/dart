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

#include "dart/common/memory_alignment.hpp"

#include <climits>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "dart/common/compiler.hpp"
#include "dart/common/macro.hpp"

namespace dart::common {

//==============================================================================
std::size_t align_offset(std::uintptr_t address, std::size_t alignment) noexcept
{
  DART_ASSERT(is_valid_alignment(alignment));
  auto misaligned = address & (alignment - 1);
  return misaligned != 0 ? (alignment - misaligned) : 0;
}

//==============================================================================
std::size_t align_offset(void* ptr, std::size_t alignment) noexcept
{
  return align_offset(reinterpret_cast<std::uintptr_t>(ptr), alignment);
}

//==============================================================================
bool is_aligned(void* ptr, std::size_t alignment) noexcept
{
  DART_ASSERT(is_valid_alignment(alignment));
  auto address = reinterpret_cast<std::uintptr_t>(ptr);
  return address % alignment == 0u;
}

} // namespace dart::common

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

#include "dart/common/heap_allocator.hpp"

#include <cstdlib>

#include "dart/common/logging.hpp"
#include "dart/common/memory.hpp"

namespace dart::common {

//==============================================================================
HeapAllocator::HeapAllocator()
{
  // Do nothing
}

//==============================================================================
void* HeapAllocator::allocate(std::size_t size, std::size_t alignment)
{
  if (size == 0) {
    return nullptr;
  }

  if (alignment == 0) {
    return std::malloc(size);
  }

  if (size % alignment != 0) {
    DART_ERROR(
        "Invalid size to allocate [{}]. The size should be a integer "
        "multiple of the alignment [{}]",
        size,
        alignment);
    return nullptr;
  }

  return common::aligned_alloc(alignment, size);
}

//==============================================================================
void HeapAllocator::deallocate(void* pointer, std::size_t size)
{
  DART_UNUSED(size);
  std::free(pointer);
}

} // namespace dart::common

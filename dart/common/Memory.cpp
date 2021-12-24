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

#include "dart/common/Memory.hpp"

#include "dart/common/Bit.hpp"
#include "dart/common/Compiler.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"

namespace dart::common {

//==============================================================================
bool isValidAlignement(size_t bytes, size_t alignment)
{
  if (alignment == 0)
  {
    return true;
  }

  if (alignment < sizeof(void*))
  {
    DART_DEBUG("Alignment '{}' must be greater than sizeof(void*).", alignment);
    return false;
  }

  if (!ispow2(alignment))
  {
    DART_DEBUG("Alignment '{}' must be a power of 2.", alignment);
    return false;
  }

  if (bytes % alignment != 0)
  {
    DART_DEBUG(
        "Size '{}' must be a multiple of alignment '{}'.", bytes, alignment);
    return false;
  }

  return true;
}

//==============================================================================
void* aligned_alloc(size_t alignment, size_t bytes)
{
#if DART_COMPILER_MSVC
  return _aligned_malloc(bytes, alignment);
#else
  return std::aligned_alloc(alignment, bytes);
#endif
}

//==============================================================================
void aligned_free(void* pointer)
{
#if DART_COMPILER_MSVC
  return _aligned_free(pointer);
#else
  return std::free(pointer);
#endif
}

} // namespace dart::common

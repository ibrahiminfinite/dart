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

#include <fmt/format.h>

#include "dart/common/logging.hpp"
#include "dart/common/string.hpp"

namespace dart::common {

//==============================================================================
template <typename Scalar>
Scalar to_scalar(const std::string& str)
{
  if constexpr (std::is_same_v<Scalar, bool>) {
    return to_bool(str);
  } else if constexpr (std::is_same_v<Scalar, char>) {
    return to_char(str);
  } else if constexpr (std::is_same_v<Scalar, int>) {
    return to_int(str);
  } else if constexpr (std::is_same_v<Scalar, unsigned int>) {
    return to_uint(str);
  } else if constexpr (std::is_same_v<Scalar, long>) {
    return to_long(str);
  } else if constexpr (std::is_same_v<Scalar, long long>) {
    return to_long_long(str);
  } else if constexpr (std::is_same_v<Scalar, float>) {
    return to_float(str);
  } else if constexpr (std::is_same_v<Scalar, double>) {
    return to_double(str);
  } else {
    DART_ERROR(
        "Unsupported scalar type [{}] to convert to.", typeid(Scalar).name());
    return 0;
  }
}

} // namespace dart::common

/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/math/type.hpp"

namespace dart::gui {

template <typename Scalar>
using Color3 = math::Vector3<Scalar>;
using Color3f = Color3<float>;
using Color3d = Color3<double>;

template <typename Scalar>
using Color4 = math::Vector4<Scalar>;
using Color4f = Color4<float>;
using Color4d = Color4<double>;

template <typename Scalar = double>
Color4<Scalar> White(Scalar alpha = 1)
{
  return Color4<Scalar>(1, 1, 1, alpha);
}

template <typename Scalar = double>
Color4<Scalar> LightGray(Scalar alpha = 1)
{
  return Color4<Scalar>(
      200 / (Scalar)255, 200 / (Scalar)255, 200 / (Scalar)255, alpha);
}

template <typename Scalar = double>
Color4<Scalar> Gray(Scalar alpha = 1)
{
  return Color4<Scalar>(
      130 / (Scalar)255, 130 / (Scalar)255, 130 / (Scalar)255, alpha);
}

} // namespace dart::gui

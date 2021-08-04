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

#include "dart/math/export.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/math/lie_group/so3.hpp"

namespace dart::math {

//==============================================================================
template <typename S>
SO3<S> exp(const SO3Tangent<S>& w)
{
  return w.exp();
}

//==============================================================================
template <typename S>
SE3<S> exp(const SE3Tangent<S>& twist)
{
  return twist.exp();
}

//==============================================================================
template <typename S>
SO3Tangent<S> log(const SO3<S>& r)
{
  return r.log();
}

//==============================================================================
template <typename S>
SE3Tangent<S> log(const SE3<S>& tf)
{
  return tf.log();
}

//==============================================================================
template <typename S>
typename SE3<S>::Tangent Ad(const SE3<S>& T, const typename SE3<S>::Tangent& V)
{
  return SE3<S>::Ad(T, V);
}

//==============================================================================
template <typename S>
typename SE3<S>::Tangent ad(
    const typename SE3<S>::Tangent& s1, const typename SE3<S>::Tangent& s2)
{
  return s1.ad(s2);
}

} // namespace dart::math

#include "dart/math/lie_group/detail/lie_group_function_impl.hpp"

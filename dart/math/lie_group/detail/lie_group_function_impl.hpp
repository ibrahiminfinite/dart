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
#include "dart/math/lie_group/lie_group_function.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/math/lie_group/so3.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar>
SO3<Scalar> exp(const SO3Tangent<Scalar>& w)
{
  return w.exp();
}

//==============================================================================
template <typename Scalar>
SE3<Scalar> exp(const SE3Tangent<Scalar>& twist)
{
  return twist.exp();
}

//==============================================================================
template <typename Scalar>
SO3Tangent<Scalar> log(const SO3<Scalar>& r)
{
  return r.log();
}

//==============================================================================
template <typename Scalar>
SE3Tangent<Scalar> log(const SE3<Scalar>& tf)
{
  return tf.log();
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
SE3Tangent<typename DerivedB::Scalar> Ad(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  using Scalar = typename DerivedB::Scalar;
  return SE3<Scalar>::Ad(T, V);
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
SE3Tangent<typename DerivedB::Scalar> Ad_R(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  using Scalar = typename DerivedB::Scalar;
  return SE3<Scalar>::Ad_R(T, V);
}

//==============================================================================
template <typename Scalar>
typename SE3<Scalar>::Tangent Ad_Tinv(
    const SE3<Scalar>& T, const typename SE3<Scalar>::Tangent& V)
{
  return SE3<Scalar>::Ad(T, V);
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
DerivedA ad(
    const SE3TangentBase<DerivedA>& s1, const SE3TangentBase<DerivedB>& s2)
{
  return s1.ad(s2);
}

} // namespace dart::math

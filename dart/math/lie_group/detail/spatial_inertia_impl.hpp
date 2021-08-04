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

#include "dart/math/lie_group/spatial_inertia.hpp"
#include "dart/math/linear_algebra.hpp"

namespace dart::math {

//==============================================================================
template <typename S>
SpatialInertia<S>::SpatialInertia() : m_matrix(Eigen::Matrix<S, 6, 6>::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S>
template <typename Derived>
SpatialInertia<S>::SpatialInertia(
    const Eigen::MatrixBase<Derived>& moment, S mass)
{
  m_matrix.template topLeftCorner<3, 3>() = moment;
  m_matrix.template topRightCorner<3, 3>().setZero();
  m_matrix.template bottomLeftCorner<3, 3>().setZero();
  m_matrix.template bottomRightCorner<3, 3>().noalias()
      = Eigen::Matrix<S, 3, 3>::Identity() * mass;
}

//==============================================================================
template <typename S>
template <typename DerivedA, typename DerivedB>
SpatialInertia<S>::SpatialInertia(
    const Eigen::MatrixBase<DerivedA>& offset,
    const Eigen::MatrixBase<DerivedB>& moment,
    S mass)
{
  const auto p_mat = skew(offset);

  m_matrix.template topLeftCorner<3, 3>() = moment;
  m_matrix.template topRightCorner<3, 3>().noalias() = mass * p_mat;
  m_matrix.template bottomLeftCorner<3, 3>()
      = m_matrix.template topRightCorner<3, 3>().transpose();
  m_matrix.template bottomRightCorner<3, 3>().noalias()
      = Eigen::Matrix<S, 3, 3>::Identity() * mass;
}

//==============================================================================
template <typename S>
template <typename Derived>
SpatialInertia<S>::SpatialInertia(
    const SE3<S>& T, const Eigen::MatrixBase<Derived>& moment, S mass)
{
  const auto R = T.rotation();
  const auto Rt = R.transpose().eval();
  const auto p = T.translation();
  const auto P = skew(p);

  m_matrix.template topLeftCorner<3, 3>().noalias() = R * moment * Rt;
  m_matrix.template topRightCorner<3, 3>().noalias() = mass * P;
  m_matrix.template bottomLeftCorner<3, 3>()
      = m_matrix.template topRightCorner<3, 3>().transpose();
  m_matrix.template bottomRightCorner<3, 3>().noalias()
      = Eigen::Matrix<S, 3, 3>::Identity() * mass;
}

//==============================================================================
template <typename S>
S SpatialInertia<S>::operator()(int row, int col) const
{
  return m_matrix(row, col);
}

//==============================================================================
template <typename S>
S& SpatialInertia<S>::operator()(int row, int col)
{
  return m_matrix(row, col);
}

//==============================================================================
template <typename S>
SE3Cotangent<S> SpatialInertia<S>::operator*(const SE3Tangent<S>& s) const
{
  return SE3Tangent<S>(m_matrix * s.vector());
}

//==============================================================================
template <typename S>
void SpatialInertia<S>::transform(const SE3<S>& T)
{
  const auto AdT = T.ad_matrix();
  m_matrix = AdT.transpose() * m_matrix * AdT;
}

//==============================================================================
template <typename S>
SpatialInertia<S> SpatialInertia<S>::transformed(const SE3<S>& T) const
{
  const auto AdT = T.ad_matrix();
  return SpatialInertia<S>(AdT.transpose() * m_matrix * AdT);
}

} // namespace dart::math

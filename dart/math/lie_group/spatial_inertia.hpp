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

#include "dart/math/lie_group/se3.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

template <typename S_>
class SpatialInertia
{
public:
  using S = S_;

  /// Default constructor
  SpatialInertia();

  template <typename Derived>
  SpatialInertia(const Eigen::MatrixBase<Derived>& moment, S mass);

  template <typename DerivedA, typename DerivedB>
  SpatialInertia(
      const Eigen::MatrixBase<DerivedA>& offset,
      const Eigen::MatrixBase<DerivedB>& moment,
      S mass);

  template <typename Derived>
  SpatialInertia(
      const SE3<S>& T, const Eigen::MatrixBase<Derived>& moment, S mass);

  S operator()(int row, int col) const;

  S& operator()(int row, int col);

  SE3Cotangent<S> operator*(const SE3Tangent<S>& s) const;

  void transform(const SE3<S>& T);

  SpatialInertia<S> transformed(const SE3<S>& T) const;

private:
  Eigen::Matrix<S, 6, 6> m_matrix;
};

} // namespace dart::math

#include "dart/math/lie_group/detail/spatial_inertia_impl.hpp"

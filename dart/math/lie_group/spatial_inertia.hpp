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

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SpatialInertia<Scalar_, Options_>>
{
  static constexpr int Options = Options_;

  using Scalar = Scalar_;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class SpatialInertiaBase
{
public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;

  template <typename OtherDerived>
  SE3Cotangent<Scalar> operator*(
      const SE3TangentBase<OtherDerived>& other) const
  {
    return SE3Cotangent<Scalar>(derived().matrix() * other.vector());
  }

  template <typename SE3Derived>
  void transform(const SE3Base<SE3Derived>& T)
  {
    const auto AdT = T.ad_matrix();
    derived().matrix() = AdT.transpose() * derived().matrix() * AdT;
  }

  template <typename SE3Derived>
  SpatialInertia<Scalar> transformed(const SE3<Scalar>& T) const
  {
    const auto AdT = T.ad_matrix();
    return SpatialInertia<Scalar>(AdT.transpose() * derived().matrix() * AdT);
  }

  Scalar operator()(int row, int col) const
  {
    return derived().matrix()(row, col);
  }

  Scalar& operator()(int row, int col)
  {
    return derived().matrix()(row, col);
  }

protected:
  Derived& derived() noexcept
  {
    return *static_cast<Derived*>(this);
  }

  const Derived& derived() const noexcept
  {
    return *static_cast<const Derived*>(this);
  }

private:
};

//==============================================================================
template <typename Scalar_, int Options_>
class SpatialInertia
  : public SpatialInertiaBase<SpatialInertia<Scalar_, Options_>>
{
public:
  using Scalar = Scalar_;
  static constexpr int Options = Options_;

  using Base = SpatialInertiaBase<SpatialInertia<Scalar, Options>>;
  using MatrixType = Eigen::Matrix<Scalar, 6, 6, Options>;

  /// Default constructor
  SpatialInertia();

  template <typename Derived>
  SpatialInertia(const Eigen::MatrixBase<Derived>& moment, Scalar mass);

  template <typename DerivedA, typename DerivedB>
  SpatialInertia(
      const Eigen::MatrixBase<DerivedA>& offset,
      const Eigen::MatrixBase<DerivedB>& moment,
      Scalar mass);

  template <typename Derived>
  SpatialInertia(
      const SE3<Scalar>& T,
      const Eigen::MatrixBase<Derived>& moment,
      Scalar mass);

  using Base::operator*;

  const MatrixType& matrix() const
  {
    return m_matrix;
  }

  MatrixType& matrix()
  {
    return m_matrix;
  }

private:
  MatrixType m_matrix;
};

} // namespace dart::math

#include "dart/math/lie_group/detail/spatial_inertia_impl.hpp"

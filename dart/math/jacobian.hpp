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
#include "dart/math/type.hpp"

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::SpatialJacobian<Scalar_, Options_>>
{
  static constexpr int Options = Options_;

  using Scalar = Scalar_;
};

} // namespace Eigen::internal

namespace dart::math {

template <typename Derived>
class SpatialJacobianBase
{
public:
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;

protected:
  Derived& derived() noexcept
  {
    return *static_cast<Derived*>(this);
  }

  const Derived& derived() const noexcept
  {
    return *static_cast<const Derived*>(this);
  }

  template <typename OtherDerived>
  SE3Tangent<Scalar> operator*(
      const GeneralizedCoordinatesBase<OtherDerived>& gen_coords) const
  {
    return SE3Tangent<Scalar>(derived().matrix() * gen_coords.vector());
  }

private:
};

template <typename Scalar_, int Dofs, int Options>
class SpatialJacobian
  : public SpatialJacobianBase<SpatialJacobian<Scalar_, Dofs, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = SpatialJacobianBase<SpatialJacobian<Scalar, Dofs, Options>>;
  using MatrixType = Eigen::Matrix<Scalar, 6, Dofs>;

  const MatrixType& matrix() const
  {
    return m_matrix;
  }

  MatrixType& matrix()
  {
    return m_matrix;
  }

protected:
private:
  MatrixType m_matrix;
};

#define DART_MATH_SPATIAL_JACOBIAN(dim)                                        \
  template <typename Scalar, int Options = 0>                                  \
  using SpatialJacobian##dim = SpatialJacobian<Scalar, dim, Options>;          \
  using SpatialJacobian##dim##f = SpatialJacobian##dim<float>;                 \
  using SpatialJacobian##dim##d = SpatialJacobian##dim<double>;                \
  using SpatialJacobian##dim##ld = SpatialJacobian##dim<long double>;

DART_MATH_SPATIAL_JACOBIAN(0)
DART_MATH_SPATIAL_JACOBIAN(1)
DART_MATH_SPATIAL_JACOBIAN(2)
DART_MATH_SPATIAL_JACOBIAN(3)
DART_MATH_SPATIAL_JACOBIAN(4)
DART_MATH_SPATIAL_JACOBIAN(5)
DART_MATH_SPATIAL_JACOBIAN(6)

template <typename Scalar, int Options = 0>
using SpatialJacobianX = SpatialJacobian<Scalar, Eigen::Dynamic, Options>;
using SpatialJacobianXf = SpatialJacobianX<float>;
using SpatialJacobianXd = SpatialJacobianX<double>;
using SpatialJacobianXld = SpatialJacobianX<long double>;

template <typename Scalar_, int Options>
class SpatialJacobian<Scalar_, Eigen::Dynamic, Options>
  : public SpatialJacobianBase<
        SpatialJacobian<Scalar_, Eigen::Dynamic, Options>>
{
public:
  using Scalar = Scalar_;
  using Base
      = SpatialJacobianBase<SpatialJacobian<Scalar, Eigen::Dynamic, Options>>;
  using MatrixType = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

  const MatrixType& matrix() const
  {
    return m_matrix;
  }

  MatrixType& matrix()
  {
    return m_matrix;
  }

protected:
private:
  MatrixType m_matrix;
};

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar_, int Dofs, int Options>
class Map<dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>
  : public dart::math::SpatialJacobianBase<
        Map<dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::SpatialJacobianBase<
      Map<dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>>;
  using MatrixType = Map<Eigen::Matrix<Scalar, 6, Dofs, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(Scalar* data);

  using Base::operator=;

  const MatrixType& matrix() const
  {
    return m_data;
  }

  MatrixType& matrix()
  {
    return m_data;
  }

private:
  MatrixType m_data;
};

//==============================================================================
template <typename Scalar_, int Dofs, int Options>
class Map<const dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>
  : public dart::math::SpatialJacobianBase<
        Map<const dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::SpatialJacobianBase<
      Map<const dart::math::SpatialJacobian<Scalar_, Dofs, Options>, Options>>;
  using MatrixType = Map<const Eigen::Matrix<Scalar, 6, Dofs, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(Scalar* data);

  using Base::operator=;

  const MatrixType& matrix() const
  {
    return m_data;
  }

private:
  MatrixType m_data;
};

} // namespace Eigen

#include "dart/math/detail/jacobian_impl.hpp"

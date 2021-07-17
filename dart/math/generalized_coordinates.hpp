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
template <typename Scalar_, int Dofs_, int Options_>
struct traits<dart::math::GeneralizedCoordinates<Scalar_, Dofs_, Options_>>
{
  static constexpr int Dofs = Dofs_;
  static constexpr int Options = Options_;

  using Scalar = Scalar_;
};

} // namespace Eigen::internal

namespace dart::math {

template <typename Derived>
class GeneralizedCoordinatesBase
{
public:
  static constexpr int Dofs = Eigen::internal::traits<Derived>::Dofs;
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;

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

private:
};

template <typename Scalar_, int Dofs_, int Options_>
class GeneralizedCoordinates
  : public GeneralizedCoordinatesBase<
        GeneralizedCoordinates<Scalar_, Dofs_, Options_>>
{
public:
  static constexpr int Dofs = Dofs_;
  static constexpr int Options = Options_;

  using Scalar = Scalar_;
  using Base = GeneralizedCoordinatesBase<
      GeneralizedCoordinates<Scalar, Dofs, Options>>;
  using VectorType = Eigen::Matrix<Scalar, 6, Dofs>;

  const VectorType& vector() const
  {
    return m_matrix;
  }

  VectorType& vector()
  {
    return m_matrix;
  }

protected:
private:
  VectorType m_matrix;
};

template <typename Scalar, int Options = 0>
using GeneralizedCoordinatesX
    = GeneralizedCoordinates<Scalar, Eigen::Dynamic, Options>;

template <typename Scalar_, int Options>
class GeneralizedCoordinates<Scalar_, Eigen::Dynamic, Options>
  : public GeneralizedCoordinatesBase<
        GeneralizedCoordinates<Scalar_, Eigen::Dynamic, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = GeneralizedCoordinatesBase<
      GeneralizedCoordinates<Scalar, Eigen::Dynamic, Options>>;
  using VectorType = Eigen::Matrix<Scalar, 6, Eigen::Dynamic>;

  const VectorType& vector() const
  {
    return m_matrix;
  }

  VectorType& vector()
  {
    return m_matrix;
  }

protected:
private:
  VectorType m_matrix;
};

} // namespace dart::math

#include "dart/math/detail/generalized_coordinates_impl.hpp"

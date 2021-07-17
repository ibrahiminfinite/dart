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

#include "dart/common/eigen_include.hpp"
#include "dart/math/export.hpp"
#include "dart/math/lie_group/lie_group.hpp"
#include "dart/math/lie_group/lie_group_macro.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

template <typename Derived>
struct Eval
{
};

template <typename ExpressionType, template <typename> class StorageBase>
class LieGroupNoAlias
{
public:
  LieGroupNoAlias(ExpressionType& expression) : m_expression(expression)
  {
    // Do nothing
  }

  template <typename OtherDerived>
  ExpressionType& operator=(const StorageBase<OtherDerived>& other)
  {
    (void)other;
    DART_NOT_IMPLEMENTED;
    return m_expression;
  }

private:
  ExpressionType& m_expression;
};

template <typename Derived>
class LieGroupBase
{
public:
  // Properties
  static constexpr int Options = Eigen::internal::traits<Derived>::Options;
  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;
  static constexpr int RepDim = Eigen::internal::traits<Derived>::RepDim;

  // Type defs
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;
  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;
  // using LieGroupData = typename
  // Eigen::internal::traits<Derived>::LieGroupData;
  using LieGroupCoeffs =
      typename Eigen::internal::traits<Derived>::LieGroupCoeffs;
  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;
  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;
  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  using Rotation = math::Matrix<Scalar, SpaceDim, SpaceDim>;
  using Translation = math::Vector<Scalar, SpaceDim>;
  using Transformation = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry>;

  void set_identity();

  const LieGroupCoeffs& coeffs() const;

  LieGroupCoeffs& coeffs();

  const Derived& derived() const noexcept;

  Derived& derived() noexcept;
};

template <typename Derived>
class LieAlgebra
{
public:
  // Properties
  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;

  // Type defs
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;

  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;

  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;

  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;

  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;

  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  LieAlgebraData& matrix();

  const LieAlgebraData& matrix() const;

protected:
  Derived& derived() noexcept;

  const Derived& derived() const noexcept;
};

template <typename Derived>
class LieGroupTangent
{
public:
  // Properties
  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;

  // Type defs
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;

  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;

  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;

  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;

  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;

  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  static Tangent Zero();

  auto& vector();

  const auto& vector() const;

protected:
  Derived& derived() noexcept;

  const Derived& derived() const noexcept;
};

template <typename Derived>
class LieGroupCotangent
{
public:
  // Properties
  static constexpr int SpaceDim = Eigen::internal::traits<Derived>::SpaceDim;
  static constexpr int GroupDim = Eigen::internal::traits<Derived>::GroupDim;
  static constexpr int MatrixDim = Eigen::internal::traits<Derived>::MatrixDim;

  // Type defs
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using Matrix = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;

  using LieGroup = typename Eigen::internal::traits<Derived>::LieGroup;

  using Tangent = typename Eigen::internal::traits<Derived>::Tangent;
  using TangentData = typename Eigen::internal::traits<Derived>::TangentData;

  using Cotangent = typename Eigen::internal::traits<Derived>::Cotangent;
  using CotangentData =
      typename Eigen::internal::traits<Derived>::CotangentData;

  using LieAlgebra = typename Eigen::internal::traits<Derived>::LieAlgebra;
  using LieAlgebraData =
      typename Eigen::internal::traits<Derived>::LieAlgebraData;

  using Jacobian = typename Eigen::internal::traits<Derived>::Jacobian;

  static Cotangent Zero();

  CotangentData& vector();

  const CotangentData& vector() const;

protected:
  Derived& derived() noexcept;

  const Derived& derived() const noexcept;
};

} // namespace dart::math

#include "dart/math/lie_group/detail/lie_group_impl.hpp"

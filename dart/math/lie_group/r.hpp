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

#include "dart/math/constant.hpp"
#include "dart/math/lie_group/lie_group.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

// Forward declaration
template <typename S, int Dim, int Options = 0>
class R;
template <typename S, int Dim, int Options = 0>
class RAlgebra;
template <typename S, int Dim, int Options = 0>
class RTangent;
template <typename S, int Dim, int Options = 0>
class RCotangent;

} // namespace dart::math

namespace Eigen::internal {

//==============================================================================
template <typename S_, int Dim, int Options_>
struct traits<dart::math::R<S_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using S = S_;

  using LieGroup = dart::math::R<S, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<S, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<S, GroupDim, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, Options>;

  using Cotangent = dart::math::RCotangent<S, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<S, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::R<S_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using S = S_;

  using LieGroup = dart::math::R<S, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<S, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<S, GroupDim, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, Options>;

  using Cotangent = dart::math::RCotangent<S, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<S, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class RBase : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;
  using Rotation = typename Base::Rotation;
  using Transformation = typename Base::Transformation;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;
};

//==============================================================================
/// Translation group where the dimension is determined in compile-time.
template <typename S, int N, int Options>
class R : public RBase<R<S, N, Options>>
{
public:
  using Base = LieGroupBase<R<S, N>>;
  using LieGroupData = math::Vector<S, N>;
  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;
  using Rotation = typename Base::Rotation;
  using Transformation = typename Base::Transformation;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  static R Zero();

  static R Identity();

  static R Random();

  /// Default constructor
  R();

  /// Copy constructor
  R(const R& other);

  /// Move constructor
  R(R&& other);

  /// Constructs from vector
  template <typename Derived>
  R(const Eigen::MatrixBase<Derived>& vec);

  /// Constructs from vector
  template <typename Derived>
  R(Eigen::MatrixBase<Derived>&& vec);

  /// Destructor
  ~R() = default;

  /// Assignment operator
  R& operator=(const R& other);

  /// Move operator
  R& operator=(R&& other);

  template <typename Derived>
  R& operator=(const Eigen::MatrixBase<Derived>& matrix);

  template <typename Derived>
  R& operator=(Eigen::MatrixBase<Derived>&& matrix);

  R operator-() const;

  R operator+(const R& other) const;

  void set_identity();

  void set_random();

  constexpr int dimension() const;

  LieGroupData& vector();

  const LieGroupData& vector() const;

protected:
  using Base::derived;

  LieGroupData m_vector;
};

//==============================================================================
/// Translation group where the dimension is determined in runtime.
template <typename S, int Options>
class R<S, Eigen::Dynamic, Options>
  : public LieGroupBase<R<S, Eigen::Dynamic, Options>>
{
public:
  using Base = LieGroupBase<R<S, Eigen::Dynamic>>;
  using LieGroupData = math::Vector<S, Eigen::Dynamic>;
  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;
  using Rotation = typename Base::Rotation;
  using Transformation = typename Base::Transformation;

  static constexpr int GroupDim = Base::GroupDim;

  /// Default constructor
  R();

  /// Constructor
  /// @param[in] dim: Dimension of this group.
  explicit R(int dim);

  /// Copy constructor
  R(const R& other);

  /// Move constructor
  R(R&& other);

  /// Constructs from vector
  template <typename Derived>
  explicit R(const Eigen::MatrixBase<Derived>& vector);

  /// Constructs from vector
  template <typename Derived>
  explicit R(Eigen::MatrixBase<Derived>&& vector);

  /// Destructor
  ~R() = default;

  /// Assign operator
  template <typename Derived>
  R& operator=(const Eigen::MatrixBase<Derived>& matrix);

  /// Move operator
  template <typename Derived>
  R& operator=(Eigen::MatrixBase<Derived>&& matrix);

  const R& operator+() const;

  R operator-() const;

  int dimension() const;

  const LieGroupData& vector() const;

  LieGroupData& mutable_vector();

protected:
  using Base::derived;

  math::VectorX<S> m_vector;
};

#define DART_MATH_REAL_VECTOR_SPACE(dim)                                       \
  /** Translation group of dim##-dimension. */                                 \
  template <typename S, int Options = 0>                                       \
  using R##dim = R<S, dim, Options>;                                           \
  /** Translation group of dim##-dimension for float scalar type. */           \
  using R##dim##f = R##dim<float>;                                             \
  /** Translation group of dim##-dimension for double scalar type. */          \
  using R##dim##d = R##dim<double>;                                            \
  /** Translation group of dim##-dimension for long double scalar type. */     \
  using R##dim##ld = R##dim<long double>

/// Translation group of dynamic-size.
template <typename S>
using RX = R<S, Eigen::Dynamic>;
/// Translation group of dynamic-size for float scalar type.
using RXf = RX<float>;
/// Translation group of dynamic-size for double scalar type.
using RXd = RX<double>;
/// Translation group of dynamic-size for long double scalar type.
using RXld = RX<long double>;

DART_MATH_REAL_VECTOR_SPACE(1);
DART_MATH_REAL_VECTOR_SPACE(2);
DART_MATH_REAL_VECTOR_SPACE(3);
DART_MATH_REAL_VECTOR_SPACE(4);
DART_MATH_REAL_VECTOR_SPACE(5);
DART_MATH_REAL_VECTOR_SPACE(6);

//==============================================================================
template <typename S, int Dim, int Options>
class RTangent : public LieGroupTangent<RTangent<S, Dim, Options>>
{
public:
  using Base = LieGroupTangent<RTangent<S, Dim, Options>>;
  using Data = typename Base::Data;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Default constructor
  RTangent();

  /// Copy constructor
  RTangent(const RTangent& other);

  /// Move constructor
  RTangent(RTangent&& other);

  /// Copy constructor for coeffs
  template <typename Derived>
  RTangent(const Eigen::MatrixBase<Derived>& coeffs);

  /// Move constructor for coeffs
  template <typename Derived>
  RTangent(Eigen::MatrixBase<Derived>&& coeffs);

  template <typename OtherDerived>
  RTangent(const LieGroupTangent<OtherDerived>& other);

  S operator*(const RCotangent<S, Dim>& cotan) const;

  LieAlgebra hat() const;

protected:
  using Base::derived;

  Data m_data;
};

//==============================================================================
template <typename S, int N, int Options>
class RCotangent : public LieGroupCotangent<RCotangent<S, N, Options>>
{
public:
  using Base = LieGroupCotangent<RTangent<S, N, Options>>;
  using Data = typename Base::Data;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

protected:
  using Base::derived;

  Data m_data;
};

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S_, int N, int Options>
class Map<dart::math::R<S_, N, Options>, Options>
  : public dart::math::RBase<Map<dart::math::R<S_, N, Options>, Options>>
{
public:
  using S = S_;
  using Base = dart::math::RBase<Map<dart::math::R<S_, N, Options>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  Map(S* data);

private:
  Map<Eigen::Matrix<S, N, 1, Options>> m_data;
};

//==============================================================================
template <typename S_, int N, int Options>
class Map<const dart::math::R<S_, N, Options>, Options>
  : public dart::math::RBase<Map<const dart::math::R<S_, N, Options>, Options>>
{
public:
  using S = S_;
  using Base
      = dart::math::RBase<Map<const dart::math::R<S_, N, Options>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  Map(S* data);

private:
  Map<Eigen::Matrix<S, N, 1, Options>> m_data;
};

} // namespace Eigen

#include "dart/math/lie_group/detail/r_impl.hpp"

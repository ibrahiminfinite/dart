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
#include "dart/math/lie_group/r.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

// Forward declaration
template <typename S, int Options = 0>
class SO3;
template <typename S, int Options = 0>
class SO3Algebra;
template <typename S, int Options = 0>
class SO3Tangent;
template <typename S, int Options = 0>
class SO3Cotangent;

} // namespace dart::math

namespace Eigen::internal {

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SO3<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  /// Dimension of internal parameters to represent the group element.
  static constexpr int RepDim = 4;

  using Scalar = S_;
  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Eigen::Quaternion<S, Options>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SO3Algebra<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  static constexpr int RepDim = 9;

  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Eigen::Quaternion<S, Options>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SO3Tangent<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  static constexpr int RepDim = 3;

  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Eigen::Quaternion<S, Options>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SO3Cotangent<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  static constexpr int RepDim = 3;

  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Eigen::Quaternion<S, Options>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<Map<dart::math::SO3<S_, Options_>>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  static constexpr int RepDim = 4;

  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Map<Eigen::Quaternion<S, Options>>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<Map<const dart::math::SO3<S_, Options_>>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 3;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 3;

  static constexpr int RepDim = 4;

  using S = S_;

  using LieGroup = dart::math::SO3<S, Options>;
  using QuaternionType = Map<const Eigen::Quaternion<S, Options>>;
  using LieGroupCoeffs = typename Eigen::Quaternion<S, Options>::Coefficients;

  using Rotation = Eigen::Matrix<S, 3, 3>;
  using Transformation
      = Eigen::Transform<S, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::SO3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SO3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SO3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class SO3Base : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using Matrix = typename Base::Matrix;
  using LieGroupCoeffs = typename Base::LieGroupCoeffs;
  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;

  // Eigen data
  using Rotation = typename Base::Rotation;
  using Translation = typename Base::Translation;
  using Transformation = typename Base::Transformation;

  static constexpr int SpaceDim = Base::SpaceDim;
  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

protected:
  /// Default constructor
  SO3Base() = default;

  SO3Base(const SO3Base& other) = default;

  SO3Base(SO3Base&& other) = default;

  ~SO3Base() = default;

public:
  Derived& operator=(const SO3Base& other);

  template <typename OtherDerived>
  Derived& operator=(const LieGroupBase<OtherDerived>& other);

  template <typename EigenDerived>
  Derived& operator=(const Eigen::MatrixBase<EigenDerived>& other);

  Derived& operator=(SO3Base&& other);

  template <typename OtherDerived>
  Derived& operator=(LieGroupBase<OtherDerived>&& other);

  template <typename EigenDerived>
  Derived& operator=(Eigen::MatrixBase<EigenDerived>&& other);

  using Base::coeffs;

protected:
  using Base::set_identity;

  using Base::derived;
};

//==============================================================================
template <typename S, int Options>
class SO3 : public SO3Base<SO3<S, Options>>
{
public:
  using Base = LieGroupBase<SO3<S, Options>>;
  using Matrix = typename Base::Matrix;
  using LieGroupData = math::Quaternion<S, Options>;
  using LieGroupCoeffs = typename Base::LieGroupCoeffs;
  using Tangent = typename Base::Tangent;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;
  using QuaternionType = Eigen::Quaternion<S, Options>;

  // Eigen data
  using Rotation = typename Base::Rotation;
  using Translation = typename Base::Translation;
  using Transformation = typename Base::Transformation;

  static constexpr int GroupDim = Base::GroupDim;

  static SO3<S, Options> Identity();

  static SO3<S, Options> Random();

  /// Default constructor
  SO3();

  /// Constructs from quaternion
  SO3(const math::Quaternion<S, Options>& quat);

  /// Constructs from quaternion
  SO3(math::Quaternion<S, Options>&& quat);

  /// Destructor
  ~SO3() = default;

  template <typename Derived>
  SO3<S, Options>& operator=(const Eigen::MatrixBase<Derived>& matrix);

  /// Group operation
  SO3 operator*(const SO3& other) const;

  R3<S> operator*(const R3<S>& other) const;

  SO3Algebra<S, Options> operator*(const SO3Algebra<S, Options>& dx) const;

  /// @{ @name Euler angles

  Eigen::Matrix<S, 3, 1> euler_angles(int axis1, int axis2, int axis3) const;

  Eigen::Matrix<S, 3, 1> euler_angles_intrinsic(
      int axis1, int axis2, int axis3) const;

  Eigen::Matrix<S, 3, 1> euler_angles_extrinsic(
      int axis1, int axis2, int axis3) const;

  Eigen::Matrix<S, 3, 1> rpy() const;

  /// @}

  void set_identity();

  void set_random();

  SO3 inverse() const;

  SO3& inverse_in_place();

  Tangent log(Jacobian* jacobian = nullptr, S tolerance = eps<S>()) const;

  Tangent ad(const Tangent& V) const;

  Jacobian ad_matrix() const;

  Transformation transformation() const;

  Matrix matrix() const;

  Rotation rotation() const;

  Translation translation() const;

  const LieGroupData& quaternion() const;

  const auto& coeffs() const;

protected:
  using Base::derived;

  LieGroupData m_data;

private:
  void normalize();
};

DART_TEMPLATE_CLASS_SCALAR(SO3)

//==============================================================================
template <typename S, int Options>
class SO3Algebra : public LieAlgebra<SO3Algebra<S, Options>>
{
public:
  using Base = LieAlgebra<SO3Algebra<S, Options>>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebraData = typename Base::LieAlgebraData;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  /// Default constructor
  SO3Algebra();

  /// Copy constructor
  SO3Algebra(const SO3Algebra& other);

  /// Move constructor
  SO3Algebra(SO3Algebra&& other);

  /// Constructs from 3x3 matrix
  template <typename Derived>
  explicit SO3Algebra(const Eigen::MatrixBase<Derived>& matrix);

  /// Constructs from 3x3 matrix
  template <typename Derived>
  explicit SO3Algebra(Eigen::MatrixBase<Derived>&& matrix);

  SO3Algebra operator/(S scalar) const;

  SO3Tangent<S, Options> vee() const;

  LieAlgebraData& mutable_matrix();

  const LieAlgebraData& matrix() const;

protected:
  using Base::derived;

  LieAlgebraData m_matrix;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Algebra)

//==============================================================================
template <typename S, int Options>
class SO3Tangent : public LieGroupTangent<SO3Tangent<S, Options>>
{
public:
  using This = SO3Tangent;
  using Base = LieGroupTangent<SO3Tangent<S, Options>>;
  using QuaternionType = typename Eigen::internal::traits<This>::QuaternionType;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  /// Default constructor
  SO3Tangent();

  /// Copy constructor
  SO3Tangent(const SO3Tangent& other);

  /// Move constructor
  SO3Tangent(SO3Tangent&& other);

  /// Copy constructor for coeffs
  template <typename Derived>
  SO3Tangent(const Eigen::MatrixBase<Derived>& coeffs);

  /// Move constructor for coeffs
  template <typename Derived>
  SO3Tangent(Eigen::MatrixBase<Derived>&& coeffs);

  template <typename OtherDerived>
  SO3Tangent(const LieGroupTangent<OtherDerived>& other);

  SO3Tangent& operator=(const SO3Tangent<S, Options>& other);

  SO3Tangent& operator=(SO3Tangent<S, Options>&& other);

  S operator[](int index) const;

  S& operator[](int index);

  SO3Tangent operator-() const;

  S operator*(const SO3Cotangent<S, Options>& cotan) const;

  void set_zero();

  void set_random();

  SO3Algebra<S, Options> hat() const;

  LieGroup exp(Jacobian* jacobian = nullptr, S tolerance = eps<S>()) const;

  Jacobian left_jacobian(S tolerance = eps<S>()) const;

  Jacobian space_jacobian(S tolerance = eps<S>()) const;

  Jacobian right_jacobian(S tolerance = eps<S>()) const;

  Jacobian body_jacobian(S tolerance = eps<S>()) const;

  Jacobian left_jacobian_inverse(S tolerance = eps<S>()) const;

  Jacobian right_jacobian_inverse(S tolerance = eps<S>()) const;

  template <typename Derived>
  Jacobian left_jacobian_time_derivative(
      const math::MatrixBase<Derived>& dq, S tolerance = eps<S>()) const;

  Jacobian left_jacobian_time_derivative(
      int index, S tolerance = eps<S>()) const;

  std::array<Jacobian, 3> left_jacobian_time_derivative(
      S tolerance = eps<S>()) const;

  template <typename Derived>
  Jacobian right_jacobian_time_derivative(
      const math::MatrixBase<Derived>& dq, S tolerance = eps<S>()) const;

  Jacobian right_jacobian_time_derivative(
      int index, S tolerance = eps<S>()) const;

  std::array<Jacobian, 3> right_jacobian_time_derivative(
      S tolerance = eps<S>()) const;

  SO3Tangent ad(const SO3Tangent& other) const;

  Jacobian ad_matrix() const;

  TangentData& vector();

  const TangentData& vector() const;

  static SO3Tangent<S, Options> Random();

protected:
  using Base::derived;

  TangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Tangent)

//==============================================================================
template <typename S, int Options>
class SO3Cotangent : public LieGroupCotangent<SO3Cotangent<S, Options>>
{
public:
  using This = SO3Cotangent<S, Options>;
  using Base = LieGroupCotangent<This>;
  using LieGroup = typename Base::LieGroup;
  using CotangentData = typename Base::CotangentData;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  CotangentData& vector();
  const CotangentData& vector() const;

protected:
  using Base::derived;

  CotangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SO3Cotangent)

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S_, int Options>
class Map<dart::math::SO3<S_, Options>, Options>
  : public dart::math::SO3Base<Map<dart::math::SO3<S_, Options>, Options>>
{
public:
  using S = S_;
  using Base = dart::math::SO3Base<Map<dart::math::SO3<S_, Options>, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(S* data);

  using Base::operator=;

  const Map<Eigen::Quaternion<S, Options>, Options>& quaternion() const;

  Map<Eigen::Quaternion<S, Options>, Options>& quaternion();

private:
  Map<Eigen::Quaternion<S, Options>> m_data;
};

//==============================================================================
template <typename S_, int Options>
class Map<const dart::math::SO3<S_, Options>, Options>
  : public dart::math::SO3Base<Map<const dart::math::SO3<S_, Options>, Options>>
{
public:
  using S = S_;
  using Base
      = dart::math::SO3Base<Map<const dart::math::SO3<S_, Options>, Options>>;

  /// Constructor
  ///
  /// @param[in] data: Pointer to an array of scalar values that this map to use
  /// as its underlying data. The size should be at least 4.
  Map(const S* data);

  using Base::operator=;

  /// Returns
  const Map<Eigen::Quaternion<S, Options>, Options>& quaternion() const;

private:
  Map<const Eigen::Quaternion<S, Options>> m_data;
};

} // namespace Eigen

namespace dart::math {

template <typename S, int Options = 0>
using SO3Map = Eigen::Map<SO3<S, Options>>;

template <typename S, int Options = 0>
using ConstSO3Map = Eigen::Map<const SO3<S, Options>>;

} // namespace dart::math

#include "dart/math/lie_group/detail/so3_impl.hpp"

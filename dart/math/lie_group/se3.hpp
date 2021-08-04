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
#include "dart/math/lie_group/so3.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

// Forward declaration
template <typename S, int Options = 0>
class SE3;
template <typename S, int Options = 0>
class SE3Algebra;
template <typename S, int Options = 0>
class SE3Tangent;
template <typename S, int Options = 0>
class SE3Cotangent;

} // namespace dart::math

namespace Eigen::internal {

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SE3<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 6;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 4;

  /// Dimension of internal parameters to represent the group element.
  static constexpr int RepDim = traits<dart::math::SO3<S_>>::RepDim
                                + traits<dart::math::R3<S_>>::RepDim;

  using S = S_;

  using LieGroup = dart::math::SE3<S, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, RepDim, 1, Options>;

  using Tangent = dart::math::SE3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SE3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SE3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SE3Algebra<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 6;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 4;

  static constexpr int RepDim = MatrixDim * MatrixDim;

  using S = S_;

  using LieGroup = dart::math::SE3<S, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, RepDim, 1, Options>;

  using Tangent = dart::math::SE3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SE3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SE3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SE3Tangent<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 6;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 4;

  static constexpr int RepDim = 6;

  using S = S_;

  using LieGroup = dart::math::SE3<S, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, RepDim, 1, Options>;

  using Tangent = dart::math::SE3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SE3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SE3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename S_, int Options_>
struct traits<dart::math::SE3Cotangent<S_, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = 6;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = 4;

  static constexpr int RepDim = 6;

  using S = S_;

  using LieGroup = dart::math::SE3<S, Options>;
  using LieGroupCoeffs = Eigen::Matrix<S, RepDim, 1, Options>;

  using Tangent = dart::math::SE3Tangent<S, Options>;
  using TangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using Cotangent = dart::math::SE3Cotangent<S, Options>;
  using CotangentData = Eigen::Matrix<S, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::SE3Algebra<S, Options>;
  using LieAlgebraData = Eigen::Matrix<S, MatrixDim, MatrixDim, Options>;

  using Jacobian = Eigen::Matrix<S, GroupDim, GroupDim, Options>;
};

} // namespace Eigen::internal

namespace dart::math {

//==============================================================================
template <typename Derived>
class SE3Base : public LieGroupBase<Derived>
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
};

//==============================================================================
template <typename S, int Options>
class SE3 : public SE3Base<SE3<S>>
{
public:
  using Base = LieGroupBase<SE3<S>>;
  using Matrix = typename Base::Matrix;
  using LieAlgebra = typename Base::LieAlgebra;
  using Tangent = typename Base::Tangent;
  using TangentData = typename Base::TangentData;
  using Cotangent = typename Base::Cotangent;
  using Jacobian = typename Base::Jacobian;

  // Eigen data
  using Rotation = typename Base::Rotation;
  using Translation = typename Base::Translation;
  using Transformation = typename Base::Transformation;

  static SE3 Identity();

  static SE3 Random();

  static Tangent Ad(const SE3& T, const typename SE3::Tangent& V);

  /// Default constructor
  SE3();

  /// Copy constructor
  SE3(const SE3& other);

  /// Move constructor
  SE3(SE3&& other);

  /// Constructs from orientation and position
  explicit SE3(const SO3<S, Options>& orientation, const R3<S>& position);

  /// Constructs from orientation and position
  explicit SE3(SO3<S, Options>&& orientation, R3<S>&& position);

  /// Assignment operator
  SE3& operator=(const SE3& other);

  /// Move operator
  SE3& operator=(SE3&& other);

  /// Assign operator for Eigen::Isometry3
  SE3& operator=(const Eigen::Transform<S, 3, Eigen::Isometry, Options>& tf);

  /// Group operation
  SE3 operator*(const SE3& other) const;

  /// Transforms a 3D point
  R3<S> operator*(const R3<S>& position) const;

  SE3Algebra<S, Options> operator*(const SE3Algebra<S, Options>& dx) const;

  void set_identity();

  void set_random();

  SE3 inverse() const;

  SE3& inverse_in_place();

  Tangent log(Jacobian* jacobian = nullptr, S tolerance = eps<S>()) const;

  Tangent ad(const Tangent& V) const;

  Jacobian ad_matrix() const;

  SO3<S>& mutable_orientation();

  const SO3<S>& orientation() const;

  R3<S>& mutable_position();

  const R3<S>& position() const;

  Transformation transformation() const;

  Matrix matrix() const;

  Rotation rotation() const;

  Translation& mutable_translation();

  const Translation& translation() const;

protected:
  using Base::derived;

  /// Orientation component
  SO3<S, Options> m_orientation;

  /// Position component
  R3<S> m_position;
};

DART_TEMPLATE_CLASS_SCALAR(SE3)

//==============================================================================
template <typename S, int Options>
class SE3Algebra : public LieAlgebra<SE3Algebra<S>>
{
public:
  using This = SE3Algebra;
  using Base = LieAlgebra<SE3Algebra<S>>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebraData = typename Base::LieAlgebraData;
  using Jacobian = typename Base::Jacobian;

  static constexpr int GroupDim = Base::GroupDim;
  static constexpr int MatrixDim = Base::MatrixDim;

  static SE3Algebra Zero();

  /// Default constructor
  SE3Algebra();

  /// Copy constructor
  SE3Algebra(const SE3Algebra& other);

  /// Move constructor
  SE3Algebra(SE3Algebra&& other);

  /// Constructs from the vector form of se(3)
  explicit SE3Algebra(const SE3Tangent<S, Options>& tangent);

  /// Constructs from the vector form of se(3)
  explicit SE3Algebra(SE3Tangent<S, Options>&& tangent);

  explicit SE3Algebra(const LieAlgebraData& data);

  explicit SE3Algebra(LieAlgebraData&& data);

  template <typename DerivedA, typename DerivedB>
  explicit SE3Algebra(
      const Eigen::MatrixBase<DerivedA>& angular,
      const Eigen::MatrixBase<DerivedB>& linear);

  void set_zero();

  SE3Algebra operator/(S scalar) const;

  SE3Tangent<S, Options> vee() const;

  LieAlgebraData& mutable_matrix();

  const LieAlgebraData& matrix() const;

protected:
  using Base::derived;

  LieAlgebraData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SE3Algebra)

//==============================================================================
template <typename S, int Options>
class SE3Tangent : public LieGroupTangent<SE3Tangent<S>>
{
public:
  using Base = LieGroupTangent<SE3Tangent<S>>;
  using Matrix = typename Base::Matrix;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Default constructor
  SE3Tangent();

  /// Copy constructor
  SE3Tangent(const SE3Tangent& other);

  /// Move constructor
  SE3Tangent(SE3Tangent&& other);

  /// Copy constructor for coeffs
  template <typename Derived>
  explicit SE3Tangent(const Eigen::MatrixBase<Derived>& vector);

  /// Move constructor for coeffs
  template <typename Derived>
  explicit SE3Tangent(Eigen::MatrixBase<Derived>&& vector);

  template <typename DerivedA, typename DerivedB>
  explicit SE3Tangent(
      const Eigen::MatrixBase<DerivedA>& angular,
      const Eigen::MatrixBase<DerivedB>& linear);

  template <typename OtherDerived>
  explicit SE3Tangent(const LieGroupTangent<OtherDerived>& other);

  SE3Tangent& operator=(const SE3Tangent& other);

  SE3Tangent& operator=(SE3Tangent&& other);

  S operator[](int index) const;

  S& operator[](int index);

  S operator*(const SE3Cotangent<S, Options>& wrench) const;

  void set_zero();

  void set_random();

  /// Hat operation
  SE3Algebra<S, Options> hat() const;

  /// Vee operation, reverse of hat().
  template <typename Derived>
  void vee(const Eigen::MatrixBase<Derived>& mat);

  SE3<S, Options> exp(
      Jacobian* jacobian = nullptr, S tolerance = eps<S>()) const;

  SE3Tangent ad(const SE3Tangent& other) const;

  Matrix ad_matrix() const;

  Jacobian left_jacobian(S tolerance = eps<S>()) const;

  Jacobian space_jacobian(S tolerance = eps<S>()) const;

  Jacobian right_jacobian(S tolerance = eps<S>()) const;

  Jacobian body_jacobian(S tolerance = eps<S>()) const;

  Jacobian left_jacobian_inverse(S tolerance = eps<S>()) const;

  Jacobian right_jacobian_inverse(S tolerance = eps<S>()) const;

  auto angular() const;

  auto mutable_angular();

  auto linear() const;

  auto mutable_linear();

  TangentData& vector();

  const TangentData& vector() const;

  static SE3Tangent Random();

protected:
  using Base::derived;

  TangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SE3Tangent)

//==============================================================================
template <typename S, int Options_>
class SE3Cotangent : public LieGroupCotangent<SE3Cotangent<S>>
{
public:
  using Base = LieGroupCotangent<SE3Cotangent<S>>;
  using CotangentData = typename Base::CotangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Default constructor
  SE3Cotangent();

  /// Copy constructor
  SE3Cotangent(const SE3Cotangent& other);

  /// Move constructor
  SE3Cotangent(SE3Cotangent&& other);

  auto torque() const;

  auto mutable_torque();

  auto force() const;

  auto mutable_force();

  auto angular() const;

  auto mutable_angular();

  auto linear() const;

  auto mutable_linear();

  CotangentData& vector();

  const CotangentData& vector() const;

protected:
  using Base::derived;

  CotangentData m_data;
};

DART_TEMPLATE_CLASS_SCALAR(SE3Cotangent)

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S_, int Options>
class Map<dart::math::SE3<S_, Options>, Options>
  : public dart::math::SE3Base<Map<dart::math::SE3<S_, Options>, Options>>
{
public:
  using S = S_;
  using Base = dart::math::SE3Base<Map<dart::math::SE3<S_, Options>, Options>>;

  using Base::operator=;

  Map(S* data);

private:
  Map<dart::math::SO3<S, Options>> m_orientation;
  Map<dart::math::R3<S, Options>> m_position;
};

//==============================================================================
template <typename S_, int Options>
class Map<const dart::math::SE3<S_, Options>, Options>
  : public dart::math::SE3Base<Map<const dart::math::SE3<S_, Options>, Options>>
{
public:
  using S = S_;
  using Base
      = dart::math::SE3Base<Map<const dart::math::SE3<S_, Options>, Options>>;

  using Base::operator=;

  Map(const S* data);

private:
  Map<const dart::math::SO3<S, Options>> m_orientation;
  Map<const dart::math::R3<S, Options>> m_position;
};

} // namespace Eigen

#include "dart/math/lie_group/detail/se3_impl.hpp"

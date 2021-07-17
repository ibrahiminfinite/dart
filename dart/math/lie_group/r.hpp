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

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::R<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::R<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::RTangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::RTangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<dart::math::RCotangent<Scalar_, Dim, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<dart::math::RCotangent<Scalar_, Eigen::Dynamic, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = 3;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = Eigen::Dynamic;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
};

//==============================================================================
template <typename Scalar_, int Dim, int Options_>
struct traits<Map<dart::math::RTangent<Scalar_, Dim>, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Dim;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Dim;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Dim;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  typedef Eigen::Dense StorageKind;
  typedef Eigen::MatrixXpr XprKind;
  typedef typename Data::StorageIndex StorageIndex;
  enum
  {
    Flags = Eigen::ColMajor,
    RowsAtCompileTime = Data::RowsAtCompileTime,
    ColsAtCompileTime = Data::RowsAtCompileTime,
    MaxRowsAtCompileTime = Data::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = Data::MaxRowsAtCompileTime,

    InnerStrideAtCompileTime = Data::InnerStrideAtCompileTime,
    SizeAtCompileTime = Data::SizeAtCompileTime,
  };
};

//==============================================================================
template <typename Scalar_, int Options_>
struct traits<Map<dart::math::RTangent<Scalar_, Eigen::Dynamic>, Options_>>
{
  static constexpr int Options = Options_;

  /// Dimension of the Euclidean space that the Lie group is embedded.
  static constexpr int SpaceDim = Eigen::Dynamic;

  /// Dimension of the Lie group.
  static constexpr int GroupDim = Eigen::Dynamic;

  /// Dimension of the matrix representation of the Lie group.
  static constexpr int MatrixDim = GroupDim + 1;

  static constexpr int RepDim = Eigen::Dynamic;

  using Scalar = Scalar_;

  using LieGroup = dart::math::R<Scalar, GroupDim, Options>;
  using LieGroupData = Eigen::Matrix<Scalar, MatrixDim, Options>;
  using LieGroupCoeffs = Eigen::Matrix<Scalar, MatrixDim, Options>;

  using Rotation = Eigen::Matrix<Scalar, 3, 3>;
  using Transformation
      = Eigen::Transform<Scalar, SpaceDim, Eigen::Isometry, Options>;

  using Tangent = dart::math::RTangent<Scalar, GroupDim, Options>;
  using TangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using Cotangent = dart::math::RCotangent<Scalar, GroupDim, Options>;
  using CotangentData = Eigen::Matrix<Scalar, GroupDim, 1, Options>;

  using LieAlgebra = dart::math::RAlgebra<Scalar, GroupDim, Options>;
  using LieAlgebraData = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

  using Data = Eigen::Matrix<Scalar, GroupDim, 1, Options>;
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

  template <typename OtherDerived>
  bool operator==(const RBase<OtherDerived>& other) const
  {
    return vector() == other.vector();
  }

  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  auto vector() const
  {
    return derived().vector();
  }

  auto& vector()
  {
    return derived().vector();
  }

protected:
  using Base::derived;
};

//==============================================================================
/// Translation group where the dimension is determined in compile-time.
template <typename Scalar, int N, int Options>
class R : public RBase<R<Scalar, N, Options>>
{
public:
  using Base = LieGroupBase<R<Scalar, N>>;
  using LieGroupData = math::Vector<Scalar, N>;
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
  R(const R& other) = default;

  /// Move constructor
  R(R&& other) = default;

  /// Copy constructor from OtherDerived
  template <typename OtherDerived>
  R(const RBase<OtherDerived>& other);

  /// Move constructor from OtherDerived
  template <typename OtherDerived>
  R(RBase<OtherDerived>&& other);

  /// Constructs from Derived
  template <typename Derived>
  R(const Eigen::MatrixBase<Derived>& vec);

  /// Constructs from Derived
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

  template <typename OtherDerived>
  bool operator!=(const RBase<OtherDerived>& other) const
  {
    return m_data != other.vector();
  }

  R operator-() const;

  R operator+(const R& other) const;

  R operator-(const R& other) const;

  void set_identity();

  void set_random();

  constexpr int dimension() const;

  bool is_identity() const
  {
    return this->is_zero();
  }

  LieGroupData& vector();

  const LieGroupData& vector() const;

protected:
  using Base::derived;

  LieGroupData m_data;
};

//==============================================================================
/// Translation group where the dimension is determined in runtime.
template <typename Scalar, int Options>
class R<Scalar, Eigen::Dynamic, Options>
  : public RBase<R<Scalar, Eigen::Dynamic, Options>>
{
public:
  using Base = RBase<R<Scalar, Eigen::Dynamic>>;
  using LieGroupData = math::Vector<Scalar, Eigen::Dynamic>;
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

  math::VectorX<Scalar> m_data;
};

//==============================================================================
template <typename Derived>
class RTangentBase : public LieGroupTangent<Derived>
{
public:
  using Base = LieGroupTangent<Derived>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Assign operator
  template <typename OtherDerived>
  Derived& operator=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  template <typename OtherDerived>
  Derived& operator=(RTangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  /// Assign operator
  ///
  /// Allows to assign the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() = other.vector();
    return derived();
  }

  /// Move operator
  ///
  /// Allows to move the tangent space of SO(3)
  template <typename OtherDerived>
  Derived& operator=(SO3TangentBase<OtherDerived>&& other)
  {
    derived().vector() = std::move(other.vector());
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator+=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() += other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator+=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() += other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const RTangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  template <typename OtherDerived>
  Derived& operator-=(const SO3TangentBase<OtherDerived>& other)
  {
    derived().vector() -= other.vector();
    return derived();
  }

  /// Returns whether the vector is zero
  bool is_zero() const
  {
    return derived().vector().isZero();
  }

  auto noalias()
  {
    return derived().vector().noalias();
  }

  friend std::ostream& operator<<(
      std::ostream& os, const RTangentBase<Derived>& x)
  {
    os << x.derived().vector().transpose();
    return os;
  }

protected:
  using Base::derived;
};

//==============================================================================
template <typename Scalar, int Dim, int Options>
class RTangent : public RTangentBase<RTangent<Scalar, Dim, Options>>
{
public:
  using Base = RTangentBase<RTangent<Scalar, Dim, Options>>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Default constructor
  RTangent();

  /// Copy constructor
  RTangent(const RTangent& other);

  /// Move constructor
  RTangent(RTangent&& other);

  template <typename OtherDerived>
  RTangent(const RTangentBase<OtherDerived>& other);

  template <typename OtherDerived>
  RTangent(RTangentBase<OtherDerived>&& other);

  /// Copy constructor for coeffs
  template <typename Derived>
  RTangent(const Eigen::MatrixBase<Derived>& coeffs);

  /// Move constructor for coeffs
  template <typename Derived>
  RTangent(Eigen::MatrixBase<Derived>&& coeffs);

  template <typename Derived>
  RTangent(const LieGroupTangent<Derived>& other);

  using Base::operator=;

  Scalar operator*(const RCotangent<Scalar, Dim>& cotan) const;

  LieAlgebra hat() const;

  const auto& vector() const
  {
    return m_data;
  }

  auto& vector()
  {
    return m_data;
  }

protected:
  using Base::derived;

  TangentData m_data;
};

//==============================================================================
template <typename Scalar, int Options>
class RTangent<Scalar, Eigen::Dynamic, Options>
  : public RTangentBase<RTangent<Scalar, Eigen::Dynamic, Options>>
{
public:
  using Base = RTangentBase<RTangent<Scalar, Eigen::Dynamic, Options>>;
  using TangentData = typename Base::TangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  /// Default constructor
  explicit RTangent(int size = 0);

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

  template <typename Derived>
  RTangent& operator=(const RTangentBase<Derived>& other)
  {
    m_data = other.vector();
    return *this;
  }

  template <typename Derived>
  RTangent& operator=(RTangentBase<Derived>&& other)
  {
    m_data = std::move(other.vector());
    return *this;
  }

  Scalar operator*(const RCotangent<Scalar, Eigen::Dynamic>& cotan) const;

  LieAlgebra hat() const;

  const auto& vector() const
  {
    return m_data;
  }

  auto& vector()
  {
    return m_data;
  }

protected:
  using Base::derived;

  TangentData m_data;
};

//==============================================================================
template <typename Scalar, int N, int Options>
class RCotangent : public LieGroupCotangent<RCotangent<Scalar, N, Options>>
{
public:
  using Base = LieGroupCotangent<RCotangent<Scalar, N, Options>>;
  using CotangentData = typename Base::CotangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  bool is_zero() const
  {
    return m_data.isZero();
  }

  const auto& vector() const
  {
    return m_data;
  }

  auto& vector()
  {
    return m_data;
  }

protected:
  using Base::derived;

  CotangentData m_data;
};

//==============================================================================
template <typename Scalar, int Options>
class RCotangent<Scalar, Eigen::Dynamic, Options>
  : public LieGroupCotangent<RCotangent<Scalar, Eigen::Dynamic, Options>>
{
public:
  using Base = LieGroupCotangent<RCotangent<Scalar, Eigen::Dynamic, Options>>;
  using CotangentData = typename Base::CotangentData;
  using LieGroup = typename Base::LieGroup;
  using LieAlgebra = typename Base::LieAlgebra;
  using Jacobian = typename Base::Jacobian;

  explicit RCotangent(int size = 0) : m_data(CotangentData::Zero(size))
  {
    // Do nothing
  }

  bool is_zero() const
  {
    return m_data.isZero();
  }

  const auto& vector() const
  {
    return m_data;
  }

  auto& vector()
  {
    return m_data;
  }

protected:
  using Base::derived;

  CotangentData m_data;
};

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename Scalar_, int N, int Options>
class Map<dart::math::R<Scalar_, N, Options>, Options>
  : public dart::math::RBase<Map<dart::math::R<Scalar_, N, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base
      = dart::math::RBase<Map<dart::math::R<Scalar_, N, Options>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  Map(Scalar* data);

  const auto& vector() const
  {
    return m_data;
  }

  auto& vector()
  {
    return m_data;
  }

private:
  Map<Eigen::Matrix<Scalar, N, 1, Options>> m_data;
};

//==============================================================================
template <typename Scalar_, int N, int Options>
class Map<const dart::math::R<Scalar_, N, Options>, Options>
  : public dart::math::RBase<
        Map<const dart::math::R<Scalar_, N, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::RBase<
      Map<const dart::math::R<Scalar_, N, Options>, Options>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  Map(const Scalar* data);

  const auto& vector() const
  {
    return m_data;
  }

private:
  Map<const Eigen::Matrix<Scalar, N, 1, Options>> m_data;
};

//==============================================================================
template <typename Scalar_, int N, int Options>
class Map<dart::math::RTangent<Scalar_, N, Options>, Options>
  : public dart::math::RTangentBase<
        Map<dart::math::RTangent<Scalar_, N, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::RTangentBase<
      Map<dart::math::RTangent<Scalar_, N, Options>, Options>>;

  Map(Scalar* data);

  using Base::operator=;
  using Base::is_zero;

  const Map<Eigen::Matrix<Scalar, N, 1, Options>>& vector() const
  {
    return m_data;
  }

  Map<Eigen::Matrix<Scalar, N, 1, Options>>& vector()
  {
    return m_data;
  }

private:
  Map<Eigen::Matrix<Scalar, N, 1, Options>> m_data;
};

//==============================================================================
template <typename Scalar_, int N, int Options>
class Map<const dart::math::RTangent<Scalar_, N, Options>, Options>
  : public dart::math::RTangentBase<
        Map<const dart::math::RTangent<Scalar_, N, Options>, Options>>
{
public:
  using Scalar = Scalar_;
  using Base = dart::math::RTangentBase<
      Map<const dart::math::RTangent<Scalar_, N, Options>, Options>>;

  Map(const Scalar* data);

  using Base::operator=;
  using Base::is_zero;

  const Map<const Eigen::Matrix<Scalar, N, 1, Options>>& vector() const
  {
    return m_data;
  }

private:
  Map<const Eigen::Matrix<Scalar, N, 1, Options>> m_data;
};

} // namespace Eigen

#include "dart/math/lie_group/detail/r_impl.hpp"

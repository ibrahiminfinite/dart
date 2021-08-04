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

#include "dart/math/lie_group/lie_group.hpp"
#include "dart/math/lie_group/se3.hpp"

namespace dart::math {

//==============================================================================
template <typename S, int Options>
SE3<S, Options> SE3<S, Options>::Identity()
{
  return SE3(SO3<S, Options>::Identity(), R3<S, Options>::Identity());
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options> SE3<S, Options>::Random()
{
  return SE3(SO3<S, Options>::Random(), R3<S, Options>::Random());
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>::SE3()
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>::SE3(const SE3<S, Options>& other)
  : m_orientation(other.m_orientation), m_position(other.m_position)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>::SE3(SE3<S, Options>&& other)
  : m_orientation(std::move(other.m_orientation)),
    m_position(std::move(other.m_position))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>::SE3(const SO3<S, Options>& orientation, const R3<S>& position)
  : m_orientation(orientation), m_position(position)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>::SE3(SO3<S, Options>&& orientation, R3<S>&& position)
  : m_orientation(std::move(orientation)), m_position(std::move(position))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>& SE3<S, Options>::operator=(const SE3<S, Options>& other)
{
  m_orientation = other.m_orientation;
  m_position = other.m_position;
  return *this;
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>& SE3<S, Options>::operator=(SE3<S, Options>&& other)
{
  m_orientation = std::move(other.m_orientation);
  m_position = std::move(other.m_position);
  return *this;
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>& SE3<S, Options>::operator=(
    const Eigen::Transform<S, 3, Eigen::Isometry, Options>& tf)
{
  m_orientation = tf.rotation();
  m_position = tf.translation();
  return *this;
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options> SE3<S, Options>::operator*(const SE3<S, Options>& other) const
{
  return SE3<S, Options>(
      m_orientation * other.m_orientation,
      m_position + m_orientation * other.m_position);
}

//==============================================================================
template <typename S, int Options>
R3<S> SE3<S, Options>::operator*(const R3<S>& position) const
{
  return m_orientation * position + m_position;
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options> SE3<S, Options>::operator*(
    const SE3Algebra<S, Options>& dx) const
{
  return SE3Algebra<S, Options>(matrix() * dx.matrix());
}

//==============================================================================
template <typename S, int Options>
void SE3<S, Options>::set_identity()
{
  m_orientation.set_identity();
  m_position.set_identity();
}

//==============================================================================
template <typename S, int Options>
void SE3<S, Options>::set_random()
{
  m_orientation.set_random();
  m_position.set_random();
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options> SE3<S, Options>::inverse() const
{
  const auto r_inv = m_orientation.inverse();
  return SE3<S, Options>(r_inv, -(r_inv * m_position));
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options>& SE3<S, Options>::inverse_in_place()
{
  *this = inverse();
  return *this;
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Tangent SE3<S, Options>::log(
    Jacobian* jacobian, S tolerance) const
{
  Tangent out;
  out.mutable_angular() = m_orientation.log(nullptr, tolerance).vector();
  out.mutable_linear().noalias()
      = SO3Tangent<S>(out.angular()).left_jacobian_inverse(tolerance)
        * m_position.vector();
  // TODO(JS): Use map

  if (jacobian) {
    (*jacobian) = out.right_jacobian_inverse(tolerance);
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Tangent SE3<S, Options>::ad(
    const SE3<S, Options>::Tangent& V) const
{
  return Ad(*this, V);
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Jacobian SE3<S, Options>::ad_matrix() const
{
  Jacobian out;
  const auto rot_mat = m_orientation.quaternion().toRotationMatrix();
  out.template topLeftCorner<3, 3>() = rot_mat;
  out.template topRightCorner<3, 3>().setZero();
  out.template bottomLeftCorner<3, 3>().noalias()
      = skew(m_position.vector()) * rot_mat;
  out.template bottomRightCorner<3, 3>() = rot_mat;
  return out;
}

//==============================================================================
template <typename S, int Options>
SO3<S>& SE3<S, Options>::mutable_orientation()
{
  return m_orientation;
}

//==============================================================================
template <typename S, int Options>
const SO3<S>& SE3<S, Options>::orientation() const
{
  return m_orientation;
}

//==============================================================================
template <typename S, int Options>
R3<S>& SE3<S, Options>::mutable_position()
{
  return m_position;
}

//==============================================================================
template <typename S, int Options>
const R3<S>& SE3<S, Options>::position() const
{
  return m_position;
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Transformation SE3<S, Options>::transformation() const
{
  Transformation out = Transformation::Identity();
  out.linear() = std::move(rotation());
  out.translation() = std::move(translation());
  return out;
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Matrix SE3<S, Options>::matrix() const
{
  return transformation().matrix();
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Rotation SE3<S, Options>::rotation() const
{
  return m_orientation.rotation();
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Translation& SE3<S, Options>::mutable_translation()
{
  return m_position.vector();
}

//==============================================================================
template <typename S, int Options>
const typename SE3<S, Options>::Translation& SE3<S, Options>::translation()
    const
{
  return m_position.vector();
}

//==============================================================================
template <typename S, int Options>
typename SE3<S, Options>::Tangent SE3<S, Options>::Ad(
    const SE3<S, Options>& T, const typename SE3<S, Options>::Tangent& V)
{
  const auto& orientation = T.orientation();
  const auto& position = T.position();

  // Using rotation matrix is more efficient when multiplying 3d vector more
  // than once
  const auto rotation = orientation.rotation();
  const auto& vector = position.vector();

  TangentData data;
  data.template head<3>().noalias() = rotation * V.vector().template head<3>();
  data.template tail<3>().noalias() = rotation * V.vector().template tail<3>()
                                      + vector.cross(data.template head<3>());

  return SE3<S, Options>::Tangent(std::move(data));
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options> SE3Algebra<S, Options>::Zero()
{
  SE3Algebra out;
  out.set_zero();
  return out;
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra() : m_data(LieAlgebraData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(const SE3Algebra& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(SE3Algebra&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(const SE3Tangent<S, Options>& tangent)
{
  const auto& vector = tangent.vector();
  m_data.setZero();
  m_data.template topLeftCorner<3, 3>() = skew(vector.template head<3>());
  m_data.template topRightCorner<3, 1>() = vector.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(SE3Tangent<S, Options>&& tangent)
{
  const auto& vector = tangent.vector();
  m_data.setZero();
  m_data.template topLeftCorner<3, 3>() = skew(vector.template head<3>());
  m_data.template topRightCorner<3, 1>() = vector.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(const LieAlgebraData& data) : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options>::SE3Algebra(LieAlgebraData&& data)
  : m_data(std::move(data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename DerivedA, typename DerivedB>
SE3Algebra<S, Options>::SE3Algebra(
    const Eigen::MatrixBase<DerivedA>& angular,
    const Eigen::MatrixBase<DerivedB>& linear)
{
  m_data.template topLeftCorner<3, 3>() = angular;
  m_data.template topRightCorner<3, 1>() = linear;
  m_data.template bottomRows<1>().setZero();
}

//==============================================================================
template <typename S, int Options>
void SE3Algebra<S, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options> SE3Algebra<S, Options>::operator/(S scalar) const
{
  return SE3Algebra<S, Options>(m_data / scalar);
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options> SE3Algebra<S, Options>::vee() const
{
  return SE3Tangent<S, Options>(
      Eigen::Matrix<S, 3, 1>(m_data(2, 1), m_data(0, 2), m_data(1, 0)),
      m_data.template topRightCorner<3, 1>());
}

//==============================================================================
template <typename S, int Options>
typename SE3Algebra<S, Options>::LieAlgebraData&
SE3Algebra<S, Options>::mutable_matrix()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SE3Algebra<S, Options>::LieAlgebraData&
SE3Algebra<S, Options>::matrix() const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options>::SE3Tangent() : m_data(TangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options>::SE3Tangent(const SE3Tangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options>::SE3Tangent(SE3Tangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SE3Tangent<S, Options>::SE3Tangent(const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SE3Tangent<S, Options>::SE3Tangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename DerivedA, typename DerivedB>
SE3Tangent<S, Options>::SE3Tangent(
    const Eigen::MatrixBase<DerivedA>& angular,
    const Eigen::MatrixBase<DerivedB>& linear)
{
  m_data.template head<3>() = angular;
  m_data.template tail<3>() = linear;
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options>& SE3Tangent<S, Options>::operator=(
    const SE3Tangent<S, Options>& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options>& SE3Tangent<S, Options>::operator=(
    SE3Tangent<S, Options>&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename S, int Options>
S SE3Tangent<S, Options>::operator[](int index) const
{
  return m_data[index];
}

//==============================================================================
template <typename S, int Options>
S& SE3Tangent<S, Options>::operator[](int index)
{
  return m_data[index];
}

//==============================================================================
template <typename S, int Options>
S SE3Tangent<S, Options>::operator*(
    const SE3Cotangent<S, Options>& wrench) const
{
  return m_data.dot(wrench.m_data);
}

//==============================================================================
template <typename S, int Options>
void SE3Tangent<S, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename S, int Options>
void SE3Tangent<S, Options>::set_random()
{
  const SE3<S, Options> tf = SE3<S, Options>::Random();
  *this = tf.log();
}

//==============================================================================
template <typename S, int Options>
SE3Algebra<S, Options> SE3Tangent<S, Options>::hat() const
{
  return SE3Algebra<S, Options>(*this);
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
void SE3Tangent<S, Options>::vee(const math::MatrixBase<Derived>& mat)
{
  if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) {
    DART_ASSERT(mat.rows() == 3);
  } else {
    static_assert(Derived::RowsAtCompileTime == 3, "Invalid size of rows.");
  }

  if constexpr (Derived::ColsAtCompileTime == Eigen::Dynamic) {
    DART_ASSERT(mat.cols() == 3);
  } else {
    static_assert(Derived::ColsAtCompileTime == 3, "Invalid size of cols.");
  }

  m_data.template head<3>() << mat(2, 1), mat(0, 2), mat(1, 0);
  m_data.template tail<3>();
  DART_NOT_IMPLEMENTED;
}

//==============================================================================
template <typename S, int Options>
SE3<S, Options> SE3Tangent<S, Options>::exp(
    Jacobian* jacobian, S tolerance) const
{
  // TODO(JS): Make this a map
  const auto so3_tangent = SO3Tangent<S>(m_data.template head<3>());
  const auto so3_exp = so3_tangent.exp(nullptr, tolerance);
  const auto so3_left_jacobian = so3_tangent.left_jacobian(tolerance);

  const SE3<S, Options> out(
      so3_exp, so3_left_jacobian * m_data.template tail<3>());

  if (jacobian) {
    (*jacobian) = right_jacobian();
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options> SE3Tangent<S, Options>::ad(const SE3Tangent& other) const
{
  //--------------------------------------------------------------------------
  // ad(s1, s2) = | [w1]    0 | | w2 |
  //              | [v1] [w1] | | v2 |
  //
  //            = |          [w1]w2 |
  //              | [v1]w2 + [w1]v2 |
  //--------------------------------------------------------------------------

  TangentData data;
  const TangentData& vec1 = m_data;
  const TangentData& vec2 = other.m_data;
  data.template head<3>()
      = vec1.template head<3>().cross(vec2.template head<3>());
  data.template tail<3>()
      = vec1.template head<3>().cross(vec2.template tail<3>())
        + vec1.template tail<3>().cross(vec2.template head<3>());
  return SE3Tangent(std::move(data));
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Matrix SE3Tangent<S, Options>::ad_matrix()
    const
{
  // TODO(JS): Create matrix type instead of using Jacobian

  Matrix out;

  out.template topLeftCorner<3>() = skew(m_data.template head<3>());
  out.template topRightCorner<3>().setZero();
  out.template topLeftCorner<3>() = skew(m_data.template tail<3>());
  out.template topLeftCorner<3>() = out.template topLeftCorner<3>();

  return out;
}

//==============================================================================
template <typename Derived, typename S>
Eigen::Matrix<S, 3, 3> compute_q(
    const Eigen::MatrixBase<Derived>& data, S tolerance)
{
  const auto& a = data.template head<3>().eval();
  const auto& b = data.template tail<3>().eval();

  const auto A = skew(a);
  const auto B = skew(b);

  const S t = a.norm();

  Eigen::Matrix<S, 3, 3> out;

  if (t < tolerance) {
    out.noalias() = 0.5 * B;
  } else {
    const S t2 = t * t;
    const S t3 = t2 * t;
    const S t4 = t3 * t;
    const S t5 = t4 * t;
    const auto AB = (A * B).eval();
    const auto BA = (B * A).eval();
    const auto ABA = (AB * A).eval();
    const auto AAB = (A * AB).eval();
    const auto BAA = (BA * A).eval();
    const auto ABAA = (ABA * A).eval();
    const auto AABA = (A * ABA).eval();
    const S st = std::sin(t);
    const S ct = std::cos(t);
    // clang-format off
    out.noalias() =
      0.5 * B
      + ((t - st) / t3) * (AB + BA  + ABA)
      - ((1 - 0.5 * t2 - ct) / t4) * (AAB + BAA - 3 * ABA)
      - 0.5*((1 - 0.5*t2 - ct)/t4 - 3*(t - st - t3 / 6) / t5) * (ABAA + AABA);
    // clang-format on
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian SE3Tangent<S, Options>::left_jacobian(
    S tolerance) const
{
  const auto& a = m_data.template head<3>();

  Jacobian jac;
  jac.template topLeftCorner<3, 3>()
      = SO3Tangent<S>(a).left_jacobian(tolerance);
  jac.template topRightCorner<3, 3>().setZero();
  jac.template bottomLeftCorner<3, 3>() = compute_q(m_data, tolerance);
  jac.template bottomRightCorner<3, 3>() = jac.template topLeftCorner<3, 3>();
  return jac;
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian
SE3Tangent<S, Options>::space_jacobian(S tolerance) const
{
  return left_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian
SE3Tangent<S, Options>::right_jacobian(S tolerance) const
{
  return SE3Tangent<S, Options>(-m_data).left_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian SE3Tangent<S, Options>::body_jacobian(
    S tolerance) const
{
  return right_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian
SE3Tangent<S, Options>::left_jacobian_inverse(S tolerance) const
{
  const auto& a = m_data.template head<3>();
  const Eigen::Matrix<S, 3, 3> jac_inv
      = SO3Tangent<S>(a).left_jacobian_inverse(tolerance);
  const auto& Q = compute_q(m_data, tolerance);

  Jacobian jac;
  jac.template topLeftCorner<3, 3>() = jac_inv;
  jac.template topRightCorner<3, 3>().setZero();
  jac.template bottomLeftCorner<3, 3>().noalias() = -jac_inv * Q * jac_inv;
  jac.template bottomRightCorner<3, 3>() = jac_inv;
  return jac;
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::Jacobian
SE3Tangent<S, Options>::right_jacobian_inverse(S tolerance) const
{
  return SE3Tangent<S, Options>(-m_data).left_jacobian_inverse(tolerance);
}

//==============================================================================
template <typename S, int Options>
auto SE3Tangent<S, Options>::angular() const
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Tangent<S, Options>::mutable_angular()
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Tangent<S, Options>::linear() const
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Tangent<S, Options>::mutable_linear()
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
typename SE3Tangent<S, Options>::TangentData& SE3Tangent<S, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SE3Tangent<S, Options>::TangentData&
SE3Tangent<S, Options>::vector() const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
SE3Tangent<S, Options> SE3Tangent<S, Options>::Random()
{
  SE3Tangent<S, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename S, int Options>
SE3Cotangent<S, Options>::SE3Cotangent() : m_data(CotangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Cotangent<S, Options>::SE3Cotangent(const SE3Cotangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SE3Cotangent<S, Options>::SE3Cotangent(SE3Cotangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::torque() const
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::mutable_torque()
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::force() const
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::mutable_force()
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::angular() const
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::mutable_angular()
{
  return m_data.template head<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::linear() const
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
auto SE3Cotangent<S, Options>::mutable_linear()
{
  return m_data.template tail<3>();
}

//==============================================================================
template <typename S, int Options>
typename SE3Cotangent<S, Options>::CotangentData&
SE3Cotangent<S, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SE3Cotangent<S, Options>::CotangentData&
SE3Cotangent<S, Options>::vector() const
{
  return m_data;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S, int Options>
Map<dart::math::SE3<S, Options>, Options>::Map(S* data)
  : m_orientation(data), m_position(data + dart::math::SO3<S>::RepDim)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
Map<const dart::math::SE3<S, Options>, Options>::Map(const S* data)
  : m_orientation(data), m_position(data + dart::math::SO3<S>::RepDim)
{
  // Do nothing
}

} // namespace Eigen

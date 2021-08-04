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

#include "dart/math/Random.hpp"
#include "dart/math/lie_group/lie_group.hpp"
#include "dart/math/lie_group/so3.hpp"

namespace dart::math {

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::operator=(const SO3Base<Derived>& o)
{
  coeffs() = o.coeffs();
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& SO3Base<Derived>::operator=(const LieGroupBase<OtherDerived>& other)
{
  coeffs() = other.coeffs();
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename EigenDerived>
Derived& SO3Base<Derived>::operator=(
    const Eigen::MatrixBase<EigenDerived>& other)
{
  coeffs() = other;
  return derived();
}

//==============================================================================
template <typename Derived>
Derived& SO3Base<Derived>::operator=(SO3Base&& other)
{
  coeffs() = std::move(other.coeffs());
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename OtherDerived>
Derived& SO3Base<Derived>::operator=(LieGroupBase<OtherDerived>&& other)
{
  coeffs() = std::move(other.coeffs());
  return derived();
}

//==============================================================================
template <typename Derived>
template <typename EigenDerived>
Derived& SO3Base<Derived>::operator=(Eigen::MatrixBase<EigenDerived>&& other)
{
  coeffs() = std::move(other);
  return derived();
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options> SO3<S, Options>::Identity()
{
  SO3<S, Options> out;
  out.set_identity();
  return out;
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options> SO3<S, Options>::Random()
{
  SO3<S, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3() : m_data(LieGroupData::Identity())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(const QuaternionType& quat) : m_data(quat)
{
  normalize();
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>::SO3(QuaternionType&& quat) : m_data(std::move(quat))
{
  normalize();
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SO3<S, Options>& SO3<S, Options>::operator=(
    const Eigen::MatrixBase<Derived>& coeffs)
{
  m_data = coeffs;
  normalize();
  return *this;
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options> SO3<S, Options>::operator*(const SO3<S, Options>& other) const
{
  return SO3<S, Options>(m_data * other.m_data);
}

//==============================================================================
template <typename S, int Options>
R3<S> SO3<S, Options>::operator*(const R3<S>& other) const
{
  return R3<S>(m_data._transformVector(other.vector()));
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options> SO3<S, Options>::operator*(
    const SO3Algebra<S, Options>& dx) const
{
  return SO3Algebra<S, Options>(matrix() * dx.matrix());
}

//==============================================================================
template <typename S, int Options>
Eigen::Matrix<S, 3, 1> SO3<S, Options>::euler_angles(
    int axis1, int axis2, int axis3) const
{
  return euler_angles_intrinsic(axis1, axis2, axis3);
}

//==============================================================================
template <typename S, int Options>
Eigen::Matrix<S, 3, 1> SO3<S, Options>::euler_angles_intrinsic(
    int axis1, int axis2, int axis3) const
{
  return m_data.toRotationMatrix().eulerAngles(axis1, axis2, axis3);
}

//==============================================================================
template <typename S, int Options>
Eigen::Matrix<S, 3, 1> SO3<S, Options>::euler_angles_extrinsic(
    int axis1, int axis2, int axis3) const
{
  return m_data.toRotationMatrix().eulerAngles(axis3, axis2, axis1).reverse();
}

//==============================================================================
template <typename S, int Options>
Eigen::Matrix<S, 3, 1> SO3<S, Options>::rpy() const
{
  return euler_angles_extrinsic(0, 1, 2);
}

//==============================================================================
template <typename S, int Options>
void SO3<S, Options>::set_identity()
{
  m_data.setIdentity();
}

//==============================================================================
template <typename S, int Options>
void SO3<S, Options>::set_random()
{
  m_data = Random::uniformUnitQuaternion<S>();
  normalize();
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options> SO3<S, Options>::inverse() const
{
  return SO3<S, Options>(m_data.inverse());
}

//==============================================================================
template <typename S, int Options>
SO3<S, Options>& SO3<S, Options>::inverse_in_place()
{
  m_data = m_data.inverse();
  return *this;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Tangent SO3<S, Options>::log(
    Jacobian* jacobian, S tolerance) const
{
  const S theta = 2 * std::acos(m_data.w());
  DART_ASSERT(!std::isnan(theta));

  typename Tangent::TangentData vec;
  if (theta < tolerance) {
    vec.noalias() = 2 * m_data.vec();
  } else {
    DART_ASSERT(std::sin(0.5 * theta));
    vec.noalias() = theta * m_data.vec() / std::sin(0.5 * theta);
  }

  const auto out = Tangent(vec);

  if (jacobian) {
    (*jacobian) = out.left_jacobian(tolerance);
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Tangent SO3<S, Options>::ad(const Tangent& V) const
{
  return Tangent(rotation() * V.vector());
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Jacobian SO3<S, Options>::ad_matrix() const
{
  return rotation();
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Matrix SO3<S, Options>::matrix() const
{
  return m_data.toRotationMatrix();
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Rotation SO3<S, Options>::rotation() const
{
  return matrix();
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Translation SO3<S, Options>::translation() const
{
  return Translation::Zero();
}

//==============================================================================
template <typename S, int Options>
typename SO3<S, Options>::Transformation SO3<S, Options>::transformation() const
{
  Transformation out = Transformation::Identity();
  out.linear() = rotation();
  return out;
}

//==============================================================================
template <typename S, int Options>
const typename SO3<S, Options>::LieGroupData& SO3<S, Options>::quaternion()
    const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const auto& SO3<S, Options>::coeffs() const
{
  return m_data.coeffs();
}

//==============================================================================
template <typename S, int Options>
void SO3<S, Options>::normalize()
{
  if (m_data.w() < 0) {
    m_data.coeffs() *= -1;
  }
  m_data.normalize();
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options>::SO3Algebra() : m_matrix(LieAlgebraData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options>::SO3Algebra(const SO3Algebra& other)
  : m_matrix(other.m_matrix)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options>::SO3Algebra(SO3Algebra&& other)
  : m_matrix(std::move(other.m_matrix))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SO3Algebra<S, Options>::SO3Algebra(const Eigen::MatrixBase<Derived>& matrix)
  : m_matrix(matrix)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SO3Algebra<S, Options>::SO3Algebra(Eigen::MatrixBase<Derived>&& matrix)
  : m_matrix(std::move(matrix))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options> SO3Algebra<S, Options>::operator/(S scalar) const
{
  return SO3Algebra<S, Options>(m_matrix / scalar);
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options> SO3Algebra<S, Options>::vee() const
{
  return SO3Tangent<S, Options>(
      Eigen::Matrix<S, 3, 1>(m_matrix(2, 1), m_matrix(0, 2), m_matrix(1, 0)));
}

//==============================================================================
template <typename S, int Options>
typename SO3Algebra<S, Options>::LieAlgebraData&
SO3Algebra<S, Options>::mutable_matrix()
{
  return m_matrix;
}

//==============================================================================
template <typename S, int Options>
const typename SO3Algebra<S, Options>::LieAlgebraData&
SO3Algebra<S, Options>::matrix() const
{
  return m_matrix;
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options>::SO3Tangent() : m_data(TangentData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options>::SO3Tangent(const SO3Tangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options>::SO3Tangent(SO3Tangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SO3Tangent<S, Options>::SO3Tangent(const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
SO3Tangent<S, Options>::SO3Tangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options>& SO3Tangent<S, Options>::operator=(
    const SO3Tangent<S, Options>& other)
{
  m_data = other.m_data;
  return *this;
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options>& SO3Tangent<S, Options>::operator=(
    SO3Tangent<S, Options>&& other)
{
  m_data = std::move(other.m_data);
  return *this;
}

//==============================================================================
template <typename S, int Options>
S SO3Tangent<S, Options>::operator[](int index) const
{
  return m_data[index];
}

//==============================================================================
template <typename S, int Options>
S& SO3Tangent<S, Options>::operator[](int index)
{
  return m_data[index];
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options> SO3Tangent<S, Options>::operator-() const
{
  return SO3Tangent<S, Options>(-m_data);
}

//==============================================================================
template <typename S, int Options>
S SO3Tangent<S, Options>::operator*(const SO3Cotangent<S, Options>& v) const
{
  return m_data.dot(v.vector());
}

//==============================================================================
template <typename S, int Options>
void SO3Tangent<S, Options>::set_zero()
{
  m_data.setZero();
}

//==============================================================================
template <typename S, int Options>
void SO3Tangent<S, Options>::set_random()
{
  const SO3<S, Options> R = SO3<S, Options>::Random();
  *this = R.log();
}

//==============================================================================
template <typename S, int Options>
SO3Algebra<S, Options> SO3Tangent<S, Options>::hat() const
{
  return SO3Algebra<S, Options>(skew(vector()));
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::LieGroup SO3Tangent<S, Options>::exp(
    Jacobian* jacobian, S tolerance) const
{
  const S theta = m_data.norm();
  typename LieGroup::LieGroupData quat;
  if (theta < tolerance) {
    const TangentData vec = 0.5 * m_data;
    quat = QuaternionType(1, vec[0], vec[1], vec[2]);
  } else {
    const TangentData vec = std::sin(0.5 * theta) * (m_data / theta);
    quat = QuaternionType(std::cos(0.5 * theta), vec[0], vec[1], vec[2]);
  }

  LieGroup out(quat);

  if (jacobian) {
    (*jacobian) = right_jacobian();
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian SO3Tangent<S, Options>::left_jacobian(
    S tolerance) const
{
  Jacobian jac;

  const S t = m_data.norm();
  const auto& A = hat().matrix();
  if (t < tolerance) {
    jac.noalias() = Jacobian::Identity() + 0.5 * A;
  } else {
    const S t2 = t * t;
    const S t3 = t2 * t;
    const S st = std::sin(t);
    const S ct = std::cos(t);
    // clang-format off
    jac.noalias() = Jacobian::Identity()
        + ((1 - ct) / t2) * A
        + ((t - st) / t3) * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::space_jacobian(S tolerance) const
{
  return left_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::right_jacobian(S tolerance) const
{
  return SO3Tangent<S, Options>(-m_data).left_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian SO3Tangent<S, Options>::body_jacobian(
    S tolerance) const
{
  return right_jacobian(tolerance);
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::left_jacobian_inverse(S tolerance) const
{
  Jacobian jac;

  const S theta = m_data.norm();
  const auto& A = hat().matrix();
  if (theta < tolerance) {
    jac.noalias() = Jacobian::Identity() + 0.5 * A;
  } else {
    const S theta2 = theta * theta;
    const S sin_theta = std::sin(theta);
    const S cos_theta = std::cos(theta);
    // clang-format off
    jac.noalias() = Jacobian::Identity()
        - 0.5 * A
        + (1 / theta2 - (1 + cos_theta) / (2 * theta * sin_theta)) * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::right_jacobian_inverse(S tolerance) const
{
  return SO3Tangent<S, Options>(-m_data).left_jacobian_inverse(tolerance);
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::left_jacobian_time_derivative(
    const math::MatrixBase<Derived>& dq, S tolerance) const
{
  Jacobian jac;

  const S t = m_data.norm();
  const LieAlgebra A = hat();
  const LieAlgebra dA = skew(dq);
  const S q_dot_dq = m_data.dot(dq);
  const S sin_t = std::sin(t);
  const S cos_t = std::cos(t);

  if (t < tolerance) {
    // clang-format off
    jac = -0.5 * dA
      + (S(1) / S(6)) * (A * dA + dA * A)
      + (S(1) / S(12)) * q_dot_dq * A;
    // clang-format on
  } else {
    const S t2 = t * t;
    const S t3 = t2 * t;
    const S t4 = t3 * t;
    const S t5 = t4 * t;
    // clang-format off
    jac = -((1 - cos_t) / t2) * dA
        + ((t - sin_t) / t3) * (A * dA + dA * A)
        - ((t * sin_t + 2 * cos_t - 2) / t4) * q_dot_dq * A
        + ((3 * sin_t - t * cos_t - 2 * t) / t5) * q_dot_dq * A * A;
    // clang-format on
  }

  return jac;
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::right_jacobian_time_derivative(
    const math::MatrixBase<Derived>& dq, S tolerance) const
{
  return left_jacobian_time_derivative(dq, tolerance).transpose();
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::right_jacobian_time_derivative(
    int index, S tolerance) const
{
  return left_jacobian_time_derivative(index, tolerance).transpose();
}

//==============================================================================
template <typename S, int Options>
std::array<typename SO3Tangent<S, Options>::Jacobian, 3>
SO3Tangent<S, Options>::right_jacobian_time_derivative(S tolerance) const
{
  std::array<typename SO3Tangent<S, Options>::Jacobian, 3> out;
  for (auto i = 0; i < 3; ++i) {
    out[i] = right_jacobian_time_derivative(i, tolerance).transpose();
  }
  return out;
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian
SO3Tangent<S, Options>::left_jacobian_time_derivative(
    int index, S tolerance) const
{
  math::Vector3<S> dq = math::Vector3<S>::Zero();
  dq[index] = 1;
  return left_jacobian_time_derivative(dq, tolerance);
}

//==============================================================================
template <typename S, int Options>
std::array<typename SO3Tangent<S, Options>::Jacobian, 3>
SO3Tangent<S, Options>::left_jacobian_time_derivative(S tolerance) const
{
  std::array<typename SO3Tangent<S, Options>::Jacobian, 3> out;

  math::Vector3<S> dq = math::Vector3<S>::Zero();
  for (auto i = 0; i < 3; ++i) {
    dq.setZero();
    dq[i] = 1;
    out[i] = left_jacobian_time_derivative(dq, tolerance);
  }

  return out;
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options> SO3Tangent<S, Options>::ad(const SO3Tangent& other) const
{
  return SO3Tangent<S, Options>(m_data.cross(other.m_data));
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::Jacobian SO3Tangent<S, Options>::ad_matrix()
    const
{
  return hat();
}

//==============================================================================
template <typename S, int Options>
typename SO3Tangent<S, Options>::TangentData& SO3Tangent<S, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SO3Tangent<S, Options>::TangentData&
SO3Tangent<S, Options>::vector() const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
SO3Tangent<S, Options> SO3Tangent<S, Options>::Random()
{
  SO3Tangent<S, Options> out;
  out.set_random();
  return out;
}

//==============================================================================
template <typename S, int Options>
typename SO3Cotangent<S, Options>::CotangentData&
SO3Cotangent<S, Options>::vector()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
const typename SO3Cotangent<S, Options>::CotangentData&
SO3Cotangent<S, Options>::vector() const
{
  return m_data;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S, int Options>
Map<dart::math::SO3<S, Options>, Options>::Map(S* data) : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
const Map<Eigen::Quaternion<S, Options>, Options>&
Map<dart::math::SO3<S, Options>, Options>::quaternion() const
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
Map<Eigen::Quaternion<S, Options>, Options>&
Map<dart::math::SO3<S, Options>, Options>::quaternion()
{
  return m_data;
}

//==============================================================================
template <typename S, int Options>
Map<const dart::math::SO3<S, Options>, Options>::Map(const S* data)
  : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
const Map<Eigen::Quaternion<S, Options>, Options>&
Map<const dart::math::SO3<S, Options>, Options>::quaternion() const
{
  return m_data;
}

} // namespace Eigen

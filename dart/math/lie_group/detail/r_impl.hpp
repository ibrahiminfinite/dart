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

#include "dart/math/lie_group/r.hpp"

namespace dart::math {

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options> R<S, N, Options>::Zero()
{
  return R<S, N, Options>(Eigen::Matrix<S, N, 1, Options>::Zero());
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options> R<S, N, Options>::Identity()
{
  return Zero();
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options> R<S, N, Options>::Random()
{
  return R<S, N, Options>(Eigen::Matrix<S, N, 1, Options>::Random());
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options>::R() : m_vector(LieGroupData::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options>::R(const R& other) : m_vector(other.m_vector)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options>::R(R&& other) : m_vector(std::move(other.m_vector))
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
template <typename Derived>
R<S, N, Options>::R(const math::MatrixBase<Derived>& vec) : m_vector(vec)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
template <typename Derived>
R<S, N, Options>::R(math::MatrixBase<Derived>&& vec) : m_vector(std::move(vec))
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options>& R<S, N, Options>::operator=(const R<S, N, Options>& other)
{
  m_vector = other.m_vector;
  return *this;
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options>& R<S, N, Options>::operator=(R<S, N, Options>&& other)
{
  m_vector = std::move(other.m_vector);
  return *this;
}

//==============================================================================
template <typename S, int N, int Options>
template <typename Derived>
R<S, N, Options>& R<S, N, Options>::operator=(
    const Eigen::MatrixBase<Derived>& matrix)
{
  m_vector = matrix;
  return *this;
}

//==============================================================================
template <typename S, int N, int Options>
template <typename Derived>
R<S, N, Options>& R<S, N, Options>::operator=(
    Eigen::MatrixBase<Derived>&& matrix)
{
  m_vector = std::move(matrix);
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options> R<S, N, Options>::operator-() const
{
  return R<S, N, Options>(-m_vector);
}

//==============================================================================
template <typename S, int N, int Options>
R<S, N, Options> R<S, N, Options>::operator+(
    const R<S, N, Options>& other) const
{
  return R<S, N, Options>(m_vector + other.m_vector);
}

//==============================================================================
template <typename S, int N, int Options>
void R<S, N, Options>::set_identity()
{
  m_vector.setZero();
}

//==============================================================================
template <typename S, int N, int Options>
void R<S, N, Options>::set_random()
{
  m_vector.setRandom();
}

//==============================================================================
template <typename S, int N, int Options>
constexpr int R<S, N, Options>::dimension() const
{
  return GroupDim;
}

//==============================================================================
template <typename S, int N, int Options>
typename R<S, N, Options>::LieGroupData& R<S, N, Options>::vector()
{
  return m_vector;
}

//==============================================================================
template <typename S, int N, int Options>
const typename R<S, N, Options>::LieGroupData& R<S, N, Options>::vector() const
{
  return m_vector;
}

//==============================================================================
template <typename S, int Options>
R<S, Eigen::Dynamic, Options>::R() : m_vector(LieGroupData())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
R<S, Eigen::Dynamic, Options>::R(int dim) : m_vector(LieGroupData::Zero(dim))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
R<S, Eigen::Dynamic, Options>::R(const R& other) : m_vector(other.m_vector)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
R<S, Eigen::Dynamic, Options>::R(R&& other)
  : m_vector(std::move(other.m_vector))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
R<S, Eigen::Dynamic, Options>::R(const math::MatrixBase<Derived>& vector)
  : m_vector(vector)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
R<S, Eigen::Dynamic, Options>::R(math::MatrixBase<Derived>&& vector)
  : m_vector(std::move(vector))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
R<S, Eigen::Dynamic, Options>& R<S, Eigen::Dynamic, Options>::operator=(
    const Eigen::MatrixBase<Derived>& matrix)
{
  m_vector = matrix;
}

//==============================================================================
template <typename S, int Options>
template <typename Derived>
R<S, Eigen::Dynamic, Options>& R<S, Eigen::Dynamic, Options>::operator=(
    Eigen::MatrixBase<Derived>&& matrix)
{
  m_vector = std::move(matrix);
}

//==============================================================================
template <typename S, int Options>
const R<S, Eigen::Dynamic, Options>& R<S, Eigen::Dynamic, Options>::operator+()
    const
{
  return *this;
}

//==============================================================================
template <typename S, int Options>
R<S, Eigen::Dynamic, Options> R<S, Eigen::Dynamic, Options>::operator-() const
{
  return R<S, Eigen::Dynamic, Options>(m_vector);
}

//==============================================================================
template <typename S, int Options>
int R<S, Eigen::Dynamic, Options>::dimension() const
{
  return m_vector.size();
}

//==============================================================================
template <typename S, int Options>
const typename R<S, Eigen::Dynamic, Options>::LieGroupData&
R<S, Eigen::Dynamic, Options>::vector() const
{
  return m_vector;
}

//==============================================================================
template <typename S, int Options>
typename R<S, Eigen::Dynamic, Options>::LieGroupData&
R<S, Eigen::Dynamic, Options>::mutable_vector()
{
  return m_vector;
}

//==============================================================================
template <typename S, int Dim, int Options>
RTangent<S, Dim, Options>::RTangent() : m_data(Data::Zero())
{
  // Do nothing
}

//==============================================================================
template <typename S, int Dim, int Options>
RTangent<S, Dim, Options>::RTangent(const RTangent& other)
  : m_data(other.m_data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Dim, int Options>
RTangent<S, Dim, Options>::RTangent(RTangent&& other)
  : m_data(std::move(other.m_data))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Dim, int Options>
template <typename Derived>
RTangent<S, Dim, Options>::RTangent(const Eigen::MatrixBase<Derived>& coeffs)
  : m_data(coeffs)
{
  // Do nothing
}

//==============================================================================
template <typename S, int Dim, int Options>
template <typename Derived>
RTangent<S, Dim, Options>::RTangent(Eigen::MatrixBase<Derived>&& coeffs)
  : m_data(std::move(coeffs))
{
  // Do nothing
}

//==============================================================================
template <typename S, int Dim, int Options>
S RTangent<S, Dim, Options>::operator*(const RCotangent<S, Dim>& cotan) const
{
  return m_data.dot(cotan.m_data);
}

//==============================================================================
template <typename S, int Dim, int Options>
typename RTangent<S, Dim, Options>::LieAlgebra RTangent<S, Dim, Options>::hat()
    const
{
  LieAlgebra out = LieAlgebra::Zero();
  out.template topRightCorner<Dim + 1, 1>() = m_data;
  return out;
}

} // namespace dart::math

namespace Eigen {

//==============================================================================
template <typename S, int N, int Options>
Map<dart::math::R<S, N, Options>, Options>::Map(S* data) : m_data(data)
{
  // Do nothing
}

//==============================================================================
template <typename S, int N, int Options>
Map<const dart::math::R<S, N, Options>, Options>::Map(S* data) : m_data(data)
{
  // Do nothing
}

} // namespace Eigen

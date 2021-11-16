/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "dart/common/macro.hpp"
#include "dart/math/field/constant_vector_field3.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar>
ConstantVectorField3<Scalar>::ConstantVectorField3(const Vector3<Scalar>& value)
  : m_value(value)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Vector3<Scalar> ConstantVectorField3<Scalar>::sample(
    const Vector3<Scalar>& x) const
{
  DART_UNUSED(x);
  return m_value;
}

//==============================================================================
template <typename Scalar>
std::function<Vector3<Scalar>(const Vector3<Scalar>&)>
ConstantVectorField3<Scalar>::sampler() const
{
  return [this](const Vector3<Scalar>&) -> Vector3<Scalar> {
    return m_value;
  };
}

//==============================================================================
template <typename Scalar>
typename ConstantVectorField3<Scalar>::Builder
ConstantVectorField3<Scalar>::builder()
{
  return Builder();
}

//==============================================================================
template <typename Scalar>
typename ConstantVectorField3<Scalar>::Builder&
ConstantVectorField3<Scalar>::Builder::with_value(const Vector3<Scalar>& value)
{
  m_value = value;
  return *this;
}

//==============================================================================
template <typename Scalar>
ConstantVectorField3<Scalar> ConstantVectorField3<Scalar>::Builder::build()
    const
{
  return ConstantVectorField3(m_value);
}

//==============================================================================
template <typename Scalar>
ConstantVectorField3Ptr<Scalar>
ConstantVectorField3<Scalar>::Builder::build_shared() const
{
  return std::shared_ptr<ConstantVectorField3>(
      new ConstantVectorField3(m_value), [](ConstantVectorField3* obj) {
        delete obj;
      });
}

} // namespace dart::math

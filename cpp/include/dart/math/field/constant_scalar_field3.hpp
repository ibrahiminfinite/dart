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

#include "dart/math/field/scalar_field3.hpp"

namespace dart::math {

template <typename Scalar_>
class ConstantScalarField3 final : public ScalarField3<Scalar_>
{
public:
  using Scalar = Scalar_;

  class Builder;

  explicit ConstantScalarField3(Scalar value);

  Scalar sample(const Vector3<Scalar>& x) const override;

  std::function<Scalar(const Vector3<Scalar>&)> sampler() const override;

  static Builder builder();

private:
  Scalar m_value{0};
};

/// Shared pointer for the ConstantScalarField3 type.
DART_DEFINE_CLASS_POINTERS_T1(ConstantScalarField3);

// extern template class ConstantScalarField3<double>;

template <typename Scalar_>
class ConstantScalarField3<Scalar_>::Builder final
{
public:
  using Scalar = Scalar_;

  Builder& with_value(Scalar value);

  ConstantScalarField3 build() const;

  ConstantScalarField3Ptr<Scalar> build_shared() const;

private:
  Scalar m_value{0};
};

} // namespace dart::math

#include "dart/math/field/detail/constant_scalar_field3_impl.hpp"

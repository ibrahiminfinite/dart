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

#include <gtest/gtest.h>

#include "dart/math/field/constant_scalar_field3.hpp"
#include "dart/math/field/constant_vector_field3.hpp"
#include "dart/test/math/GTestUtils.hpp"

using namespace dart;
using namespace math;

//==============================================================================
template <typename T>
struct Field3Test : public testing::Test
{
  using Type = T;

  constexpr Type tol() const
  {
    if constexpr (std::is_same_v<Type, double>) {
      return 1e-9;
    } else {
      return 1e-4;
    }
  }
};

//==============================================================================
using Types = dart::test::FloatingTypes;
TYPED_TEST_SUITE(Field3Test, Types);

//==============================================================================
TYPED_TEST(Field3Test, ConstantScalarField3)
{
  using S = typename TestFixture::Type;

  auto field1
      = math::ConstantScalarField3<S>::builder().with_value(1).build_shared();
  EXPECT_DOUBLE_EQ(field1->sample(math::Vector3<S>::Random()), 1);
}

//==============================================================================
TYPED_TEST(Field3Test, ConstantVectorField3)
{
  using S = typename TestFixture::Type;

  auto field1 = math::ConstantVectorField3<S>::builder()
                    .with_value({1, 0, 0})
                    .build_shared();
  EXPECT_TRUE(field1->sample(math::Vector3<S>::Random())
                  .isApprox(math::Vector3<S>::UnitX()));
}

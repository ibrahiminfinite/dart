/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <gtest/gtest.h>

#include "dart/math/math.hpp"
#include "dart/test/math/GTestUtils.hpp"

using namespace dart;
using namespace math;

//==============================================================================
template <typename T>
struct SO3Test : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(SO3Test, Types);

//==============================================================================
TYPED_TEST(SO3Test, StaticProperties)
{
  using Scalar = typename TestFixture::Type;

  EXPECT_EQ(SO3<Scalar>::GroupDim, 3);
}

//==============================================================================
TYPED_TEST(SO3Test, Constructors)
{
  using Scalar = typename TestFixture::Type;

  // Default constructor
  SO3<Scalar> a;

  // Copy constructor
  SO3<Scalar> b = a;

  // Move constructor
  SO3<Scalar> c = std::move(a);

  // From quaternions
  auto d = SO3<Scalar>(Eigen::Quaternion<Scalar>());
  auto q_b = Eigen::Quaternion<Scalar>();
  auto e = SO3<Scalar>(std::move(q_b));

  // Using static functions
  SO3<Scalar> f = SO3<Scalar>::Identity();
  SO3<Scalar> g = SO3<Scalar>::Random();

  DART_UNUSED(b, c, f, g);
}

//==============================================================================
TYPED_TEST(SO3Test, Inverse)
{
  using Scalar = typename TestFixture::Type;

  SO3<Scalar> a;
  SO3<Scalar> b;
  SO3<Scalar> c;

  b = a.inverse();
  c = a.inverse() * b;
  c = a * b.inverse();
  // c = a.inverse().eval() * b;
  // c = a * b.inverse().eval();
  // c.noalias() = a * b.inverse();
  // c.noalias() = a.inverse() * b;
}

//==============================================================================
TYPED_TEST(SO3Test, Map)
{
  using Scalar = typename TestFixture::Type;
  std::array<Scalar, 4> data{1, 2, 3, 4};

  SO3Map<Scalar> a(data.data());
  auto q = a.quaternion();
  EXPECT_S_EQ(q.x(), data[0]);
  EXPECT_S_EQ(q.y(), data[1]);
  EXPECT_S_EQ(q.z(), data[2]);
  EXPECT_S_EQ(q.w(), data[3]);

  q.x() = -1;
  q.y() = -2;
  q.z() = -3;
  q.w() = -4;
  EXPECT_S_EQ(data[0], -1);
  EXPECT_S_EQ(data[1], -2);
  EXPECT_S_EQ(data[2], -3);
  EXPECT_S_EQ(data[3], -4);

  ConstSO3Map<Scalar> b(data.data());
  q = a.quaternion();
  EXPECT_S_EQ(q.x(), data[0]);
  EXPECT_S_EQ(q.y(), data[1]);
  EXPECT_S_EQ(q.z(), data[2]);
  EXPECT_S_EQ(q.w(), data[3]);
}

//==============================================================================
TYPED_TEST(SO3Test, Jacobians)
{
  using Scalar = typename TestFixture::Type;
  const Scalar eps = test::eps_for_diff<Scalar>();

  for (auto i = 0; i < 100; ++i) {
    const SO3Tangent<Scalar> q = SO3<Scalar>::Random().log();
    Eigen::Matrix<Scalar, 3, 3> jac_numeric;
    for (int j = 0; j < 3; ++j) {
      SO3Tangent<Scalar> q_a = q;
      q_a[j] -= Scalar(0.5) * eps;

      SO3Tangent<Scalar> q_b = q;
      q_b[j] += Scalar(0.5) * eps;

      const SO3<Scalar> R_a = exp(q_a);
      const SO3<Scalar> R_b = exp(q_b);
      const SO3<Scalar> dR_left = R_b * R_a.inverse();
      const SO3Tangent<Scalar> dr = log(dR_left);
      const SO3Algebra<Scalar> dr_dt = dr.hat() / eps;
      jac_numeric.col(j) = dr_dt.vee().vector();
    }
    EXPECT_TRUE(test::equals(jac_numeric, q.left_jacobian()));
  }
}

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
struct SE3Test : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(SE3Test, Types);

//==============================================================================
TYPED_TEST(SE3Test, StaticProperties)
{
  using S = typename TestFixture::Type;

  EXPECT_EQ(SE3<S>::SpaceDim, 3);
  EXPECT_EQ(SE3<S>::GroupDim, 6);
  EXPECT_EQ(SE3<S>::MatrixDim, 4);
}

//==============================================================================
TYPED_TEST(SE3Test, Constructors)
{
  using S = typename TestFixture::Type;

  // Default constructor
  SE3<S> a;

  // Copy constructor
  SE3<S> b = a;

  // Move constructor
  SE3<S> c = std::move(a);

  // From quaternions
  SE3<S> d(Eigen::Quaternion<S>::Identity(), Eigen::Matrix<S, 3, 1>::Zero());
  Eigen::Quaternion<S> quat = Eigen::Quaternion<S>::Identity();
  Eigen::Matrix<S, 3, 1> vec = Eigen::Matrix<S, 3, 1>::Zero();
  SE3<S> e(std::move(quat), std::move(vec));

  // Using static functions
  SE3<S> f = SE3<S>::Identity();
  SE3<S> g = SE3<S>::Random();

  DART_UNUSED(b, c, f, g);
}

//==============================================================================
TYPED_TEST(SE3Test, Jacobians)
{
  using S = typename TestFixture::Type;
  const S eps = test::eps_for_diff<S>();

  for (auto i = 0; i < 100; ++i) {
    const SE3Tangent<S> q = SE3<S>::Random().log();
    Eigen::Matrix<S, 6, 6> jac_numeric;
    for (int j = 0; j < 6; ++j) {
      SE3Tangent<S> q_a = q;
      q_a[j] -= S(0.5) * eps;

      SE3Tangent<S> q_b = q;
      q_b[j] += S(0.5) * eps;

      const SE3<S> T_a = exp(q_a);
      const SE3<S> T_b = exp(q_b);
      const SE3<S> dT_left = T_b * T_a.inverse();
      const SE3Tangent<S> dt = log(dT_left);
      const SE3Algebra<S> dt_dt = dt.hat() / eps;
      jac_numeric.col(j) = dt_dt.vee().vector();
    }
    EXPECT_TRUE(test::equals(jac_numeric, q.left_jacobian()));
  }
}

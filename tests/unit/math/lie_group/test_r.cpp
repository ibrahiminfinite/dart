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

using namespace dart;
using namespace math;

//==============================================================================
template <typename T>
struct TTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(TTest, Types);

//==============================================================================
template <typename Scalar, int Dim>
void test_constructors_fixed_size()
{
  using RN = R<Scalar, Dim>;

  // Default constructor
  RN a;
  EXPECT_EQ(a.dimension(), Dim);

  // Copy constructor
  RN b = a;

  // Move constructor
  RN c = std::move(a);

  // From Eigen vectors
  RN d(Eigen::Matrix<Scalar, Dim, 1>::Random());
  Eigen::Matrix<Scalar, Dim, 1> vec = Eigen::Matrix<Scalar, Dim, 1>::Random();
  RN e = std::move(vec);
  EXPECT_TRUE(e.vector().isApprox(vec));

  // Using static functions
  RN f = RN::Zero();
  RN g = RN::Identity();
  RN h = RN::Random();
}

//==============================================================================
TYPED_TEST(TTest, Constructors)
{
  using Scalar = typename TestFixture::Type;

  // Fixed size
  {
    // Default constructor
    RX<Scalar> a;
    EXPECT_EQ(a.dimension(), 0);

    // Copy constructor
    RX<Scalar> b = a;

    // Move constructor
    RX<Scalar> c = std::move(a);
  }

  // Dynamic size
  test_constructors_fixed_size<Scalar, 0>();
  test_constructors_fixed_size<Scalar, 1>();
  test_constructors_fixed_size<Scalar, 2>();
  test_constructors_fixed_size<Scalar, 3>();
  test_constructors_fixed_size<Scalar, 4>();
  test_constructors_fixed_size<Scalar, 5>();
  test_constructors_fixed_size<Scalar, 6>();
}

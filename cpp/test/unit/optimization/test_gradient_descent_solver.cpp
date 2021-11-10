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

#include "dart/common/macro.hpp"
#include "dart/optimization/all.hpp"

using namespace dart;

template <typename Scalar>
class SampleObjectFunction : public optimizatiion::Function<Scalar>
{
public:
  using Base = optimizatiion::Function<Scalar>;
  using typename Base::VectorX;

  Scalar compute_value(const VectorX& x) override
  {
    return std::sqrt(x[1]);
  }

protected:
};

void foo1(Eigen::Ref<Eigen::VectorXd> x)
{
  x = 2 * x;
}

void foo2(Eigen::Map<Eigen::VectorXd> x)
{
  x = 2 * x;
}

template <typename Derived>
void foo3(Eigen::MatrixBase<Derived>& x)
{
  x = 2 * x;
}

//==============================================================================
template <typename T>
struct GradientDescentSolverTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<double>;

//==============================================================================
TYPED_TEST_SUITE(GradientDescentSolverTest, Types);

//==============================================================================
TYPED_TEST(GradientDescentSolverTest, Basics)
{
  using Scalar = typename TestFixture::Type;

  auto solver = optimization::GradientDescentSolver<Scalar>();
  DART_UNUSED(solver);

  double d2[2] = {0, 0};

  Eigen::VectorXd x1 = Eigen::VectorXd::Random(2);
}

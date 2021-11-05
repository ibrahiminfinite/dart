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

#include "dart/math/lcp/jacobi_lcp_solver.hpp"
#include "dart/math/lcp/lcp_problem.hpp"
#include "dart/math/lcp/pgs_lcp_solver.hpp"
#include "dart/test/math/util.hpp"

using namespace dart;

//==============================================================================
template <typename T>
struct LcpProblemTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(LcpProblemTest, Types);

//==============================================================================
TYPED_TEST(LcpProblemTest, Basics)
{
  using Scalar = typename TestFixture::Type;

  auto n = 10;

  for (auto i = 0; i < 1; ++i) {
  auto problem = math::LcpProblem<Scalar>(n);
  problem.A = test::generate_random_pd_matrix<Scalar>(n);
  problem.b.setRandom(n);
  problem.x.setZero(n);

  auto problem2 = problem;

//  problem.A << 1, 2, 2, 1;
//  problem.b << 1, -1;

  auto solver1 = math::JacobiLcpSolver<Scalar>();
  auto result1 = solver1.solve(problem);
  EXPECT_TRUE(result1);
  problem.print();

  auto solver2 = math::PgsLcpSolver<Scalar>();
  auto result2 = solver2.solve(problem2);
  EXPECT_TRUE(result2);
  problem2.print();
  }
}

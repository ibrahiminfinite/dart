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

#include "dart/math/lcp/jacobi_lcp_solver.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar>
bool JacobiLcpSolver<Scalar>::solve(LcpProblem<Scalar>& problem)
{
  using Matrix = typename LcpProblem<Scalar>::Matrix;
  using Vector = typename LcpProblem<Scalar>::Vector;

  const auto& A = problem.A;
  const auto& b = problem.b;
  auto& x = problem.x;
  const Scalar n = problem.size;

  const Vector invM = A.diagonal().cwiseInverse();
  const Vector invM_b = b.cwiseProduct(invM);
  Matrix minusN = A;
  minusN.diagonal().setZero();

  Vector minusNx;
  unsigned int iteration = 0u;
  Scalar new_x;

  x.setZero();

  while (true) {
    minusNx.noalias() = minusN * x;

    for (auto i = 0u; i < n; ++i) {
      new_x = minusNx[i] + b[i];

      if (new_x < 0) {
        x[i] = invM[i] * -new_x; // assumed M[i] is positive
      } else {
        x[i] = 0;
      }
    }

    if (++iteration > 1e+2) {
      break;
    }

    const Scalar error = problem.compute_error();
    if (error < 1e-12) {
      break;
    }
  }

  std::cout << "[DEBUG] iteration: " << iteration << std::endl;

  return true;
}

} // namespace dart::math

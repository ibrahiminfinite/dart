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

#include <iostream>

#include "dart/common/macro.hpp"
#include "dart/math/export.hpp"
#include "dart/math/lcp/type.hpp"
#include "dart/math/type.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar_>
struct LcpProblem final
{
  // Type aliases
  using Scalar = Scalar_;
  using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  /// Size of the problem
  int size;

  /// A matrix of the problem
  Matrix A;

  /// b vector of the problem
  Vector b;

  /// x vector of the problem
  Vector x;

  /// y vector of the problem
  // Vector y;

  Scalar absolute_tolerance = 1e-6;
  Scalar relative_tolerance = 1e-3;

  explicit LcpProblem(int size = 0);

  ~LcpProblem();

  Scalar compute_error() const;

  //bool satisfied() const;

  void print(
      std::ostream& os = std::cout,
      unsigned int indent = 0,
      bool list = false) const;

  template <typename ScalarOther>
  friend std::ostream& operator<<(
      std::ostream& os, const LcpProblem<ScalarOther>& object);
};

} // namespace dart::math

#include "dart/math/lcp/detail/lcp_problem_impl.hpp"

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

#include <algorithm>

#include "dart/math/lcp/lcp_problem.hpp"

namespace dart::math {

//==============================================================================
template <typename Scalar>
LcpProblem<Scalar>::LcpProblem(int size) : size(size)
{
  A.resize(size, size);
  b.resize(size);
  x.resize(size);
  //y.resize(size);
}

//==============================================================================
template <typename Scalar>
LcpProblem<Scalar>::~LcpProblem()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Scalar LcpProblem<Scalar>::compute_error() const
{
  // Compute y = A*x + b
  Vector y = b;
  y.noalias() += A * x;

//  Scalar error = 0;
//  for (auto i = 0; i < size; ++i) {
//    const Scalar e = x[i] - std::max(Scalar(0), (x[i] - y[i]));
//    error += e * e;
//  }

  const Scalar error = std::abs(x.dot(y));

  if (const Scalar norm = b.norm()) {
    return (error / norm);
  } else {
    return error;
  }
}

//==============================================================================
//template <typename Scalar>
//bool LcpProblem<Scalar>::satisfied() const
//{
//  for (auto i = 0; i < x.size(); ++i) {
//    if (x[i] < 0.0 || y[i] < 0.0)
//      return false;

//    if (x[i] != 0.0 && y[i] != 0.0)
//      return false;
//  }

//  return true;
//}

//==============================================================================
template <typename Scalar>
void LcpProblem<Scalar>::print(
    std::ostream& os, unsigned int indent, bool list) const
{
  if (indent == 0) {
    os << "[ LcpProblem ]\n";
  }
  std::string prefix0(indent, ' ');
  std::string prefix1 = prefix0;
  if (list) {
    prefix0 += "- ";
    prefix1 += "  ";
    indent += 2;
  }

  os << "dimension: " << size << "\n";
  os << "A:\n" << A << "\n";
  os << "x: " << x.transpose() << "\n";
  os << "b: " << b.transpose() << "\n";
  os << "y: " << (A*x + b).transpose() << "\n";
  //os << "satisfied: " << (satisfied() ? "true" : "false") << "\n";
}

//==============================================================================
template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const LcpProblem<Scalar>& object)
{
  object.print(os);
  return os;
}

} // namespace dart::math

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

#pragma once

#include <string>

#include "dart/math/type.hpp"
#include "dart/optimization/type.hpp"

namespace dart::optimization {

template <typename Scalar_>
class Function
{
public:
  using Scalar = Scalar_;
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using VectorXMap = Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>;

  explicit Function(const std::string& name = "") : m_name(name)
  {
    // Do nothing
  }

  void set_name(const std::string& name)
  {
    m_name = name;
  }

  const std::string& get_name() const
  {
    return m_name;
  }

  virtual Scalar compute_value_gradient(
      const VectorX& x, Eigen::Ref<VectorX> gradient) {
    compute_value(x);
    compute_gradient(x);
  }

  virtual Scalar compute_value(const VectorX& x) = 0;

  virtual Scalar compute_gradient(const VectorX& x) = 0;

protected:
  std::string m_name;
};

} // namespace dart::optimization

//#include "dart/optimization/detail/function_impl.hpp"

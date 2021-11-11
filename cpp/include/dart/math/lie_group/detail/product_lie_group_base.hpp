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

#include "dart/math/lie_group/detail/lie_group_base.hpp"

namespace dart::math {

template <typename Derived>
class ProductLieGroupBase : public LieGroupBase<Derived>
{
public:
  using Base = LieGroupBase<Derived>;
  using This = ProductLieGroupBase<Derived>;

  static constexpr int ProductSize
      = Eigen::internal::traits<Derived>::ProductSize;

  /** The dimension of the Euclidean space that the group is embedded. */
  static constexpr int SpaceDim = ::Eigen::internal::traits<Derived>::SpaceDim;

  /** The dimension (or degrees of freedom) of the Lie group. */
  static constexpr int GroupDim = ::Eigen::internal::traits<Derived>::GroupDim;

  /** The dimension of the matrix representation of the Lie group. */
  static constexpr int MatrixDim
      = ::Eigen::internal::traits<Derived>::MatrixDim;

  /** The dimension of internal parameters to represent the Lie group element.
   */
  static constexpr int DataDim = ::Eigen::internal::traits<Derived>::DataDim;

  using DataType = typename Base::DataType;

protected:
  using Base::derived;
};

} // namespace dart::math

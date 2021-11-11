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

#include <tuple>

#include "dart/math/lie_group/detail/product_lie_group_base.hpp"
#include "dart/math/lie_group/detail/util.hpp"

namespace dart::math {

template <typename Scalar, template <typename> class... T>
class ProductLieGroup;

} // namespace dart::math

namespace Eigen::internal {

//==============================================================================
template <typename Scalar_, template <typename> class... T>
struct traits<dart::math::ProductLieGroup<Scalar_, T...>>
{
  // Number of Lie groups in the product
  static constexpr int ProductSize = sizeof...(T);

  using Components = std::tuple<T<Scalar_>...>;

  template <int N>
  using Component = typename std::tuple_element_t<N, Components>;

  template <int N>
  using MapComponent = Eigen::Map<Component<N>>;

  template <int N>
  using ConstMapComponent = Eigen::Map<const Component<N>>;

  //  static constexpr std::array<int, sizeof...(T)> SpaceDimIndex
  //      = dart::math::detail::compute_indices<T<Scalar>::SpaceDim...>();

  using Scalar = Scalar_;
  using LieGroup = dart::math::ProductLieGroup<Scalar>;
  using LieAlgebra = std::vector<Scalar>;
  using Tangent = std::vector<Scalar>;
  using Cotangent = std::vector<Scalar>;
  using Jacobian = Eigen::Matrix<Scalar, 1, 1>;

  static constexpr int SpaceDim = std::max({int(T<Scalar>::SpaceDim)...});
  static constexpr int GroupDim
      = dart::math::detail::accumulate(int(T<Scalar>::GroupDim)...);
  static constexpr int MatrixDim
      = dart::math::detail::accumulate(int(T<Scalar>::MatrixDim)...);
  static constexpr int DataDim
      = dart::math::detail::accumulate(int(T<Scalar>::DataDim)...);

  using DataType = Eigen::Matrix<Scalar, DataDim, 1>;
};

} // namespace Eigen::internal

namespace dart::math {

template <typename Scalar, template <typename> class... T>
class ProductLieGroup
  : public ProductLieGroupBase<ProductLieGroup<Scalar, T...>>
{
public:
  using Base = ProductLieGroupBase<ProductLieGroup<Scalar, T...>>;
  using This = ProductLieGroup<Scalar, T...>;

  using Base::ProductSize;

  using Base::DataDim;
  using Base::GroupDim;
  using Base::MatrixDim;
  using Base::SpaceDim;

  using DataType = typename Base::DataType;

protected:
  DataType m_data;
};

} // namespace dart::math

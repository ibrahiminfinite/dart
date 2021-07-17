/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/common/eigen_include.hpp"

namespace dart {
namespace math {

constexpr int Dynamic = Eigen::Dynamic;

template <typename Derived>
using MatrixBase = Eigen::MatrixBase<Derived>;

template <typename Scalar>
using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar, int Rows, int Cols = Rows>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

template <typename Scalar>
using Matrix1 = Matrix<Scalar, 1>;

template <typename Scalar>
using Matrix2 = Matrix<Scalar, 2>;

template <typename Scalar>
using Matrix3 = Matrix<Scalar, 3>;

template <typename Scalar>
using Matrix4 = Matrix<Scalar, 4>;

template <typename Scalar>
using Matrix5 = Matrix<Scalar, 5>;

template <typename Scalar>
using Matrix6 = Matrix<Scalar, 6>;

template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template <typename Scalar, int Dim>
using Vector = Eigen::Matrix<Scalar, Dim, 1>;

template <typename Scalar>
using Vector1 = Vector<Scalar, 1>;

template <typename Scalar>
using Vector2 = Vector<Scalar, 2>;

template <typename Scalar>
using Vector3 = Vector<Scalar, 3>;

template <typename Scalar>
using Vector4 = Vector<Scalar, 4>;

template <typename Scalar>
using Vector5 = Vector<Scalar, 5>;

template <typename Scalar>
using Vector6 = Vector<Scalar, 6>;

using Vector2i = Vector2<int>;
using Vector3i = Vector3<int>;
using Vector4i = Vector4<int>;
using Vector5i = Vector5<int>;
using Vector6i = Vector6<int>;
using VectorXi = VectorX<int>;

using Vector2f = Vector2<float>;
using Vector3f = Vector3<float>;
using Vector4f = Vector4<float>;
using Vector5f = Vector5<float>;
using Vector6f = Vector6<float>;
using VectorXf = VectorX<float>;

using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;
using Vector5d = Vector5<double>;
using Vector6d = Vector6<double>;
using VectorXd = VectorX<double>;

template <typename Scalar, int Options = 0>
using Quaternion = Eigen::Quaternion<Scalar, Options>;

template <typename Scalar, int Options = 0>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry, Options>;

template <typename Scalar, int Dofs, int Options = 0>
class SpatialJacobian;

// TODO(JS): Placeholder
template <typename Scalar>
class InertiaMatrix
{
public:
  void resize(int size)
  {
    m_matrix.resize(size, size);
  }

  const math::MatrixX<Scalar>& get_matrix() const
  {
    return m_matrix;
  }

protected:
private:
  math::MatrixX<Scalar> m_matrix;
};

// Forward declarations
template <typename Scalar, int Dim, int Options = 0>
class R;
template <typename Scalar, int Dim, int Options = 0>
class RAlgebra;
template <typename Scalar, int Dim, int Options = 0>
class RTangent;
template <typename Scalar, int Dim, int Options = 0>
class RCotangent;

#define DART_MATH_REAL_VECTOR_SPACE(dim)                                       \
  /** Translation group of dim##-dimension. */                                 \
  template <typename Scalar, int Options = 0>                                  \
  using R##dim = R<Scalar, dim, Options>;                                      \
  /** Translation group of dim##-dimension for float scalar type. */           \
  using R##dim##f = R##dim<float>;                                             \
  /** Translation group of dim##-dimension for double scalar type. */          \
  using R##dim##d = R##dim<double>;                                            \
  /** Translation group of dim##-dimension for long double scalar type. */     \
  using R##dim##ld = R##dim<long double>;                                      \
  /** Tangent space of the translation group of dim##-dimension for long       \
   * double scalar type. */                                                    \
  template <typename Scalar, int Options = 0>                                  \
  using R##dim##Tangent = RTangent<Scalar, dim, Options>;                      \
  /** Tangent space of the translation group of dim##-dimension for float      \
   * scalar type. */                                                           \
  using R##dim##Tangent##f = R##dim##Tangent<float>;                           \
  /** Tangent space of the translation group of dim##-dimension for double     \
   * scalar type. */                                                           \
  using R##dim##Tangent##d = R##dim##Tangent<double>;                          \
  /** Tangent space of the translation group of dim##-dimension for long       \
   * double scalar type. */                                                    \
  using R##dim##Tangent##ld = R##dim##Tangent<long double>;                    \
  /** Cotangent space of the translation group of dim##-dimension for long     \
   * double scalar type. */                                                    \
  template <typename Scalar, int Options = 0>                                  \
  using R##dim##Cotangent = RCotangent<Scalar, dim, Options>;                  \
  /** Cotangent space of the translation group of dim##-dimension for float    \
   * scalar type. */                                                           \
  using R##dim##Cotangent##f = R##dim##Cotangent<float>;                       \
  /** Cotangent space of the translation group of dim##-dimension for double   \
   * scalar type. */                                                           \
  using R##dim##Cotangent##d = R##dim##Cotangent<double>;                      \
  /** Cotangent space of the translation group of dim##-dimension for long     \
   * double scalar type. */                                                    \
  using R##dim##Cotangent##ld = R##dim##Cotangent<long double>;

/// Translation group of dynamic-size.
template <typename Scalar>
using RX = R<Scalar, Eigen::Dynamic>;
/// Translation group of dynamic-size for float scalar type.
using RXf = RX<float>;
/// Translation group of dynamic-size for double scalar type.
using RXd = RX<double>;
/// Translation group of dynamic-size for long double scalar type.
using RXld = RX<long double>;

DART_MATH_REAL_VECTOR_SPACE(0);
DART_MATH_REAL_VECTOR_SPACE(1);
DART_MATH_REAL_VECTOR_SPACE(2);
DART_MATH_REAL_VECTOR_SPACE(3);
DART_MATH_REAL_VECTOR_SPACE(4);
DART_MATH_REAL_VECTOR_SPACE(5);
DART_MATH_REAL_VECTOR_SPACE(6);

// Forward declarations of SO3 types
template <typename Scalar, int Options = 0>
class SO3;
template <typename Scalar, int Options = 0>
class SO3Algebra;
template <typename Derived>
class SO3TangentBase;
template <typename Scalar, int Options = 0>
class SO3Tangent;
template <typename Scalar, int Options = 0>
class SO3Cotangent;

template <typename Scalar, int Options = 0>
class SE3;
template <typename Scalar, int Options = 0>
class SE3Inverse;
template <typename Scalar, int Options = 0>
class SE3Algebra;
template <typename Derived>
class SE3TangentBase;
template <typename Scalar, int Options = 0>
class SE3Tangent;
template <typename Scalar, int Options = 0>
class SE3Cotangent;

template <typename Derived>
class GeneralizedCoordinatesBase;
template <typename Scalar, int Dofs, int Options = 0>
class GeneralizedCoordinates;

template <typename Derived>
class SpatialInertiaBase;
template <typename Scalar, int Options = 0>
class SpatialInertia;

} // namespace math
} // namespace dart

#include "dart/math/detail/type_impl.hpp"

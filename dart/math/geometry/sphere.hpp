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

#include "dart/common/eigen_include.hpp"
#include "dart/math/geometry/convex3.hpp"

namespace dart {
namespace math {

template <typename Scalar_>
class Sphere : public Convex3<Scalar_>
{
public:
  // Type aliases
  using Scalar = Scalar_;
  using Vector3 = typename Convex3<Scalar>::Vector3;

  /// Returns type string
  static const std::string& GetType();

  /// Computes volume given radius
  static Scalar ComputeVolume(Scalar radius);

  /// Computes inertia given radius and mass
  static Eigen::Matrix<Scalar, 3, 3> ComputeInertiaFromMass(
      Scalar radius, Scalar mass);

  /// Computes inertia given radius and density
  static Eigen::Matrix<Scalar, 3, 3> ComputeInertiaFromDensity(
      Scalar radius, Scalar density);

  /// Constructor
  ///
  /// \param[in] radius: The radius of this sphere to set.
  explicit Sphere(Scalar radius = 0.5);

  // Documentation inherited
  const std::string& get_type() const override;

  /// Returns the radius
  Scalar get_radius() const;

  /// Sets the radius5
  void set_radius(Scalar radius);

  // Documentation inherited
  Vector3 get_local_support_point(const Vector3& direction) const override;

private:
  Scalar m_radius;
};

using Spheref = Sphere<float>;
using Sphered = Sphere<double>;

extern template class DART_MATH_API Sphere<double>;

} // namespace math
} // namespace dart

#include "dart/math/geometry/detail/sphere_impl.hpp"

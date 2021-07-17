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

#include "dart/collision/ode/detail/ode_geom.hpp"
#include "dart/collision/ode/ode_include.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/math/geometry/plane3.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {
namespace detail {

//==============================================================================
template <typename Scalar>
OdePlane<Scalar>::OdePlane(
    const OdeObject<Scalar>* parent,
    const math::Vector3<Scalar>& normal,
    Scalar offset)
  : OdeGeom<Scalar>(parent)
{
  this->m_geom_id
      = dCreatePlane(nullptr, normal.x(), normal.y(), normal.z(), offset);
}

//==============================================================================
template <typename Scalar>
OdePlane<Scalar>::~OdePlane()
{
  dGeomDestroy(this->m_geom_id);
}

//==============================================================================
template <typename Scalar>
void OdePlane<Scalar>::update_engine_data()
{
  const math::Isometry3<Scalar>& tf = this->m_parent_object->get_pose();
  const math::Vector3<Scalar> pos = tf.translation();
  const math::Matrix3<Scalar> rot = tf.linear();

  auto plane = this->m_parent_object->get_geometry()
                   ->template as<math::Plane3<Scalar>>();
  const math::Vector3<Scalar>& normal = plane->get_normal();
  const Scalar offset = plane->get_offset();

  const math::Vector3<Scalar>& normal2 = rot * normal;
  const Scalar offset2 = offset + pos.dot(normal2);

  dGeomPlaneSetParams(
      this->m_geom_id, normal2.x(), normal2.y(), normal2.z(), offset2);
}

//==============================================================================
template <typename Scalar>
bool OdePlane<Scalar>::is_placeable() const
{
  return false;
}

} // namespace detail
} // namespace collision
} // namespace dart

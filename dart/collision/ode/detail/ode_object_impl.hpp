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

#include "dart/collision/ode/detail/ode_box.hpp"
#include "dart/collision/ode/detail/ode_capsule.hpp"
#include "dart/collision/ode/detail/ode_cylinder.hpp"
#include "dart/collision/ode/detail/ode_heightmap.hpp"
#include "dart/collision/ode/detail/ode_mesh.hpp"
#include "dart/collision/ode/detail/ode_plane.hpp"
#include "dart/collision/ode/detail/ode_sphere.hpp"
#include "dart/collision/ode/ode_conversion.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/collision/ode/ode_scene.hpp"
#include "dart/math/geometry/capsule.hpp"
#include "dart/math/geometry/cuboid.hpp"
#include "dart/math/geometry/cylinder.hpp"
#include "dart/math/geometry/heightmap.hpp"
#include "dart/math/geometry/plane3.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
detail::OdeGeom<Scalar>* create_ode_geom(
    OdeObject<Scalar>* object, const math::Geometry* shape)
{
  detail::OdeGeom<Scalar>* geom = nullptr;

  if (const auto sphere = shape->as<math::Sphere<Scalar>>()) {
    const auto& radius = sphere->get_radius();
    geom = new detail::OdeSphere<Scalar>(object, radius);

  } else if (const auto box = shape->as<math::Cuboid<Scalar>>()) {
    const auto& size = box->get_size();
    geom = new detail::OdeBox<Scalar>(object, size);

  } else if (const auto capsule = shape->as<math::Capsule<Scalar>>()) {
    const auto& radius = capsule->get_radius();
    const auto& height = capsule->get_height();
    geom = new detail::OdeCapsule<Scalar>(object, radius, height);

  } else if (const auto cylinder = shape->as<math::Cylinder<Scalar>>()) {
    const auto& radius = cylinder->get_radius();
    const auto& height = cylinder->get_height();
    geom = new detail::OdeCylinder<Scalar>(object, radius, height);

  } else if (const auto plane = shape->as<math::Plane3<Scalar>>()) {
    const auto& normal = plane->get_normal();
    const auto& offset = plane->get_offset();
    geom = new detail::OdePlane<Scalar>(object, normal, offset);

  } else if (const auto mesh = shape->as<math::TriMesh<Scalar>>()) {
    geom = new detail::OdeMesh<Scalar>(
        object, mesh, math::Vector3<Scalar>::Ones());

  } else if (const auto heightmap = shape->as<math::Heightmap<Scalar>>()) {
    geom = new detail::OdeHeightmap<Scalar>(object, heightmap);

  } else {
    DART_ERROR(
        "Attempting to create an unsupported shape type [{}]. Creating a "
        "sphere with 0.001 radius instead.",
        shape->get_type());
    geom = new detail::OdeSphere<Scalar>(object, Scalar(0.01));
  }
  // TODO(JS): not implemented for EllipsoidShape, ConeShape, MultiSphereShape,
  // and SoftMeshShape.

  assert(geom);
  const auto geom_id = geom->get_ode_geom_id();
  dGeomSetData(geom_id, object);

  return geom;
}

//==============================================================================
template <typename Scalar>
math::Isometry3<Scalar> OdeObject<Scalar>::get_pose() const
{
  math::Isometry3<Scalar> out = math::Isometry3<Scalar>::Identity();

  const auto* ode_quat = dBodyGetQuaternion(m_ode_body_id);
  out.linear() = math::Quaternion<Scalar>(
                     ode_quat[0], ode_quat[1], ode_quat[2], ode_quat[3])
                     .toRotationMatrix();

  const auto* ode_pos = dBodyGetPosition(m_ode_body_id);
  out.translation() << ode_pos[0], ode_pos[1], ode_pos[2];

  return out;
}

//==============================================================================
template <typename Scalar>
void OdeObject<Scalar>::set_pose(const math::Isometry3<Scalar>& tf)
{
  // If body id is nullptr, this object is immobile. Immobile geom doesn't need
  // to update its pose.
  if (!m_ode_body_id) {
    return;
  }

  const math::Quaternion<Scalar> quat(tf.linear());
  dQuaternion ode_quat;
  ode_quat[0] = quat.w();
  ode_quat[1] = quat.x();
  ode_quat[2] = quat.y();
  ode_quat[3] = quat.z();
  dBodySetQuaternion(m_ode_body_id, ode_quat);

  set_position(tf.translation());
}

//==============================================================================
template <typename Scalar>
math::Vector3<Scalar> OdeObject<Scalar>::get_position() const
{
  const auto* ode_pos = dBodyGetPosition(m_ode_body_id);
  return math::Vector3<Scalar>(ode_pos[0], ode_pos[1], ode_pos[2]);
}

//==============================================================================
template <typename Scalar>
void OdeObject<Scalar>::set_position(const math::Vector3<Scalar>& pos)
{
  // If body id is nullptr, this object is immobile. Immobile geom doesn't need
  // to update its pose.
  if (!m_ode_body_id) {
    return;
  }

  dBodySetPosition(m_ode_body_id, pos[0], pos[1], pos[1]);
}

//==============================================================================
template <typename Scalar>
OdeObject<Scalar>::OdeObject(OdeScene<Scalar>* group, math::GeometryPtr shape)
  : Object<Scalar>(group, shape), m_ode_geom{nullptr}, m_ode_body_id(nullptr)
{
  // Create detail::OdeGeom according to the shape type.
  // The geometry may have a transform assigned to it which is to
  // be treated as relative transform to the main body.
  m_ode_geom.reset(create_ode_geom(this, shape.get()));

  const auto geomId = m_ode_geom->get_ode_geom_id();
  assert(geomId);

  if (m_ode_geom->is_placeable()) {
    // if the geometry already has a pose, it is to be considered
    // a constant relative pose to the body.
    // Get the geometry pose to ensure this offset is set correctly.
    // Assigning a body to the geometry will overwrite the geometry
    // pose, so back it up first.
    dQuaternion geomRelRot;
    dGeomGetQuaternion(geomId, geomRelRot);
    const dReal* geomRelPos = dGeomGetPosition(geomId);
    assert(geomRelPos);

    // create the body
    m_ode_body_id = dBodyCreate(group->get_ode_engine()->get_ode_world_id());
    // attach geometry to body. This will set the geometry pose to identity.
    dGeomSetBody(geomId, m_ode_body_id);

    // set the offset
    dGeomSetOffsetPosition(geomId, geomRelPos[0], geomRelPos[1], geomRelPos[2]);
    dGeomSetOffsetQuaternion(geomId, geomRelRot);
  }
}

//==============================================================================
template <typename Scalar>
dBodyID OdeObject<Scalar>::get_ode_body_id() const
{
  return m_ode_body_id;
}

//==============================================================================
template <typename Scalar>
dGeomID OdeObject<Scalar>::get_ode_geom_id() const
{
  return m_ode_geom->get_ode_geom_id();
}

//==============================================================================
template <typename Scalar>
void OdeObject<Scalar>::update_engine_data()
{
  m_ode_geom->update_engine_data();
}

} // namespace collision
} // namespace dart

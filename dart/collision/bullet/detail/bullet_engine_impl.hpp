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

#include "dart/collision/bullet/bullet_conversion.hpp"
#include "dart/collision/bullet/bullet_engine.hpp"
#include "dart/collision/bullet/bullet_include.hpp"
#include "dart/collision/bullet/bullet_object.hpp"
#include "dart/collision/bullet/bullet_scene.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/capsule.hpp"
#include "dart/math/geometry/cuboid.hpp"
#include "dart/math/geometry/cylinder.hpp"
#include "dart/math/geometry/heightmap.hpp"
#include "dart/math/geometry/plane3.hpp"
#include "dart/math/geometry/sphere.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
btCollisionShape* create_bullet_mesh(const math::TriMesh<S>& mesh_data)
{
  auto tri_mesh = new btTriangleMesh();

  const auto& vertices = mesh_data.get_vertices();
  for (const auto& triangle : mesh_data.get_triangles()) {
    tri_mesh->addTriangle(
        to_bullet_vector3<S>(vertices[triangle[0]]),
        to_bullet_vector3<S>(vertices[triangle[1]]),
        to_bullet_vector3<S>(vertices[triangle[2]]));
  }

  auto* gimpact_mesh_shape = new btGImpactMeshShape(tri_mesh);
  gimpact_mesh_shape->updateBound();
  gimpact_mesh_shape->setUserPointer(tri_mesh);

  return gimpact_mesh_shape;
}

//==============================================================================
template <typename S>
btCollisionShape* create_bullet_heightmap(
    const math::Heightmap<S>& heightmap_src)
{
  // get the heightmap parameters
  const auto& scale = heightmap_src.get_scale();
  const S min_height = heightmap_src.get_min_height();
  const S max_height = heightmap_src.get_max_height();

  // determine which data type (float or double) is to be used for the field
  PHY_ScalarType scalar_yype = PHY_FLOAT;
  if constexpr (std::is_same_v<S, double>) {
    dterr << "Bullet does not support DOUBLE as heightmap field yet.\n";
    return nullptr;
    // take this back in as soon as it is supported
    // scalarType = PHY_DOUBLE;
  }

  // the y-values in the height field need to be flipped
  math::Heightmap<S> heightmap_data = heightmap_src;
  heightmap_data.flip_y();

  const auto& heights = heightmap_data.get_height_field();

  // create the height field
  const btVector3 localScaling(scale.x(), scale.y(), scale.z());
  const bool flipQuadEdges = false;
  auto height_field_shape = std::make_unique<btHeightfieldTerrainShape>(
      heightmap_data.get_width(), // Width of height field
      heightmap_data.get_depth(), // Depth of height field
      heights.data(),             // Height values
      1,                          // Height scaling
      min_height,                 // Min height
      max_height,                 // Max height
      2,                          // Up axis
      scalar_yype,                // Float or double field
      flipQuadEdges);             // Flip quad edges
  height_field_shape->setLocalScaling(localScaling);
  height_field_shape->setUseZigzagSubdivision(true);

  // change the relative transform of the height field so that the minimum
  // height is at the same z coordinate. Bullet shifts the height map such
  // that its center is the AABB center.
  const btVector3 trans(
      0, 0, ((max_height - min_height) * 0.5 + min_height) * scale.z());
  btTransform relativeShapeTransform(btMatrix3x3::getIdentity(), trans);

  // bullet places the heightfield such that the origin is in the
  // middle of the AABB. We want however that the minimum height value
  // is on x/y plane.
  btVector3 min;
  btVector3 max;
  height_field_shape->getAabb(btTransform::getIdentity(), min, max);
  DART_DEBUG(
      "AABB: min = [{}, {}, {}], max = [{}, {}, {}] (will be translated by "
      "z={})",
      min.x(),
      min.y(),
      min.z(),
      max.x(),
      max.y(),
      max.z(),
      trans.z());

  return nullptr;
}

//==============================================================================
template <typename S>
std::shared_ptr<BulletEngine<S>> BulletEngine<S>::Create()
{
  return std::shared_ptr<BulletEngine>(new BulletEngine());
}

//==============================================================================
template <typename S>
BulletEngine<S>::BulletEngine()
{
  // Do nothing
}

//==============================================================================
template <typename S>
BulletEngine<S>::~BulletEngine()
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& BulletEngine<S>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename S>
const std::string& BulletEngine<S>::GetType()
{
  static const std::string type = "bullet";
  return type;
}

//==============================================================================
template <typename S>
ScenePtr<S> BulletEngine<S>::create_scene()
{
  return std::make_shared<BulletScene<S>>(this);
}

//==============================================================================
template <typename S>
bool BulletEngine<S>::collide(
    ObjectPtr<S> object1,
    ObjectPtr<S> object2,
    const CollisionOption<S>& option,
    CollisionResult<S>* result)
{
  auto derived1 = std::dynamic_pointer_cast<BulletObject<S>>(object1);
  if (!derived1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto derived2 = std::dynamic_pointer_cast<BulletObject<S>>(object2);
  if (!derived2) {
    DART_ERROR("Invalid object");
    return false;
  }

  const auto bt_object1 = derived1->get_bullet_collision_object();
  if (!bt_object1) {
    DART_ERROR("Invalid object");
    return false;
  }

  const auto bt_object2 = derived2->get_bullet_collision_object();
  if (!bt_object2) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto group = create_scene();

  // Add bt_object1 and bt_object2
  DART_UNUSED(option, result);

  return false;
}

//==============================================================================
template <typename S>
std::shared_ptr<btCollisionShape>
BulletEngine<S>::create_bullet_collision_shape(
    const math::ConstGeometryPtr& shape)
{
  btCollisionShape* bt_shape = nullptr;

  if (const auto sphere = shape->as<math::Sphere<S>>()) {
    const auto& radius = sphere->get_radius();
    bt_shape = new btSphereShape(radius);

  } else if (const auto box = shape->as<math::Cuboid<S>>()) {
    const auto& size = box->get_size();
    bt_shape = new btBoxShape(to_bullet_vector3<S>(0.5 * size));

  } else if (const auto capsule = shape->as<math::Capsule<S>>()) {
    const auto& radius = capsule->get_radius();
    const auto& height = capsule->get_height();
    bt_shape = new btCapsuleShapeZ(radius, height);

  } else if (const auto cylinder = shape->as<math::Cylinder<S>>()) {
    const auto& radius = cylinder->get_radius();
    const auto& height = cylinder->get_height();
    bt_shape = new btCylinderShapeZ(btVector3(radius, radius, 0.5 * height));

  } else if (const auto plane = shape->as<math::Plane3<S>>()) {
    const auto& normal = plane->get_normal();
    const auto& offset = plane->get_offset();
    bt_shape = new btStaticPlaneShape(to_bullet_vector3<S>(normal), offset);

  } else if (const auto mesh = shape->as<math::TriMesh<S>>()) {
    bt_shape = create_bullet_mesh(*mesh);

  } else if (const auto heightmap = shape->as<math::Heightmap<S>>()) {
    bt_shape = create_bullet_heightmap(*heightmap);

  } else {
    DART_ERROR(
        "Attempting to create an unsupported shape type [{}]. Creating a "
        "sphere with 0.5 radius instead.",
        shape->get_type());
    bt_shape = new btSphereShape(0.5);
  }

  return std::shared_ptr<btCollisionShape>(bt_shape);
}

} // namespace collision
} // namespace dart

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
#include "dart/collision/bullet/bullet_group.hpp"
#include "dart/collision/bullet/bullet_object.hpp"
#include "dart/math/geometry/capsule.hpp"
#include "dart/math/geometry/cuboid.hpp"
#include "dart/math/geometry/cylinder.hpp"
#include "dart/math/geometry/heightmap.hpp"
#include "dart/math/geometry/plane3.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
math::Isometry3<S> BulletObject<S>::get_pose() const
{
  return to_pose<S>(m_bullet_collision_object->getWorldTransform());
}

//==============================================================================
template <typename S>
void BulletObject<S>::set_pose(const math::Isometry3<S>& tf)
{
  DART_ASSERT(m_bullet_collision_object);
  m_bullet_collision_object->setWorldTransform(to_bullet_pose<S>(tf));
}

//==============================================================================
template <typename S>
math::Vector3<S> BulletObject<S>::get_position() const
{
  const btTransform& bullet_pose
      = m_bullet_collision_object->getWorldTransform();
  return to_vector3<S>(bullet_pose.getOrigin());
}

//==============================================================================
template <typename S>
void BulletObject<S>::set_position(const math::Vector3<S>& pos)
{
  btTransform bullet_pose = m_bullet_collision_object->getWorldTransform();
  bullet_pose.setOrigin(to_bullet_vector3(pos));
  m_bullet_collision_object->setWorldTransform(bullet_pose);
}

//==============================================================================
template <typename S>
BulletObject<S>::BulletObject(BulletGroup<S>* group, math::GeometryPtr shape)
  : Object<S>(group, shape)
{
  auto engine = group->get_mutable_bullet_engine();
  auto bullet_collision_shape = engine->create_bullet_collision_shape(shape);

  m_bullet_collision_object = std::make_unique<btCollisionObject>();
  m_bullet_collision_object->setCollisionShape(bullet_collision_shape.get());
  m_bullet_collision_object->setUserPointer(this);
}

//==============================================================================
template <typename S>
const btCollisionObject* BulletObject<S>::get_bullet_collision_object() const
{
  return m_bullet_collision_object.get();
}

//==============================================================================
template <typename S>
void BulletObject<S>::update_engine_data()
{
  // Do nothing
}

} // namespace collision
} // namespace dart

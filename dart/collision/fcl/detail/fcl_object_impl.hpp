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

#include "dart/collision/fcl/fcl_conversion.hpp"
#include "dart/collision/fcl/fcl_object.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
math::Isometry3<S> FclObject<S>::get_pose() const
{
  return to_pose3<S>(m_fcl_collision_object->getTransform());
}

//==============================================================================
template <typename S>
void FclObject<S>::set_pose(const math::Isometry3<S>& tf)
{
  m_fcl_collision_object->setTransform(to_fcl_pose3<S>(tf));
}

//==============================================================================
template <typename S>
math::Vector3<S> FclObject<S>::get_position() const
{
  return to_vector3<S>(m_fcl_collision_object->getTranslation());
}

//==============================================================================
template <typename S>
void FclObject<S>::set_position(const math::Vector3<S>& pos)
{
  m_fcl_collision_object->setTranslation(to_fcl_vector3<S>(pos));
}

//==============================================================================
template <typename S>
FclCollisionObject<S>* FclObject<S>::get_fcl_collision_object()
{
  return m_fcl_collision_object.get();
}

//==============================================================================
template <typename S>
const FclCollisionObject<S>* FclObject<S>::get_fcl_collision_object() const
{
  return m_fcl_collision_object.get();
}

//==============================================================================
template <typename S>
FclObject<S>::FclObject(
    Scene<S>* collision_scene,
    math::GeometryPtr shape,
    const std::shared_ptr<FclCollisionGeometry<S>>& fcl_coll_geom)
  : Object<S>(collision_scene, shape),
    m_fcl_collision_object(new FclCollisionObject<S>(fcl_coll_geom))
{
  assert(fcl_coll_geom);
  m_fcl_collision_object->setUserData(this);
}

//==============================================================================
template <typename S>
void FclObject<S>::update_engine_data()
{
  m_fcl_collision_object->setTransform(to_fcl_pose3(get_pose()));
  m_fcl_collision_object->computeAABB();
}

} // namespace collision
} // namespace dart

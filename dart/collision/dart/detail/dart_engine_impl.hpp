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

#include "dart/collision/collision_result.hpp"
#include "dart/collision/dart/dart_engine.hpp"
#include "dart/collision/dart/dart_object.hpp"
#include "dart/collision/dart/dart_scene.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
int collide_sphere_sphere(
    DartObject<S>* o1,
    DartObject<S>* o2,
    S r0,
    const math::Isometry3<S>& c0,
    S r1,
    const math::Isometry3<S>& c1,
    CollisionResult<S>* result)
{
  S rsum = r0 + r1;
  math::Vector3<S> normal = c0.translation() - c1.translation();
  S normal_sqr = normal.squaredNorm();

  if (normal_sqr > rsum * rsum) {
    return 0;
  }

  r0 /= rsum;
  r1 /= rsum;

  math::Vector3<S> point = r1 * c0.translation() + r0 * c1.translation();
  double penetration;

  if (normal_sqr < 1e-6) {
    normal.setZero();
    penetration = rsum;

    if (result) {
      ContactPoint<S> contact;
      contact.point = point;
      contact.normal = normal;
      contact.depth = penetration;

      ContactShape<S> contact_shape;
      contact_shape.object1 = o1;
      contact_shape.object2 = o2;
      contact_shape.add_contact_point(contact);

      result->add_contact_shape(contact_shape);
    }
    return 1;
  }

  normal_sqr = std::sqrt(normal_sqr);
  normal *= (1.0 / normal_sqr);
  penetration = rsum - normal_sqr;

  if (result) {
    ContactPoint<S> contact;
    contact.point = point;
    contact.normal = normal;
    contact.depth = penetration;

    ContactShape<S> contact_shape;
    contact_shape.object1 = o1;
    contact_shape.object2 = o2;
    contact_shape.add_contact_point(contact);

    result->add_contact_shape(contact_shape);
  }

  return 1;
}

//==============================================================================
template <typename S>
int collide_pair(
    DartObject<S>* o1, DartObject<S>* o2, CollisionResult<S>* result)
{
  // TODO(JS): We could make the contact point computation as optional for
  // the case that we want only binary check.

  const auto& shape1 = o1->get_geometry();
  const auto& shape2 = o2->get_geometry();

  const math::Isometry3<S>& T1 = o1->get_pose();
  const math::Isometry3<S>& T2 = o2->get_pose();

  if (const auto& sphere1 = shape1->template as<math::Sphere<S>>()) {
    if (const auto& sphere2 = shape2->template as<math::Sphere<S>>()) {
      return collide_sphere_sphere(
          o1, o2, sphere1->get_radius(), T1, sphere2->get_radius(), T2, result);
    }
  }

  DART_ERROR(
      "Attempted to collision check for an unsupported shape pair [{}]-[{}]",
      shape1->get_type(),
      shape2->get_type());

  return false;
}

//==============================================================================
template <typename S>
std::shared_ptr<DartEngine<S>> DartEngine<S>::Create()
{
  return std::shared_ptr<DartEngine>(new DartEngine());
}

//==============================================================================
template <typename S>
DartEngine<S>::~DartEngine()
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& DartEngine<S>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename S>
const std::string& DartEngine<S>::GetType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
template <typename S>
ScenePtr<S> DartEngine<S>::create_scene()
{
  return std::make_shared<DartScene<S>>(this);
}

//==============================================================================
template <typename S>
bool DartEngine<S>::collide(
    ObjectPtr<S> object1,
    ObjectPtr<S> object2,
    const CollisionOption<S>& option,
    CollisionResult<S>* result)
{
  auto derived1 = std::dynamic_pointer_cast<DartObject<S>>(object1);
  if (!derived1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto derived2 = std::dynamic_pointer_cast<DartObject<S>>(object2);
  if (!derived2) {
    DART_ERROR("Invalid object");
    return false;
  }

  const int num_contacts = collide_pair(derived1.get(), derived2.get(), result);

  DART_UNUSED(option);

  return (num_contacts > 0);
}

} // namespace collision
} // namespace dart

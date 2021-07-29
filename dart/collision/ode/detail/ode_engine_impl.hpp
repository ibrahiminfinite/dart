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
#include "dart/collision/contact_shape.hpp"
#include "dart/collision/ode/ode_conversion.hpp"
#include "dart/collision/ode/ode_engine.hpp"
#include "dart/collision/ode/ode_object.hpp"
#include "dart/collision/ode/ode_scene.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
ContactPoint<S> convert_contact(
    const dContactGeom& odeContact, const CollisionOption<S>& option)
{
  ContactPoint<S> contact;

  if (option.enable_contact) {
    contact.point = to_vector3<S>(odeContact.pos);
    contact.normal = to_vector3<S>(odeContact.normal);
    contact.depth = odeContact.depth;
  }

  return contact;
}

//==============================================================================
template <typename S>
void report_contacts(
    int num_contacts,
    const dContactGeom* contact_geoms,
    OdeObject<S>* o1,
    OdeObject<S>* o2,
    const CollisionOption<S>& option,
    CollisionResult<S>& result)
{
  ContactShape<S> contact_shape;
  contact_shape.object1 = o1;
  contact_shape.object2 = o2;

  for (auto i = 0; i < num_contacts; ++i) {
    contact_shape.add_contact_point(convert_contact(contact_geoms[i], option));

    if (result.get_num_contacts() >= option.max_num_contacts) {
      result.add_contact_shape(contact_shape);
      return;
    }
  }
}

//==============================================================================
template <typename S>
std::shared_ptr<OdeEngine<S>> OdeEngine<S>::Create()
{
  return std::shared_ptr<OdeEngine>(new OdeEngine());
}

//==============================================================================
template <typename S>
OdeEngine<S>::OdeEngine()
{
  // Initialize ODE. dInitODE is deprecated.
  const auto initialized = dInitODE2(0);
  DART_ASSERT(initialized, "Failed to initialize the ODE engine.");
  DART_UNUSED(initialized);

  dAllocateODEDataForThread(dAllocateMaskAll);

  m_ode_world_id = dWorldCreate();
  DART_ASSERT(m_ode_world_id, "Failed to create ODE world.");
}

//==============================================================================
template <typename S>
OdeEngine<S>::~OdeEngine()
{
  // Do nothing
}

//==============================================================================
template <typename S>
const std::string& OdeEngine<S>::get_type() const
{
  return GetType();
}

//==============================================================================
template <typename S>
const std::string& OdeEngine<S>::GetType()
{
  static const std::string type = "ode";
  return type;
}

//==============================================================================
template <typename S>
ScenePtr<S> OdeEngine<S>::create_scene()
{
  return std::make_shared<OdeScene<S>>(this);
}

//==============================================================================
template <typename S>
bool OdeEngine<S>::collide(
    ObjectPtr<S> object1,
    ObjectPtr<S> object2,
    const CollisionOption<S>& option,
    CollisionResult<S>* result)
{
  auto derived1 = std::dynamic_pointer_cast<OdeObject<S>>(object1);
  if (!derived1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto derived2 = std::dynamic_pointer_cast<OdeObject<S>>(object2);
  if (!derived2) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto ode_geom_id1 = derived1->get_ode_geom_id();
  if (!ode_geom_id1) {
    DART_ERROR("Invalid object");
    return false;
  }

  auto ode_geom_id2 = derived2->get_ode_geom_id();
  if (!ode_geom_id2) {
    DART_ERROR("Invalid object");
    return false;
  }

  dContactGeom* ode_contact_geoms = new dContactGeom[option.max_num_contacts];
  const int num_contacts = dCollide(
      ode_geom_id1,
      ode_geom_id2,
      option.max_num_contacts,
      ode_contact_geoms,
      sizeof(dContactGeom));

  if (num_contacts > 0 && result) {
    report_contacts(
        num_contacts,
        ode_contact_geoms,
        derived1.get(),
        derived2.get(),
        option,
        *result);
  }

  delete[] ode_contact_geoms;
  return (num_contacts > 0);
}

//==============================================================================
template <typename S>
dWorldID OdeEngine<S>::get_ode_world_id() const
{
  return m_ode_world_id;
}

} // namespace collision
} // namespace dart

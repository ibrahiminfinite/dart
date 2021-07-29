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

#include "dart/collision/fcl/fcl_engine.hpp"
#include "dart/collision/fcl/fcl_object.hpp"
#include "dart/collision/fcl/fcl_scene.hpp"
#include "dart/collision/object.hpp"
#include "dart/common/logging.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

namespace {

//========================================================================================
template <typename S>
struct FclCollisionCallbackData
{
  FclCollisionRequest<S> fcl_request;

  FclCollisionResult<S> fcl_result;

  CollisionOption<S> option;

  CollisionResult<S>* result = nullptr;

  int num_collision_pairs = 0;

  bool done = false;

  bool found_collision() const
  {
    if (result) {
      return result->is_collision();
    } else {
      return (found_collision > 0);
    }
  }
};

//========================================================================================
template <typename S>
bool collision_callback(
    FclCollisionObject<S>* fcl_object1,
    FclCollisionObject<S>* fcl_object2,
    void* callback_data)
{
  auto fcl_callback_data
      = static_cast<FclCollisionCallbackData<S>*>(callback_data);

  if (fcl_callback_data->done) {
    return true;
  }

  const FclCollisionRequest<S>& fcl_request = fcl_callback_data->fcl_request;
  FclCollisionResult<S>& fcl_result = fcl_callback_data->fcl_result;

  const CollisionOption<S> option = fcl_callback_data->option;
  CollisionResult<S>* result = fcl_callback_data->result;

  // Clear previous results
  fcl_result.clear();

  // Perform narrow-phase detection
  ::fcl::collide(fcl_object1, fcl_object2, fcl_request, fcl_result);

  if (result) {
    DART_UNUSED(option);
    DART_NOT_IMPLEMENTED;
  } else {
    DART_UNUSED(option);
    DART_NOT_IMPLEMENTED;
  }

  return fcl_callback_data->done;
}

} // namespace

//========================================================================================
template <typename S>
FclScene<S>::FclScene(Engine<S>* engine)
  : Scene<S>(engine),
    m_broad_phase_alg(new FclDynamicAABBTreeCollisionManager<S>())
{
  // Do nothing
}

//========================================================================================
template <typename S>
ObjectPtr<S> FclScene<S>::create_object(math::GeometryPtr shape)
{
  if (!shape) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  auto fcl_collision_geometry
      = get_mutable_fcl_engine()->create_fcl_collision_geometry(shape);
  if (!fcl_collision_geometry) {
    DART_WARN("Failed to create FCL collision geometry.");
    return nullptr;
  }

  return std::shared_ptr<FclObject<S>>(
      new FclObject<S>(this, std::move(shape), fcl_collision_geometry));
}

//========================================================================================
template <typename S>
bool FclScene<S>::collide(
    const CollisionOption<S>& option, CollisionResult<S>* result)
{
  DART_UNUSED(option, result);
  DART_ASSERT(m_broad_phase_alg);

  FclCollisionCallbackData<S> fcl_callback_data;

  m_broad_phase_alg->collide(&fcl_callback_data, collision_callback);
  return false;
};

//========================================================================================
template <typename S>
FclEngine<S>* FclScene<S>::get_mutable_fcl_engine()
{
  return static_cast<FclEngine<S>*>(this->m_engine);
}

//========================================================================================
template <typename S>
const FclEngine<S>* FclScene<S>::get_fcl_engine() const
{
  return static_cast<const FclEngine<S>*>(this->m_engine);
}

//========================================================================================
template <typename S>
typename FclScene<S>::FCLCollisionManager*
FclScene<S>::get_fcl_collision_manager()
{
  return m_broad_phase_alg.get();
}

//========================================================================================
template <typename S>
const typename FclScene<S>::FCLCollisionManager*
FclScene<S>::get_fcl_collision_manager() const
{
  return m_broad_phase_alg.get();
}

} // namespace collision
} // namespace dart

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

#include "dart/collision/dart/broad_phase/broad_phase_algorithm.hpp"
#include "dart/collision/dart/broad_phase/simple_broad_phase.hpp"
#include "dart/collision/dart/dart_engine.hpp"
#include "dart/collision/dart/dart_object.hpp"
#include "dart/collision/dart/dart_scene.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart::collision {

//==============================================================================
template <typename Scalar>
DartScene<Scalar>::DartScene(Engine<Scalar>* engine)
  : Scene<Scalar>(engine),
    m_objects(
        engine->get_mutable_memory_manager().get_mutable_free_list_allocator())
{
  m_broad_phase = std::make_shared<detail::SimpleBroadPhaseAlgorithm<Scalar>>();
}

//==============================================================================
template <typename Scalar>
DartScene<Scalar>::~DartScene()
{
  auto& mm = this->m_engine->get_mutable_memory_manager();
  auto& allocator = mm.get_mutable_free_list_allocator();

  for (auto i = 0; i < m_objects.size(); ++i) {
    allocator.destroy(m_objects.get_derived(i));
  }
  m_objects.clear();
}

//==============================================================================
template <typename Scalar>
Object<Scalar>* DartScene<Scalar>::create_object_impl(
    ObjectId id, math::Geometry3Ptr<Scalar> geometry)
{
  if (!geometry) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  auto& mm = this->m_engine->get_mutable_memory_manager();
  auto object = mm.template construct_using_free<DartObject<Scalar>>(
      this, id, std::move(geometry));

  if (object) {
    if (!m_broad_phase->add_object(object)) {
      DART_DEBUG("Failed to add collision object to broad phase algorithm.");
      return nullptr;
    }

    m_objects.push_back(object);
  }

  return object;
}

//==============================================================================
template <typename Scalar>
void DartScene<Scalar>::destroy_object_impl(Object<Scalar>* object)
{
  if (auto casted = dynamic_cast<DartObject<Scalar>*>(object)) {
    auto& mm = this->m_engine->get_mutable_memory_manager();
    auto& allocator = mm.get_mutable_free_list_allocator();
    allocator.destroy(casted);

    m_objects.erase_derived(casted);
  }
}

//==============================================================================
template <typename Scalar>
const ObjectArray<Scalar>& DartScene<Scalar>::get_objects() const
{
  return m_objects;
}

//==============================================================================
template <typename Scalar>
ObjectArray<Scalar>& DartScene<Scalar>::get_mutable_objects()
{
  return m_objects;
}

//==============================================================================
template <typename Scalar>
DartEngine<Scalar>* DartScene<Scalar>::get_mutable_dart_engine()
{
  return static_cast<DartEngine<Scalar>*>(this->m_engine);
}

//==============================================================================
template <typename Scalar>
const DartEngine<Scalar>* DartScene<Scalar>::get_dart_engine() const
{
  return static_cast<const DartEngine<Scalar>*>(this->m_engine);
}

//==============================================================================
template <typename Scalar>
void DartScene<Scalar>::update(Scalar time_step)
{
  DART_NOT_IMPLEMENTED;

  m_broad_phase->update_overlapping_pairs(time_step, m_overlapping_pairs);
}

} // namespace dart::collision

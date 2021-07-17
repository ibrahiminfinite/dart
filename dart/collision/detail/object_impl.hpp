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

#include <cassert>

#include "dart/collision/engine.hpp"
#include "dart/collision/object.hpp"
#include "dart/collision/scene.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
Object<Scalar>::Object()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Object<Scalar>::~Object()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
Engine<Scalar>* Object<Scalar>::get_mutable_engine()
{
  return m_scene->get_mutable_engine();
}

//==============================================================================
template <typename Scalar>
const Engine<Scalar>* Object<Scalar>::get_engine() const
{
  return m_scene->get_engine();
}

//==============================================================================
template <typename Scalar>
const void* Object<Scalar>::get_user_data() const
{
  return m_user_data;
}

//==============================================================================
template <typename Scalar>
math::ConstGeometryPtr Object<Scalar>::get_geometry() const
{
  return m_geometry;
}

//==============================================================================
template <typename Scalar>
Object<Scalar>::Object(Scene<Scalar>* collisionGroup, math::GeometryPtr shape)
  : m_scene(collisionGroup), m_geometry(std::move(shape))
{
  assert(m_scene);
  assert(m_geometry);
}

} // namespace collision
} // namespace dart

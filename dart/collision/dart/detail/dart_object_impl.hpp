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

#include "dart/collision/dart/dart_group.hpp"
#include "dart/collision/dart/dart_object.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
math::Isometry3<S> DartObject<S>::get_pose() const
{
  return m_pose;
}

//==============================================================================
template <typename S>
void DartObject<S>::set_pose(const math::Isometry3<S>& tf)
{
  m_pose = tf;
}

//==============================================================================
template <typename S>
math::Vector3<S> DartObject<S>::get_position() const
{
  return m_pose.translation();
}

//==============================================================================
template <typename S>
void DartObject<S>::set_position(const math::Vector3<S>& pos)
{
  m_pose.translation() = pos;
}

//==============================================================================
template <typename S>
DartObject<S>::DartObject(DartGroup<S>* group, math::GeometryPtr shape)
  : Object<S>(group, shape)
{
  // Do nothing
}

//==============================================================================
template <typename S>
void DartObject<S>::update_engine_data()
{
  // Do nothing
}

} // namespace collision
} // namespace dart

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
 *   * Redistributions of source cbullet must retain the above copyright
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

namespace dart {
namespace collision {

//==============================================================================
template <typename S>
math::Vector3<S> to_vector3(const btVector3& vec)
{
  return math::Vector3<S>(vec.x(), vec.y(), vec.z());
}

//==============================================================================
template <typename S>
btVector3 to_bullet_vector3(const math::Vector3<S>& vec)
{
  return btVector3(vec.x(), vec.y(), vec.z());
}

//==============================================================================
template <typename S>
math::Matrix3<S> to_matrix3(const btMatrix3x3& rot)
{
  math::Matrix3<S> out;
  // clang-format off
  out << rot[0][0], rot[0][1], rot[0][2],
         rot[1][0], rot[1][1], rot[1][2],
         rot[2][0], rot[2][1], rot[2][2];
  // clang-format on
  return out;
}

//==============================================================================
template <typename S>
btMatrix3x3 to_bullet_matrix3(const math::Matrix3<S>& rot)
{
  // clang-format off
  return btMatrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
                     rot(1, 0), rot(1, 1), rot(1, 2),
                     rot(2, 0), rot(2, 1), rot(2, 2));
  // clang-format on
}

//==============================================================================
template <typename S>
math::Isometry3<S> to_pose(const btTransform& pose)
{
  math::Isometry3<S> out = math::Isometry3<S>::Identity();
  out.linear() = to_matrix3<S>(pose.getBasis());
  out.translation() = to_vector3<S>(pose.getOrigin());
  return out;
}

//==============================================================================
template <typename S>
btTransform to_bullet_pose(const math::Isometry3<S>& pose)
{
  btTransform out;
  out.setOrigin(to_bullet_vector3<S>(pose.translation()));
  out.setBasis(to_bullet_matrix3<S>(pose.linear()));
  return out;
}

} // namespace collision
} // namespace dart

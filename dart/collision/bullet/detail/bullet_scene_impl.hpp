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

#include "dart/collision/bullet/bullet_engine.hpp"
#include "dart/collision/bullet/bullet_object.hpp"
#include "dart/collision/bullet/bullet_scene.hpp"
#include "dart/collision/object.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/geometry/sphere.hpp"

namespace dart {
namespace collision {

//==============================================================================
template <typename Scalar>
BulletScene<Scalar>::BulletScene(Engine<Scalar>* engine) : Scene<Scalar>(engine)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
BulletScene<Scalar>::~BulletScene()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
ObjectPtr<Scalar> BulletScene<Scalar>::create_object(math::GeometryPtr shape)
{
  if (!shape) {
    DART_WARN("Not allowed to create a collision object for a null shape");
    return nullptr;
  }

  return std::shared_ptr<BulletObject<Scalar>>(
      new BulletObject<Scalar>(this, std::move(shape)));
}

//==============================================================================
template <typename Scalar>
BulletEngine<Scalar>* BulletScene<Scalar>::get_mutable_bullet_engine()
{
  return static_cast<BulletEngine<Scalar>*>(this->m_engine);
}

//==============================================================================
template <typename Scalar>
const BulletEngine<Scalar>* BulletScene<Scalar>::get_bullet_engine() const
{
  return static_cast<const BulletEngine<Scalar>*>(this->m_engine);
}

} // namespace collision
} // namespace dart

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

#include "dart/collision/engine.hpp"
#include "dart/collision/narrow_phase_interface.hpp"

namespace dart::collision {

//==============================================================================
template <typename Scalar>
bool collide(
    Object<Scalar>* object1,
    Object<Scalar>* object2,
    const CollisionOption<Scalar>& option,
    CollisionResult<Scalar>* result)
{
  if (!object1 || !object2) {
    return false;
  }

  DART_ASSERT(object1->get_engine());
  DART_ASSERT(object2->get_engine());

  Engine<Scalar>* engine = object1->get_mutable_engine();
  DART_ASSERT(engine);

  if (engine != object2->get_engine()) {
    DART_ERROR(
        "Failed to check collision for objects created from different engines. "
        "Object1 from [{}] while Object2 from [{}]",
        object1->get_engine()->get_type(),
        object2->get_engine()->get_type());
    return false;
  }

  return engine->collide(object1, object2, option, result);
}

} // namespace dart::collision

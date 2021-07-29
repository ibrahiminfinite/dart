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

#include "dart/collision/bullet/bullet_include.hpp"
#include "dart/collision/bullet/bullet_type.hpp"
#include "dart/collision/export.hpp"
#include "dart/collision/scene.hpp"
#include "dart/collision/type.hpp"

namespace dart {
namespace collision {

template <typename S_>
class BulletScene : public Scene<S_>
{
public:
  using S = S_;

  friend class BulletEngine<S>;

  /// Constructor
  BulletScene(Engine<S>* engine);

  /// Destructor
  ~BulletScene() override;

  ObjectPtr<S> create_object(math::GeometryPtr shape) override;

  // Documentation inherited
  bool collide(
      const CollisionOption<S>& option = {},
      CollisionResult<S>* result = nullptr) override
  {
    DART_UNUSED(option, result);
    DART_NOT_IMPLEMENTED;
    return false;
  };

protected:
  /// Returns the parent collision engine
  BulletEngine<S>* get_mutable_bullet_engine();

  /// Returns the parent collision engine
  const BulletEngine<S>* get_bullet_engine() const;

private:
  friend class BulletObject<S>;
};

#if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE
extern template class DART_COLLISION_API BulletScene<double>;
#endif

#if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT
extern template class DART_COLLISION_API BulletScene<float>;
#endif

} // namespace collision
} // namespace dart

#include "dart/collision/bullet/detail/bullet_scene_impl.hpp"

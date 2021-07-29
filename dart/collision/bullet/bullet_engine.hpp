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

#include <map>

#include "dart/collision/bullet/bullet_include.hpp"
#include "dart/collision/bullet/bullet_type.hpp"
#include "dart/collision/engine.hpp"
#include "dart/collision/export.hpp"
#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace collision {

template <typename S_>
class BulletEngine : public Engine<S_>
{
public:
  // Type aliases
  using S = S_;

  static std::shared_ptr<BulletEngine> Create();

  /// Constructor
  ~BulletEngine() override;

  // Documentation inherited
  const std::string& get_type() const override;

  /// Get collision detector type for this class
  static const std::string& GetType();

  // Documentation inherited
  ScenePtr<S> create_scene() override;

  // Documentation inherited
  bool collide(
      ObjectPtr<S> object1,
      ObjectPtr<S> object2,
      const CollisionOption<S>& option = {},
      CollisionResult<S>* result = nullptr) override;

protected:
  /// Constructor
  BulletEngine();

  //  dWorldID get_bullet_world_id() const;

  //  /// Top-level world for all bodies
  //  dWorldID m_bullet_world_id;

  std::shared_ptr<btCollisionShape> create_bullet_collision_shape(
      const math::ConstGeometryPtr& shape);

private:
  friend class BulletScene<S>;
  friend class BulletObject<S>;

  DART_REGISTER_ENGINE_IN_HEADER(BulletEngine<S>);
};

DART_REGISTER_ENGINE_OUT_HEADER(BulletEngine<S>);

using BulletEnginef = BulletEngine<float>;
using BulletEngined = BulletEngine<double>;

#if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE
extern template class DART_COLLISION_API BulletEngine<double>;
#endif

#if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT
extern template class DART_COLLISION_API BulletEngine<float>;
#endif

} // namespace collision
} // namespace dart

#include "dart/collision/bullet/detail/bullet_engine_impl.hpp"

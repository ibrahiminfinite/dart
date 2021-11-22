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

#include "dart/collision/fcl/backward_compatibility.hpp"
#include "dart/collision/fcl/fcl_type.hpp"
#include "dart/collision/scene.hpp"
#include "dart/collision/type.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class FclScene : public Scene<Scalar_>
{
public:
  using Scalar = Scalar_;
  using FCLCollisionManager = FclDynamicAABBTreeCollisionManager<Scalar>;

  friend class FclEngine<Scalar>;

  /// Constructor
  FclScene(Engine<Scalar>* engine);

  /// Destructor
  ~FclScene() override;

protected:
  // Documentation inherited
  Object<Scalar>* create_object_impl(
      ObjectId id, math::Geometry3Ptr<Scalar> geometry) override;

  // Documentation inherited
  void destroy_object_impl(Object<Scalar>* object) override;

  const ObjectArray<Scalar>& get_objects() const override;

  ObjectArray<Scalar>& get_mutable_objects() override;

public:
  void update(Scalar time_step = 1e-3) override;

protected:
  FclEngine<Scalar>* get_mutable_fcl_engine();

  const FclEngine<Scalar>* get_fcl_engine() const;

  /// Return FCL collision manager that is also a broad-phase algorithm
  FCLCollisionManager* get_fcl_collision_manager();

  /// Return FCL collision manager that is also a broad-phase algorithm
  const FCLCollisionManager* get_fcl_collision_manager() const;

private:
  FclObjectArray<Scalar> m_objects;

  /// FCL broad-phase algorithm
  std::unique_ptr<FCLCollisionManager> m_broad_phase_alg;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, FclScene)

} // namespace collision
} // namespace dart

#include "dart/collision/fcl/detail/fcl_scene_impl.hpp"

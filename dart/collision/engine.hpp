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

#include <memory>
#include <string>
#include <unordered_map>

#include "dart/collision/collision_option.hpp"
#include "dart/collision/export.hpp"
#include "dart/collision/object.hpp"
#include "dart/collision/type.hpp"
#include "dart/common/factory.hpp"
#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace collision {

template <typename Scalar_>
class Engine
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Creates a new collision detector.
  ///
  /// \param[in] engine_name: Name of the underlying collision detection engine
  /// to create.
  static EnginePtr<Scalar> Create(const std::string& engine_name);

  /// Destructor
  virtual ~Engine();

  /// Returns collision detection engine type as a std::string.
  virtual const std::string& get_type() const = 0;

  /// Creates a collision scene.
  virtual ScenePtr<Scalar> create_scene() = 0;

  /// Create an collision object for a geometry type
  template <typename GeometryType, typename... Args>
  ObjectPtr<Scalar> create_object(Args&&... args);

  /// Create an collision object for a sphere shape
  template <typename... Args>
  ObjectPtr<Scalar> create_sphere_object(Args&&... args);

  /// Performs narrow phase collision detection
  virtual bool collide(
      ObjectPtr<Scalar> object1,
      ObjectPtr<Scalar> object2,
      const CollisionOption<Scalar>& option = {},
      CollisionResult<Scalar>* result = nullptr)
      = 0;

protected:
  /// Registrar to register a concrete engine to the factory
  ///
  /// Add the following line to the concrete engine:
  /// static typename Engine<Scalar>::template Regist
  template <typename Derived>
  using Registrar = common::FactoryRegistrar<
      std::string,
      Engine<Scalar>,
      Derived,
      std::shared_ptr<Engine<Scalar>>>;

  /// Constructor
  Engine() = default;

private:
  using Factory = common::Factory<std::string, Engine, std::shared_ptr<Engine>>;
  using SingletonFactory = common::Singleton<Factory>;

  static std::unordered_map<std::string, EnginePtr<Scalar>> m_engines;

  Scene<Scalar>* get_default_scene();

  ScenePtr<Scalar> m_default_scene;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, Engine)

} // namespace collision
} // namespace dart

#define DART_REGISTER_ENGINE_IN_HEADER(engine_type)                            \
  static typename Engine<Scalar>::template Registrar<engine_type> m_registrar

#define DART_REGISTER_ENGINE_OUT_HEADER(engine_type)                           \
  template <typename Scalar>                                                   \
  typename Engine<Scalar>::template Registrar<engine_type>                     \
      engine_type::m_registrar                                                 \
  {                                                                            \
    engine_type::GetType(), []() -> std::shared_ptr<engine_type> {             \
      return engine_type::Create();                                            \
    }                                                                          \
  }

#include "dart/collision/detail/engine_impl.hpp"

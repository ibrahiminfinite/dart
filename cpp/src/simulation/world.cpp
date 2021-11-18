/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/simulation/world.hpp"

#include "dart/collision/all.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"
#include "dart/simulation/system/collision_detection_system.hpp"

namespace dart::simulation {

//==============================================================================
struct World::Implementation
{
  CollisionDetectionSystem collision_detection_system;

  collision::EnginePtr<double> collision_engine;

  collision::Scene<double>* collision_scene;

  math::Vector3d gravity = math::Vector3d(0, 0, -9.81);

  std::vector<RigidBodyPtr> rigid_bodies;

  long step_count = 0;

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
World::World() : m_impl(std::make_unique<Implementation>())
{
  m_impl->collision_engine = collision::DartEngine<double>::Create();
  m_impl->collision_scene = m_impl->collision_engine->create_scene();
}

//==============================================================================
World::~World()
{
  m_impl->collision_engine->destroy_scene(m_impl->collision_scene);
}

//==============================================================================
void World::update(double time_step)
{
  // Perform broad-phase collision detection (rough collision detection)
  m_impl->collision_scene->update(time_step);

  // Update islands (active system groups) based on the broad-phase result

  // Perform narrow-phase collision detection

  // Report the collision results to the subscribers

  // Update physics objects

  // Update constraint activations

  // Proceed unconstrained systems

  // Solve the contacts and constraints

  // Integrate the position and orientation of the systems

  // Correct the positions

  // Update kinematics

  // Update collision objects

  // Update sleeping objects

  // Reset inputs

  // Reset the islands

  // Update memory allocator

  m_impl->step_count++;

  DART_NOT_IMPLEMENTED;
}

//==============================================================================
long World::get_step_count() const
{
  return m_impl->step_count;
}

//==============================================================================
void World::set_gravity(const math::Vector3d& gravity)
{
  m_impl->gravity = gravity;
}

//==============================================================================
math::Vector3d& World::get_gravity() const
{
  return m_impl->gravity;
}

//==============================================================================
bool World::add_rigid_body(RigidBodyPtr rigid_body)
{
  m_impl->rigid_bodies.push_back(rigid_body);
  return true;
}

//==============================================================================
int World::get_rigid_body_count() const
{
  return m_impl->rigid_bodies.size();
}

} // namespace dart::simulation

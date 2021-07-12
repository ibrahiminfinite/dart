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

#include <dart/collision/collision.hpp>
#include <dart/math/math.hpp>
#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
template <typename T>
struct NarrowPhaseTest : public testing::Test {
  using Type = T;
};

//==============================================================================
using Types = testing::Types</*double, */ float>;

//==============================================================================
TYPED_TEST_CASE(NarrowPhaseTest, Types);

//==============================================================================
template <typename EngineT>
void test_collide(const EngineT& engine)
{
  using S = typename EngineT::element_type::S;

  if (!engine) {
    return;
  }

  if (engine->get_type() == collision::BulletEngine<S>::GetType()) {
    return;
  }

  auto sphere1 = engine->create_sphere_object(0.5);
  auto sphere2 = engine->create_sphere_object(0.5);
  ASSERT_TRUE(sphere1);
  ASSERT_TRUE(sphere2);

  sphere1->set_position(math::Vector3<S>(-1, 0, 0));
  sphere2->set_position(math::Vector3<S>(1, 0, 0));
  EXPECT_FALSE(collision::collide(sphere1, sphere2));

  sphere1->set_position(math::Vector3<S>(-0.25, 0, 0));
  sphere2->set_position(math::Vector3<S>(0.25, 0, 0));
  EXPECT_TRUE(collision::collide(sphere1, sphere2));

  collision::CollisionOption<S> option;
  option.enable_contact = true;
  option.max_num_contacts = 10;
  collision::CollisionResult<S> result;
  sphere1->set_position(math::Vector3<S>(-0.25, 0, 0));
  sphere2->set_position(math::Vector3<S>(0.25, 0, 0));
  EXPECT_TRUE(collision::collide(sphere1, sphere2, option, &result));
}

//==============================================================================
TYPED_TEST(NarrowPhaseTest, Collide)
{
  using S = typename TestFixture::Type;

  test_collide(collision::FclEngine<S>::Create());
#if DART_HAVE_ODE
  test_collide(collision::OdeEngine<S>::Create());
#endif
#if DART_HAVE_Bullet
  test_collide(collision::BulletEngine<S>::Create());
#endif
}

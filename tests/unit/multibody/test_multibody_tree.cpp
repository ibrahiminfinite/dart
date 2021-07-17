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

#include <gtest/gtest.h>

#include "dart/multibody/multibody.hpp"

using namespace dart;
using namespace multibody;

//==============================================================================
template <typename T>
struct MultibodyTest : public testing::Test
{
  using Type = T;
};

//==============================================================================
using Types = testing::Types<float, double, long double>;

//==============================================================================
TYPED_TEST_SUITE(MultibodyTest, Types);

//==============================================================================
TYPED_TEST(MultibodyTest, Basics)
{
  using Scalar = typename TestFixture::Type;

  //  auto mb1 = MultibodyTree<Scalar>::Create().from_urdf(
  //      "~/.dart/data/samples/urdf/ur10/ur10.urdf").with(ros_package_manager);

  //  auto mb2 = MultibodyTree<Scalar>::Create().
  //      add_body_node()
  //        .with_parent_joint()
  //          .with_axis()

  auto mb_1 = MultibodyTree<Scalar>::CreateByBuilder().build();
  std::shared_ptr<MultibodyTree<Scalar>> mb_2 = MultibodyTree<Scalar>::Create();

  MultibodyTreeBuilder<Scalar> mb_builder;
  // clang-format off
  std::shared_ptr<MultibodyTree<Scalar>> mb_3 = mb_builder
      .name("MultibodyTree 1")
      .root_body_node()
        .name("Body 1")
        .template parent_joint<FreeJoint<Scalar>>()
        .child_body_node()
          .name("Body 2")
          .child_body_node()
            .name("Body 3")
            .child_body_node()
            .end_branch()
          .child_body_node()
          .name("Body 4");
  // clang-format on
  mb_3->print(std::cout);

  //  mb_builder

  //  auto root = mb->create_root_body_node();
  //  EXPECT_TRUE(root != nullptr);

  //  EXPECT_TRUE(mb->create_root_body_node() == nullptr);

  //  auto body_1 = root->create_child_body_node();
  //  EXPECT_TRUE(body_1 != nullptr);

  //  // Fixed joint is assigned by default
  //  auto joint_1 = body_1->get_parent_joint();
  //  EXPECT_TRUE(joint_1 != nullptr);
  //  EXPECT_TRUE(joint_1->template is<FixedJoint<Scalar>>());

  //  body_1->template set_parent_joint<FreeJoint<Scalar>>();

  //  auto ee_1 = body_1->template create_child_node<EndEffector<Scalar>>();
  //  EXPECT_TRUE(ee_1 != nullptr);

  //  auto jac = ee_1->get_spatial_jacobian();
  //  (void)jac;

  //  BodyNodePtr<Scalar> body_3 = mb->create_body_node2().with_mass(10);
  //  auto body_4 = mb->create_body_node2().with_mass(10).build();

  //  (void)body_3;
  //  (void)body_4;
}

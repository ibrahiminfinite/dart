/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <iostream>
#include <gtest/gtest.h>
#include "dart/dart.hpp"
#include "TestHelpers.hpp"

using namespace dart;

////==============================================================================
//template <typename SE3Type>
//void testSettersAndGetters()
//{
//  using SO3Type = typename SE3Type::SO3Type;
//  using TranslationType = typename SE3Type::TranslationType;

//  SE3Type tf1;
//  tf1.setIdentity();
//  EXPECT_TRUE(tf1 == SE3Type::Identity());

//  SE3Type tf2(SO3Type::Random());
//  EXPECT_TRUE(tf2.getTranslation() == TranslationType::Zero());

//  SE3Type tf3(SE3Type::FromTranslation, TranslationType::Zero());
//  EXPECT_TRUE(tf3.getRotation() == SO3Type::Identity());

////  Eigen::Isometry3d q = Eigen::Matrix<double, 4, 4>::Zero();
//}

////==============================================================================
//TEST(SE3, SettersAndGetters)
//{
//  testSettersAndGetters<SE3<double, SO3RotationMatrix>>();
//  testSettersAndGetters<SE3<double, SO3RotationVector>>();
//  testSettersAndGetters<SE3<double, SO3AngleAxis>>();
//  testSettersAndGetters<SE3<double, SO3Quaternion>>();
//  // EulerAngles
//}

////==============================================================================
//template <typename SE3Type>
//void testGroupOperations()
//{
//  SE3Type tf1 = SE3Type::Random();

//  SE3Type inverse1 = tf1.getInverse();
//  SE3Type inverse2 = tf1;
//  inverse2.invert();

//  EXPECT_TRUE(inverse1.isApprox(inverse2));
//}

////==============================================================================
//TEST(SE3, GroupOperations)
//{
//  testGroupOperations<SE3<double, SO3RotationMatrix>>();
////  testGroupOperations<SE3<double, SO3RotationVector>>();
////  testGroupOperations<SE3<double, SO3AngleAxis>>();
////  testGroupOperations<SE3<double, SO3Quaternion>>();
//}

////==============================================================================
//TEST(SE3, HeterogeneousAssignment)
//{
//  SE3<double, SO3RotationMatrix> tf1;
//  SE3<double, SO3RotationVector> tf2;
//  SE3<double, SO3AngleAxis> tf3;
//  SE3<double, SO3Quaternion> tf4;

//  tf1 = tf2;
//  tf2 = tf3;
//  tf3 = tf4;

////  tf1.setRandom();
////  tf2.setRandom();
////  EXPECT_FALSE(tf1.isApprox(tf2));

////  tf1 = tf2;
////  EXPECT_TRUE(tf1.isApprox(tf2));
//}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

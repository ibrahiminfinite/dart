/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "MyWindow.hpp"

//==============================================================================
MyWindow::MyWindow() : SimWindow()
{
  // Do nothing
}

//==============================================================================
MyWindow::~MyWindow()
{
  // Do nothing
}

//==============================================================================
void MyWindow::drawWorld() const
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawWorld();
}

//==============================================================================
void MyWindow::keyboard(unsigned char key, int x, int y)
{
  switch (key)
  {
    case ' ': // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'q':   // Spawn a cube
    case 'Q': { // Spawn a cube
      Eigen::Vector3d position = Eigen::Vector3d(
          dart::math::Random::uniform(-1.0, 1.0),
          dart::math::Random::uniform(0.5, 1.0),
          dart::math::Random::uniform(-1.0, 1.0));
      Eigen::Vector3d size = Eigen::Vector3d(
          dart::math::Random::uniform(0.1, 0.5),
          dart::math::Random::uniform(0.1, 0.5),
          dart::math::Random::uniform(0.1, 0.5));
      spawnCube(position, size);
      break;
    }
    case 'w':   // Spawn a cube
    case 'W': { // Spawn a cube
      if (mWorld->getNumSkeletons() > 1)
        mWorld->removeSkeleton(
            mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
      break;
    }
    default:
      Win3D::keyboard(key, x, y);
  }
  glutPostRedisplay();
}

//==============================================================================
void MyWindow::spawnCube(
    const Eigen::Vector3d& position, const Eigen::Vector3d& size, double mass)
{
  static size_t index = 0;

  dart::dynamics::SkeletonPtr newCubeSkeleton
      = dart::dynamics::Skeleton::create(
          "skeleton (" + std::to_string(index++) + ")");

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(mass);
  body.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(size, mass));
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(size));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(position);

  auto pair
      = newCubeSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
          nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  mWorld->addSkeleton(newCubeSkeleton);
}

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

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>

using namespace dart;

//==============================================================================
dynamics::SkeletonPtr createBox(
    int index, const Eigen::Vector3d& position, const Eigen::Vector3d& size)
{
  dart::dynamics::SkeletonPtr skel = dart::dynamics::Skeleton::create();
  skel->setName("skeleton (" + std::to_string(index) + ")");

  const double dense = 1000;
  const double mass = dart::dynamics::BoxShape::computeVolume(size) * dense;

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link (" + std::to_string(index) + ")";
  body.mInertia.setMass(mass);
  body.mInertia.setMoment(dart::dynamics::BoxShape::computeInertia(size, mass));
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(size));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint (" + std::to_string(index) + ")";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(position);

  auto pair = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
      nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(
      dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0));

  return skel;
}

//==============================================================================
dynamics::SkeletonPtr createGround(
    double x = 20, double y = 20, double thickness = 0.01)
{
  // Create a Skeleton to represent the ground
  dynamics::SkeletonPtr ground = dynamics::Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
  dynamics::WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr, joint);
  dynamics::ShapePtr groundShape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(x, y, thickness));

  auto shapeNode = ground->getBodyNode(0)
                       ->createShapeNodeWith<
                           dynamics::VisualAspect,
                           dynamics::CollisionAspect,
                           dynamics::DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

//==============================================================================
simulation::WorldPtr createWorld(int rows = 10, int cols = 10, int depths = 10)
{
  auto world = simulation::World::create();

  auto ground = createGround();
  world->addSkeleton(ground);

  const double length = 0.25;
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(length);
  const double space = 0.2 * length;
  const double spaceFromGround = length * 5;
  int index = 0;
  double z = length / 2 + spaceFromGround;
  for (int k = 0; k < depths; ++k)
  {
    double y = -(cols / 2) * (length + space);
    for (int j = 0; j < cols; ++j)
    {
      double x = -(rows / 2) * (length + space);
      for (int i = 0; i < rows; ++i)
      {
        auto box = createBox(index++, Eigen::Vector3d(x, y, z), size);
        world->addSkeleton(box);
        x += length + space;
      }
      y += length + space;
    }
    z += length + space;
  }

  return world;
}

//==============================================================================
class CustomWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  CustomWorldNode(
      const std::shared_ptr<dart::simulation::World>& world,
      int maxFrames = 0)
    : dart::gui::osg::RealTimeWorldNode(std::move(world)), mMaxFrames(maxFrames)
  {
    // Do nothing
  }

private:
  int mMaxFrames;
};

//==============================================================================
int main()
{
  // Create world
  auto world = createWorld(10, 10, 10);
  assert(world != NULL);

  // Use bullet collision detector for capsule and multi-sphere-convex-hull
  // shapes
  world->getConstraintSolver()->setCollisionDetector(
      collision::BulletCollisionDetector::create());

  // Wrap a WorldNode around it
  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::Viewer();
  viewer.addWorldNode(node);

  viewer.addInstructionText("Press space to start free falling the box.\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 1600x900
  viewer.setUpViewInWindow(0, 0, 1600, 900);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(10, 10, 10), ::osg::Vec3(0, 0, 1.5), ::osg::Vec3(0, 0, 1));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}

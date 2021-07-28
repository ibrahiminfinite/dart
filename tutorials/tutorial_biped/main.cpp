/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/dart.hpp>

const double default_speed_increment = 0.5;

const int default_ik_iterations = 4500;

const double default_force = 50.0; // N
const int default_countdown = 100; // Number of timesteps for applying force

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui::osg;
using namespace dart::io;
using namespace dart::math;

class Controller
{
public:
  /// Constructor
  Controller(const SkeletonPtr& biped) : mBiped(biped), mSpeed(0.0)
  {
    int nDofs = mBiped->getNumDofs();

    mForces = Eigen::VectorXd::Zero(nDofs);

    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    for (std::size_t i = 0; i < 6; ++i) {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for (std::size_t i = 6; i < biped->getNumDofs(); ++i) {
      mKp(i, i) = 1000;
      mKd(i, i) = 50;
    }

    setTargetPositions(mBiped->getPositions());
  }

  /// Reset the desired dof position to the current position
  void setTargetPositions(const Eigen::VectorXd& pose)
  {
    mTargetPositions = pose;
  }

  /// Clear commanding forces
  void clearForces()
  {
    mForces.setZero();
  }

  /// Add commanding forces from PD controllers
  void addPDForces()
  {
    // Lesson 2
  }

  /// Add commanind forces from Stable-PD controllers
  void addSPDForces()
  {
    // Lesson 3
  }

  /// add commanding forces from ankle strategy
  void addAnkleStrategyForces()
  {
    // Lesson 4
  }

  // Send velocity commands on wheel actuators
  void setWheelCommands()
  {
    // Lesson 6
  }

  void changeWheelSpeed(double increment)
  {
    mSpeed += increment;
    std::cout << "wheel speed = " << mSpeed << std::endl;
  }

protected:
  /// The biped Skeleton that we will be controlling
  SkeletonPtr mBiped;

  /// Joint forces for the biped (output of the Controller)
  Eigen::VectorXd mForces;

  /// Control gains for the proportional error terms in the PD controller
  Eigen::MatrixXd mKp;

  /// Control gains for the derivative error terms in the PD controller
  Eigen::MatrixXd mKd;

  /// Target positions for the PD controllers
  Eigen::VectorXd mTargetPositions;

  /// For velocity actuator: Current speed of the skateboard
  double mSpeed;
};

class TutorialWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  TutorialWorldNode(WorldPtr world)
    : dart::gui::osg::RealTimeWorldNode(std::move(world)),
      mForceCountDown(0),
      mPositiveSign(true)
  {
    mController = std::make_unique<Controller>(mWorld->getSkeleton("biped"));
  }

  void customPreStep() override
  {
    mController->clearForces();

    // Lesson 3
    mController->addSPDForces();

    // Lesson 4
    mController->addAnkleStrategyForces();

    // Lesson 6
    mController->setWheelCommands();

    // Apply body forces based on user input, and color the body shape red
    if (mForceCountDown > 0) {
      BodyNode* bn = mWorld->getSkeleton("biped")->getBodyNode("h_abdomen");
      auto shapeNodes = bn->getShapeNodesWith<VisualAspect>();
      shapeNodes[0]->getVisualAspect()->setColor(dart::Color::Red());

      if (mPositiveSign)
        bn->addExtForce(
            default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);
      else
        bn->addExtForce(
            -default_force * Eigen::Vector3d::UnitX(),
            bn->getCOM(),
            false,
            false);

      --mForceCountDown;
    }
  }

  Controller* getController()
  {
    return mController.get();
  }

  void setForceCountDown(int value)
  {
    mForceCountDown = value;
  }

  void setPositiveSign(bool value)
  {
    mPositiveSign = value;
  }

protected:
  /// Controller
  std::unique_ptr<Controller> mController;

  /// Number of iterations before clearing a force entry
  int mForceCountDown;

  /// Whether a force should be applied in the positive or negative direction
  bool mPositiveSign;
};

class InputHandler : public ::osgGA::GUIEventHandler
{
public:
  /// Constructor
  InputHandler(TutorialWorldNode* node) : mNode(node)
  {
    // Do nothing
  }

  /// Handles key events
  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType()) {
      switch (ea.getKey()) {
        case ',':
          mNode->setForceCountDown(default_countdown);
          mNode->setPositiveSign(false);
          return true;
        case '.':
          mNode->setForceCountDown(default_countdown);
          mNode->setPositiveSign(true);
          return true;
        case 'a':
          mNode->getController()->changeWheelSpeed(default_speed_increment);
          return true;
        case 's':
          mNode->getController()->changeWheelSpeed(-default_speed_increment);
          return true;
      }
    }

    return false;
  }

protected:
  TutorialWorldNode* mNode;
};

// Load a biped model and enable joint limits and self-collision
SkeletonPtr loadBiped()
{
  // Lesson 1

  // Create the world with a skeleton
  WorldPtr world = SkelParser::readWorld("dart://sample/skel/biped.skel");
  assert(world != nullptr);

  SkeletonPtr biped = world->getSkeleton("biped");

  return biped;
}

// Load a skateboard model and connect it to the biped model via an Euler joint
void modifyBipedWithSkateboard(SkeletonPtr /*biped*/)
{
  // Lesson 5
}

// Set the actuator type for four wheel joints to "VELOCITY"
void setVelocityAccuators(SkeletonPtr /*biped*/)
{
  // Lesson 6
}

// Solve for a balanced pose using IK
Eigen::VectorXd solveIK(SkeletonPtr biped)
{
  // Lesson 7
  Eigen::VectorXd newPose = biped->getPositions();
  return newPose;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body
      = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
      new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

int main()
{
  SkeletonPtr floor = createFloor();

  // Lesson 1
  SkeletonPtr biped = loadBiped();

  // Lesson 5
  modifyBipedWithSkateboard(biped);

  // Lesson 6
  setVelocityAccuators(biped);

  // Lesson 7
  Eigen::VectorXd balancedPose = solveIK(biped);
  biped->setPositions(balancedPose);

  WorldPtr world = std::make_shared<World>();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  if (dart::dynamics::CollisionDetector::getFactory()->canCreate("bullet")) {
    world->getConstraintSolver()->setCollisionDetector(
        dart::dynamics::CollisionDetector::getFactory()->create("bullet"));
  }

  world->addSkeleton(floor);
  world->addSkeleton(biped);

  ::osg::ref_ptr<TutorialWorldNode> node = new TutorialWorldNode(world);

  // Create a viewer for rendering the world and handling user input
  dart::gui::osg::Viewer viewer;
  viewer.setWindowTitle("Biped Tutorial");

  // Add tutorial node
  viewer.addWorldNode(node);

  // Add our custom input handler to the Viewer
  viewer.addEventHandler(new InputHandler(node.get()));

  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  // Print instructions
  std::cout << "'.': forward push" << std::endl;
  std::cout << "',': backward push" << std::endl;
  std::cout << "'s': increase skateboard forward speed" << std::endl;
  std::cout << "'a': increase skateboard backward speed" << std::endl;
  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'p': replay simulation" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;
  std::cout << "'[' and ']': replay one frame backward and forward"
            << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.34f, 3.00f, 2.41f), // eye
      ::osg::Vec3(0.f, 0.f, 0.f),       // center
      ::osg::Vec3(0.f, 1.f, 0.f));      // up

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Run the Viewer
  viewer.run();
}

# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

dart_add_test("integration" test_Building)
dart_add_test("integration" test_Common)
dart_add_test("integration" test_Concurrency)
dart_add_test("integration" test_Constraint)
dart_add_test("integration" test_Frames)
dart_add_test("integration" test_Friction)
dart_add_test("integration" test_InverseKinematics)
dart_add_test("integration" test_NameManagement)

if(TARGET dart-optimization-pagmo)
  dart_add_test("integration" test_MultiObjectiveOptimization)
  target_link_libraries(test_MultiObjectiveOptimization dart-optimization-pagmo)
endif()

dart_add_test("integration" test_Distance)
if(TARGET dart-collision)
  target_link_libraries(test_Distance dart-collision)
endif()

dart_add_test("integration" test_Raycast)
if(TARGET dart-collision)
  target_link_libraries(test_Raycast dart-collision)
endif()

if(TARGET dart-collision)
  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_add_test("integration" test_ForceDependentSlip)
  target_link_libraries(test_ForceDependentSlip dart-collision)
endif()

if(TARGET dart-io)

  dart_add_test("integration" test_Collision)
  target_link_libraries(test_Collision dart-io)
  if(TARGET dart-collision)
    target_link_libraries(test_Collision dart-collision)
  endif()
  if(TARGET dart-collision)
    target_link_libraries(test_Collision dart-collision)
  endif()

  dart_add_test("integration" test_Dynamics)
  target_link_libraries(test_Dynamics dart-io)

  dart_add_test("integration" test_Joints)
  target_link_libraries(test_Joints dart-io)

  dart_add_test("integration" test_Skeleton)
  target_link_libraries(test_Skeleton dart-io)

  dart_add_test("integration" test_MetaSkeleton)
  target_link_libraries(test_MetaSkeleton dart-io)

  dart_add_test("integration" test_SoftDynamics)
  target_link_libraries(test_SoftDynamics dart-io)

  dart_add_test("integration" test_World)
  target_link_libraries(test_World dart-io)
  if(TARGET dart-collision)
    target_link_libraries(test_World dart-collision)
  endif()

  if(TARGET dart-io-urdf)
    dart_add_test("integration" test_ForwardKinematics)
    target_link_libraries(test_ForwardKinematics dart-io-urdf)
  endif()

endif()

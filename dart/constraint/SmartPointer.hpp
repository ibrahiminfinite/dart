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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES) LOSS OF
 *   USE, DATA, OR PROFITS) OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_CONSTRAINT_SMARTPOINTER_HPP_
#define DART_CONSTRAINT_SMARTPOINTER_HPP_

#include "dart/common/SmartPointer.hpp"

namespace dart {
namespace constraint {

DART_COMMON_DECLARE_SMART_POINTERS(ConstraintSolver)

DART_COMMON_DECLARE_SMART_POINTERS(ConstrainedGroup)

DART_COMMON_DECLARE_SMART_POINTERS(ConstraintBase)
DART_COMMON_DECLARE_SMART_POINTERS(ClosedLoopConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(ContactConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(SoftContactConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(JointConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(MimicMotorConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(JointCoulombFrictionConstraint)

DART_COMMON_DECLARE_SMART_POINTERS(LCPSolver)
DART_COMMON_DECLARE_SMART_POINTERS(BoxedLcpSolver)
DART_COMMON_DECLARE_SMART_POINTERS(PgsBoxedLcpSolver)
DART_COMMON_DECLARE_SMART_POINTERS(PsorBoxedLcpSolver)
DART_COMMON_DECLARE_SMART_POINTERS(JacobiBoxedLcpSolver)

DART_COMMON_DECLARE_SMART_POINTERS(DynamicJointConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(BallJointConstraint)
DART_COMMON_DECLARE_SMART_POINTERS(WeldJointConstraint)

DART_COMMON_DECLARE_SMART_POINTERS(BalanceConstraint)

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_SMARTPOINTER_HPP_

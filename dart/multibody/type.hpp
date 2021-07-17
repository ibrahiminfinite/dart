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

#pragma once

#include "dart/common/smart_pointer.hpp"

namespace dart {
namespace multibody {

// MultibodyTree
template <typename Scalar>
class InverseKinematics;

// MultibodyTree
template <typename Scalar>
class MultibodyTree;

// Node smart pointers
template <typename NodeT>
using NodePtr = common::SharedPtr<NodeT, MultibodyTree<typename NodeT::Scalar>>;
template <typename NodeT>
using NodeWeakPtr
    = common::WeakPtr<NodeT, MultibodyTree<typename NodeT::Scalar>>;

// DegreeOfFreedom
template <typename Scalar>
class DegreeOfFreedom;

// Joint
template <typename Scalar>
class Joint;
template <typename Scalar>
class FixedJoint;

// BodyNode
template <typename Scalar>
class BodyNode;
template <typename Scalar>
using BodyNodePtr = common::SharedPtr<BodyNode<Scalar>, MultibodyTree<Scalar>>;
template <typename Scalar>
using ConstBodyNodePtr
    = common::SharedPtr<const BodyNode<Scalar>, const MultibodyTree<Scalar>>;

// EndEffector
template <typename Scalar>
class EndEffector;
template <typename Scalar>
using EndEffectorPtr
    = common::SharedPtr<EndEffector<Scalar>, MultibodyTree<Scalar>>;
template <typename Scalar>
using ConstEndEffectorPtr
    = common::SharedPtr<const EndEffector<Scalar>, const MultibodyTree<Scalar>>;

} // namespace multibody
} // namespace dart

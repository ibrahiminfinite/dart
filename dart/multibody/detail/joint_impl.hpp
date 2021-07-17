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

#include "dart/multibody/body_node.hpp"
#include "dart/multibody/joint.hpp"

namespace dart {
namespace multibody {

//==============================================================================
template <typename Scalar>
Joint<Scalar>::Joint() : m_child_body_node(nullptr)
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
const BodyNode<Scalar>* Joint<Scalar>::get_parent_body_node() const
{
  return const_cast<Joint*>(this)->get_mutable_child_body_node();
}

//==============================================================================
template <typename Scalar>
BodyNode<Scalar>* Joint<Scalar>::get_mutable_parent_body_node()
{
  if (m_child_body_node) {
    m_child_body_node->get_mutable_parent_body_node();
  }

  return nullptr;
}

//==============================================================================
template <typename Scalar>
const BodyNode<Scalar>* Joint<Scalar>::get_child_body_node() const
{
  return m_child_body_node;
}

//==============================================================================
template <typename Scalar>
BodyNode<Scalar>* Joint<Scalar>::get_mutable_child_body_node()
{
  return m_child_body_node;
}

} // namespace multibody
} // namespace dart

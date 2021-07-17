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
#include "dart/multibody/fixed_joint.hpp"
#include "dart/multibody/multibody_tree.hpp"

namespace dart::multibody {

//==============================================================================
template <typename Scalar>
const math::SE3<Scalar>& BodyNode<Scalar>::get_local_pose() const
{
  return m_local_pose;
}

//==============================================================================
template <typename Scalar>
const math::SpatialMotion<Scalar>&
BodyNode<Scalar>::get_local_spatial_velocity() const
{
  return m_local_spatial_velocity;
}

//==============================================================================
template <typename Scalar>
const math::SpatialMotion<Scalar>&
BodyNode<Scalar>::get_local_spatial_acceleration() const
{
  return m_local_spatial_acceleration;
}

//==============================================================================
template <typename Scalar>
const BodyNode<Scalar>* BodyNode<Scalar>::get_parent_body_node() const
{
  return m_parent_body_node;
}

//==============================================================================
template <typename Scalar>
BodyNode<Scalar>* BodyNode<Scalar>::get_mutable_parent_body_node()
{
  return m_parent_body_node;
}

//==============================================================================
template <typename Scalar>
BodyNodePtr<Scalar> BodyNode<Scalar>::create_child_body_node()
{
  BodyNode<Scalar>* body_node = new BodyNode<Scalar>();
  const bool success
      = this->m_multibody_tree->register_body_node(body_node, this);
  if (!success) {
    delete body_node;
    return nullptr;
  }
  m_child_body_nodes.push_back(body_node);
  return BodyNodePtr<Scalar>(
      body_node, this->m_multibody_tree->get_shared_ptr());
}

//==============================================================================
template <typename Scalar>
int BodyNode<Scalar>::get_num_child_body_nodes() const
{
  return m_child_body_nodes.size();
}

//==============================================================================
template <typename Scalar>
const BodyNode<Scalar>* BodyNode<Scalar>::get_child_body_node_by_index(
    int index) const
{
  return m_child_body_nodes[index];
}

//==============================================================================
template <typename Scalar>
BodyNode<Scalar>* BodyNode<Scalar>::get_mutable_child_body_node_by_index(
    int index)
{
  return m_child_body_nodes[index];
}

} // namespace dart::multibody

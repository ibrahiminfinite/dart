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

#include <memory>

#include "dart/common/builder.hpp"
#include "dart/common/smart_pointer.hpp"
#include "dart/multibody/node.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

template <typename Scalar>
class BodyNodeBuilder;

// A BodyNode can have Nodes as children, but A Node cannot have a BodyNode as a
// child.
template <typename Scalar_>
class BodyNode : public Node<Scalar_>
{
public:
  using Scalar = Scalar_;

  friend class MultibodyTree<Scalar>;

  static BodyNodeBuilder<Scalar> Create();

  BodyNode() = default;

  // Implementation of Frame<Scalar>
  const math::SE3<Scalar>& get_local_pose() const override;
  const math::SpatialMotion<Scalar>& get_local_spatial_velocity()
      const override;
  const math::SpatialMotion<Scalar>& get_local_spatial_acceleration()
      const override;

  /// @{ @name Tree structure

  const BodyNode<Scalar>* get_parent_body_node() const;

  BodyNode<Scalar>* get_mutable_parent_body_node();

  BodyNodePtr<Scalar> create_child_body_node();

  int get_num_child_body_nodes() const;

  const BodyNode<Scalar>* get_child_body_node_by_index(int index) const;

  BodyNode<Scalar>* get_mutable_child_body_node_by_index(int index);

  /// @}

  template <typename JointT>
  const JointT* set_parent_joint()
  {
    if (m_parent_joint) {
      // multibody_tree->unregister_joint(m_parent_joint);
    }

    JointT* new_joint = new JointT();

    m_parent_joint.reset(new_joint);

    return new_joint;
  }

  const Joint<Scalar>* get_parent_joint() const
  {
    return m_parent_joint.get();
  }

  Joint<Scalar>* get_mutable_parent_joint()
  {
    return m_parent_joint.get();
  }

protected:
  BodyNode* m_parent_body_node;
  std::vector<BodyNode*> m_child_body_nodes;
  std::vector<Node<Scalar>> m_child_nodes;

  std::unique_ptr<Joint<Scalar>> m_parent_joint;

  math::SE3<Scalar> m_local_pose;
  math::SpatialMotion<Scalar> m_local_spatial_velocity;
  math::SpatialMotion<Scalar> m_local_spatial_acceleration;
  int m_index_in_multibody;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, BodyNode)

} // namespace dart::multibody

#include "dart/multibody/detail/body_node_impl.hpp"

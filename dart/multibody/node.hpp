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

#include "dart/common/builder.hpp"
#include "dart/multibody/entity.hpp"
#include "dart/multibody/frame.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

template <typename NodeDerived>
class NodeBuilder
{
public:
  using Scalar = typename NodeDerived::Scalar;

  NodeBuilder(MultibodyTree<Scalar>* multibody_tree)
    : m_multibody_tree(multibody_tree)
  {
    DART_ASSERT(m_multibody_tree);
  }

protected:
  MultibodyTree<Scalar>* m_multibody_tree;
};

/// Node represents a node in a tree.
///
/// Node can have a parent node where is can be either Node or BodyNode.
/// Node can have multiple child Nodes.
/// Node has no joint.
/// Node has no degree of freedom, so a fixed Jacobian node.
template <typename Scalar_>
class Node : public Entity<Scalar_>, public RelativeFrame<Scalar_>
{
public:
  using Scalar = Scalar_;

  Node() = default;

protected:
  explicit Node(
      MultibodyTree<Scalar>* multibody_tree, Node<Scalar>* parent = nullptr);

public:
  // Implementation of Frame<Scalar>
  void set_name(const std::string& name) override;
  const std::string& get_name() const override;

  template <typename NodeT, typename... Args>
  NodePtr<NodeT> create_child_node(Args&&... args)
  {
    auto node = new NodeT(m_multibody_tree, this, std::forward<Args>(args)...);
    // m_multibody_tree->register_node()
    // if (!success) {
    //   delete node;
    //   return nullptr;
    // }
    m_child_nodes.push_back(node);
    return NodePtr<NodeT>(node, m_multibody_tree->get_shared_ptr());
  }

  template <typename ComponentType>
  bool has() const
  {
    return false;
  }

protected:
  std::string m_name;
  MultibodyTree<Scalar>* m_multibody_tree;
  Node<Scalar>* m_parent_node;
  std::vector<Node<Scalar>*> m_child_nodes;
  std::size_t m_node_index;

private:
  friend class BodyNode<Scalar>;
};

} // namespace dart::multibody

#include "dart/multibody/detail/node_impl.hpp"

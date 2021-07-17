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

#include "dart/multibody/multibody_tree.hpp"

namespace dart {
namespace multibody {

//==============================================================================
template <typename Scalar>
MultibodyTreePtr<Scalar> MultibodyTree<Scalar>::Create(
    const MultibodyTreeConfig<Scalar>& config)
{
  std::shared_ptr<MultibodyTree<Scalar>> self(new MultibodyTree<Scalar>());
  return self->build(config) ? self : nullptr;
}

//==============================================================================
template <typename Scalar>
MultibodyTreeBuilder<Scalar> MultibodyTree<Scalar>::CreateByBuilder()
{
  return MultibodyTreeBuilder<Scalar>();
}

//==============================================================================
template <typename Scalar>
MultibodyTree<Scalar>::MultibodyTree(
    std::shared_ptr<common::Allocator> allocator)
  : m_allocator(std::move(allocator))
{
  if (!m_allocator) {
    // TODO(JS): Assign default allocator
    DART_NOT_IMPLEMENTED;
  }
}

//==============================================================================
template <typename Scalar>
MultibodyTree<Scalar>::~MultibodyTree()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
bool MultibodyTree<Scalar>::build(const MultibodyTreeConfig<Scalar>& config)
{
  DART_NOT_IMPLEMENTED;
  set_name(config.name);
  return true;
}

//==============================================================================
template <typename Scalar>
const std::string& MultibodyTree<Scalar>::get_name() const
{
  return m_name;
}

//==============================================================================
template <typename Scalar>
void MultibodyTree<Scalar>::set_name(const std::string& name)
{
  m_name = name;
}

//==============================================================================
template <typename Scalar>
ConstBodyNodePtr<Scalar> MultibodyTree<Scalar>::get_root_body_node() const
{
  return ConstBodyNodePtr<Scalar>(m_root_body_node, get_shared_ptr());
}

//==============================================================================
template <typename Scalar>
void MultibodyTree<Scalar>::set_joint_positions(
    const math::VectorX<Scalar>& /*positions*/)
{
  DART_NOT_IMPLEMENTED;
}

//==============================================================================
template <typename Scalar>
bool MultibodyTree<Scalar>::register_node(Node<Scalar>* node)
{
  (void)node;
  return true;
}

//==============================================================================
template <typename Scalar>
bool MultibodyTree<Scalar>::register_body_node(
    BodyNode<Scalar>* body_node, BodyNode<Scalar>* parent_body_node)
{
  // Prevent registering more than one root body node.
  if (parent_body_node == nullptr && m_root_body_node != nullptr) {
    DART_ERROR("Not allowed registering more than one root body node.");
    return false;
  }

  // Set up body node
  body_node->m_multibody_tree = this;
  body_node->m_parent_body_node = parent_body_node;
  body_node->m_index_in_multibody = m_body_nodes.size();

  // Register as node
  if (!register_node(body_node)) {
    return false;
  }

  // Register parent joint
  if (!register_joint(body_node->get_mutable_parent_joint())) {
    return false;
  }

  m_body_nodes.push_back(body_node);
  if (parent_body_node == nullptr) {
    m_root_body_node = body_node;
  } else {
    // parent_body_node->add_child_body_node_internal(body_node);
  }

  return true;
}

//==============================================================================
template <typename Scalar>
void print_body_node_recursive(
    std::ostream& os, const BodyNode<Scalar>* body_node, int depth)
{
  const std::string indent(" ", depth * 2);
  os << indent << "- [BodyNode]:\n";
  os << indent << "  name: " << body_node->get_name() << "\n";
  for (int i = 0; i < body_node->get_num_child_body_nodes(); ++i) {
    print_body_node_recursive(
        os, body_node->get_child_body_node_by_index(i), depth + 2);
  }
}

//==============================================================================
template <typename Scalar>
void MultibodyTree<Scalar>::print(std::ostream& os)
{
  os << "[MultibodyTree]\n";
  os << "name: " << get_name() << "\n";
  if (auto root_body_node = get_root_body_node()) {
    print_body_node_recursive(os, root_body_node.get(), 0);
  }
}

//==============================================================================
template <typename Scalar>
bool MultibodyTree<Scalar>::register_joint(Joint<Scalar>* joint)
{
  (void)joint;
  return true;
}

} // namespace multibody
} // namespace dart

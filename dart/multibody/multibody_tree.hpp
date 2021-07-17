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
#include <string>
#include <optional>

#include "dart/common/allocator.hpp"
#include "dart/common/config.hpp"
#include "dart/common/macro.hpp"
#include "dart/math/jacobian.hpp"
#include "dart/math/lie_group/se3.hpp"
#include "dart/math/type.hpp"
#include "dart/multibody/body_node.hpp"
#include "dart/multibody/entity.hpp"
#include "dart/multibody/export.hpp"
#include "dart/multibody/frame.hpp"
#include "dart/multibody/node.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

template <typename Scalar>
struct BodyNodeConfig;

template <typename Scalar>
struct JointConfig;

template <typename Scalar>
class MultibodyTreeBuilderBase;
template <typename Scalar>
class MultibodyTreeBuilder;
template <typename Scalar>
class MultibodyTreeBodyNodeBuilder;
template <typename JointT>
class MultibodyTreeJointBuilder;

template <typename Scalar>
struct BodyNodeConfig : common::Config
{
  std::string name;

  std::optional<JointConfig<Scalar>> parent_joint;
};

template <typename Scalar>
struct JointConfig : common::Config
{
  std::string name;

  std::string parent_body_node;

  std::optional<std::vector<BodyNodeConfig<Scalar>>> m_child_body_nodes;
};

template <typename Scalar>
struct MultibodyTreeConfig : common::Config
{
  std::string name;

  std::optional<std::vector<BodyNodeConfig<Scalar>>> m_body_nodes;
  std::optional<std::vector<JointConfig<Scalar>>> m_joints;

#if DART_HAVE_yaml_cpp
  bool load(const YAML::Node& node) override
  {
    bool success = false;

    (void)node;
    //    success &= load()

    return success;
  }
#endif
};

template <typename Scalar>
class MultibodyTree;
template <typename Scalar>
using MultibodyTreePtr = std::shared_ptr<MultibodyTree<Scalar>>;
template <typename Scalar>
using ConstMultibodyTreePtr = std::shared_ptr<const MultibodyTree<Scalar>>;

// Design choice:
// - Split out APIs for building tree (using the builder pattern)
template <typename Scalar_>
class MultibodyTree
  : public std::enable_shared_from_this<MultibodyTree<Scalar_>>
{
public:
  using Scalar = Scalar_;

  friend class BodyNode<Scalar>;
  friend class BodyNodeBuilder<Scalar>;
  friend class MultibodyTreeBuilder<Scalar>;
  friend class MultibodyTreeBuilderBase<Scalar>;
  friend class MultibodyTreeBodyNodeBuilder<Scalar>;
  // friend class MultibodyTreeJointBuilder<Scalar>;

  static MultibodyTreePtr<Scalar> Create(const MultibodyTreeConfig<Scalar>& config = MultibodyTreeConfig<Scalar>());
  static MultibodyTreeBuilder<Scalar> CreateByBuilder();

private:
  explicit MultibodyTree(
      std::shared_ptr<common::Allocator> allocator = nullptr);

public:
  virtual ~MultibodyTree();

  bool build(const MultibodyTreeConfig<Scalar>& config);

  /// Returns the name.
  const std::string& get_name() const;

  /// Sets the name.
  void set_name(const std::string& name);

  /// @{ @name Tree Modifiers

  int get_num_bodies() const;

  ConstBodyNodePtr<Scalar> get_root_body_node() const;

  /// @}

  /// @{ @name Joint positions

  void set_joint_positions(const math::VectorX<Scalar>& positions);

  /// @}

  /// @{ @name System Equations

  math::InertiaMatrix<Scalar> get_inertia_matrix() const
  {
    return {};
  }

  math::SpatialJacobianX<Scalar> get_spatial_jacobian() const
  {
    return {};
  }

  /// @}

  auto get_shared_ptr() const
  {
    return this->shared_from_this();
  }

  auto get_shared_ptr()
  {
    return this->shared_from_this();
  }

  bool register_node(Node<Scalar>* node);

  bool register_body_node(
      BodyNode<Scalar>* body_node,
      BodyNode<Scalar>* parent_body_node = nullptr);

  void print(std::ostream& os);

protected:
  bool register_joint(Joint<Scalar>* joint);

private:
  std::shared_ptr<common::Allocator> m_allocator;

  std::string m_name;

  BodyNode<Scalar>* m_root_body_node{nullptr};
  std::vector<BodyNode<Scalar>*> m_body_nodes;

  math::VectorX<Scalar> m_positions;
  math::VectorX<Scalar> m_veloticies;
  math::VectorX<Scalar> m_accelerations;
  math::VectorX<Scalar> m_forces;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, MultibodyTree)

template <typename Scalar>
class MultibodyTreeBuilderBase
{
public:
  using This = MultibodyTreeBuilderBase;
  using ReturnType = std::shared_ptr<MultibodyTree<Scalar>>;

  MultibodyTreeBuilderBase(std::shared_ptr<MultibodyTreeConfig<Scalar>> config)
    : m_multibody_tree_config(std::move(config))
  {
    // Do nothing
  }

  operator ReturnType()
  {
    return build();
  }

  virtual ReturnType build()
  {
    return MultibodyTree<Scalar>::Create(*m_multibody_tree_config);
  }

protected:
  std::shared_ptr<MultibodyTreeConfig<Scalar>> m_multibody_tree_config;
};

template <typename Scalar>
class MultibodyTreeBuilder : public MultibodyTreeBuilderBase<Scalar>
{
public:
  using This = MultibodyTreeBuilder;
  using Base = MultibodyTreeBuilderBase<Scalar>;
  using ReturnType = typename Base::ReturnType;

  MultibodyTreeBuilder() : Base(std::make_shared<MultibodyTreeConfig<Scalar>>())
  {
    // Do nothing
  }

  MultibodyTreeBodyNodeBuilder<Scalar> root_body_node() const
  {
    return MultibodyTreeBodyNodeBuilder<Scalar>(this->m_multibody_tree_config);
  }

  This& name(const std::string& name)
  {
    this->m_multibody_tree_config->name = name;
    return *this;
  }

private:
};

template <typename Scalar_>
class MultibodyTreeBodyNodeBuilder : public MultibodyTreeBuilderBase<Scalar_>
{
public:
  using Scalar = Scalar_;

  using Base = MultibodyTreeBuilderBase<Scalar_>;
  using ReturnType = BodyNodePtr<Scalar>;

  explicit MultibodyTreeBodyNodeBuilder(
      std::shared_ptr<MultibodyTreeConfig<Scalar>> config)
    : Base(std::move(config))
  {
    // Do nothing
  }

  template <typename JointT>
  MultibodyTreeJointBuilder<JointT> parent_joint() const
  {
    return MultibodyTreeJointBuilder<JointT>(
          this->m_multibody_tree_config);
  }

  MultibodyTreeBodyNodeBuilder<Scalar> child_body_node() const
  {
    return MultibodyTreeBodyNodeBuilder<Scalar>(
          this->m_multibody_tree_config);
  }

  MultibodyTreeBodyNodeBuilder<Scalar> end_branch() const
  {
    return MultibodyTreeBodyNodeBuilder<Scalar>(
        this->m_multibody_tree_config);
  }

  MultibodyTreeBodyNodeBuilder& name(const std::string& name)
  {
    m_body_node_config.name = name;
    return *this;
  }

  MultibodyTreeBodyNodeBuilder& with_mass(Scalar mass)
  {
    (void)mass;
    return *this;
  }

protected:
  BodyNodeConfig<Scalar> m_body_node_config;
};

// TODO(JS): Take memory allocator as a template parameter
template <typename JointT>
class MultibodyTreeJointBuilder
  : public MultibodyTreeBodyNodeBuilder<typename JointT::Scalar>
{
public:
  using Scalar = typename JointT::Scalar;

  using Base = MultibodyTreeBodyNodeBuilder<Scalar>;
  using ReturnType = BodyNodePtr<Scalar>;

  MultibodyTreeJointBuilder(
      std::shared_ptr<MultibodyTreeConfig<Scalar>> config)
    : Base(std::move(config))
  {
//    DART_ASSERT(child_body_node);
    // child_body_node->set_parent_joint
  }

  MultibodyTreeJointBuilder& name(const std::string& name)
  {
    (void)name;
    return *this;
  }

protected:
  Joint<Scalar>* m_joint{nullptr};
};

} // namespace dart::multibody

#include "dart/multibody/detail/multibody_tree_impl.hpp"

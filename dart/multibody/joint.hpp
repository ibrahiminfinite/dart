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

#include "dart/common/castable.hpp"
#include "dart/multibody/entity.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

template <typename Scalar_>
class DegreeOfFreedom;

template <typename Scalar_>
class Joint : public Entity<Scalar_>, public common::Castable<Joint<Scalar_>>
{
  friend class BodyNode<Scalar_>;
  friend class MultibodyTree<Scalar_>;

public:
  using Scalar = Scalar_;

protected:
  /// Default constructor
  Joint();

public:
  virtual const std::string& get_type() const = 0;

  /// @{ @name Tree structure

  const BodyNode<Scalar>* get_parent_body_node() const;

  BodyNode<Scalar>* get_mutable_parent_body_node();

  const BodyNode<Scalar>* get_child_body_node() const;

  BodyNode<Scalar>* get_mutable_child_body_node();

  /// @}

  /// @{ @name Degree of freedom

  // virtual const DegreeOfFreedom<Scalar>* get_mutable_dof_by_index(
  //    int index) const = 0;

  // virtual DegreeOfFreedom<Scalar>* get_mutable_dof_by_index(int index) = 0;

  /// @}

protected:
  BodyNode<Scalar>* m_child_body_node{nullptr};

private:
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, Joint)

} // namespace dart::multibody

#include "dart/multibody/detail/joint_impl.hpp"

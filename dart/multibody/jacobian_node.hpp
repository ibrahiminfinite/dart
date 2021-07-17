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
#include "dart/math/jacobian.hpp"
#include "dart/multibody/node.hpp"
#include "dart/multibody/type.hpp"

namespace dart::multibody {

template <typename Scalar_>
class JacobianNode : public Node<Scalar_>
{
public:
  using Scalar = Scalar_;

  friend class Node<Scalar>;

protected:
  explicit JacobianNode(
      MultibodyTree<Scalar>* multibody_tree, Node<Scalar>* parent_node);

public:
  /// @{ @name Jacobian

  virtual const math::SpatialJacobianX<Scalar>& get_spatial_jacobian()
      const = 0;

  /// @}

protected:
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, JacobianNode)

template <typename Scalar_>
class FixedJacobianNode : public JacobianNode<Scalar_>
{
public:
  using Scalar = Scalar_;

  friend class Node<Scalar>;

protected:
  explicit FixedJacobianNode(
      MultibodyTree<Scalar>* multibody_tree, Node<Scalar>* parent_node);

public:
  // Implementation of Frame<Scalar>
  const math::SE3<Scalar>& get_local_pose() const override;
  const math::SpatialMotion<Scalar>& get_local_spatial_velocity()
      const override;
  const math::SpatialMotion<Scalar>& get_local_spatial_acceleration()
      const override;

  // Implementation of JacobianFrame<Scalar>
  const math::SpatialJacobianX<Scalar>& get_spatial_jacobian() const final;

protected:
  math::SE3<Scalar> m_local_pose;
  math::SpatialMotion<Scalar> m_local_spatial_velocity;
  math::SpatialMotion<Scalar> m_local_spatial_acceleration;

  struct Cache
  {
    math::SpatialJacobianX<Scalar> body_jacobian;

    math::SpatialJacobianX<Scalar> global_jacobian;
  };

  mutable Cache m_cache;
};

DART_TEMPLATE_CLASS_HEADER(MULTIBODY, FixedJacobianNode)

} // namespace dart::multibody

#include "dart/multibody/detail/jacobian_node_impl.hpp"

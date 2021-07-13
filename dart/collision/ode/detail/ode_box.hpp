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

#include "dart/collision/ode/detail/ode_geom.hpp"
#include "dart/collision/ode/ode_include.hpp"
#include "dart/collision/ode/ode_type.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {
namespace detail {

template <typename S>
class OdeBox : public OdeGeom<S> {
public:
  /// Constructor
  OdeBox(const OdeObject<S>* parent, const math::Vector3<S>& size);

  /// Destructor
  ~OdeBox() override;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, OdeBox)

//==============================================================================
template <typename S>
OdeBox<S>::OdeBox(const OdeObject<S>* parent, const math::Vector3<S>& size)
  : OdeGeom<S>(parent)
{
  this->m_geom_id = dCreateBox(nullptr, size.x(), size.y(), size.z());
}

//==============================================================================
template <typename S>
OdeBox<S>::~OdeBox()
{
  dGeomDestroy(this->m_geom_id);
}

} // namespace detail
} // namespace collision
} // namespace dart

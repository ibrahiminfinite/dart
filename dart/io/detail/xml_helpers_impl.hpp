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

#include <sstream>

#include "dart/common/logging.hpp"
#include "dart/common/string.hpp"
#include "dart/io/xml_helpers.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace io {

//==============================================================================
template <typename Scalar, int N>
std::string to_string(const math::Vector<Scalar, N>& v)
{
  std::stringstream ss;
  ss << v.transpose();
  return ss.str();
}

//==============================================================================
template <typename Scalar>
std::string to_string(
    const math::Isometry3<Scalar>& v, const std::string& rotation_type)
{
  math::Vector3<Scalar> angles;
  if (rotation_type == "intrinsic") {
    angles = math::matrixToEulerXYZ(v.rotation());
  } else if (rotation_type == "extrinsic") {
    angles = math::matrixToEulerZYX(v.rotation()).reverse();
  } else {
    DART_ERROR(
        "Unsupported rotation type [{}]. Assuming intrinsic.", rotation_type);
    angles = math::matrixToEulerXYZ(v.rotation());
  }

  std::stringstream ss;
  ss.precision(6);
  ss << v.translation().transpose() << " ";
  ss << angles;

  return ss.str();
}

//==============================================================================
template <typename Scalar, int N>
math::Vector<Scalar, N> to_vector(const std::string& str)
{
  math::Vector<Scalar, N> out;

  auto pieces = common::split(common::trim(str));

  if (pieces.size() != N) {
    DART_ERROR("Invalid string [{}] to convert to vector2", str);
    return math::Vector<Scalar, N>::Zero();
  }

  for (auto i = 0u; i < N; ++i) {
    if (pieces[i].empty()) {
      DART_ERROR(
          "Invalid {}-th token of string [{}] to convert to vector{}",
          i,
          str,
          N);
      return math::Vector<Scalar, N>::Zero();
    }

    out[i] = common::to_scalar<Scalar>(pieces[i]);
  }

  return out;
}

//==============================================================================
template <typename Scalar>
math::VectorX<Scalar> to_vector_x(const std::string& str)
{
  auto pieces = common::split(common::trim(str));

  math::VectorX<Scalar> out(pieces.size());

  for (auto i = 0u; i < pieces.size(); ++i) {
    if (pieces[i].empty()) {
      DART_ERROR(
          "Invalid {}-th token of string [{}] to convert to vectorX", i, str);
      return math::VectorX<Scalar>();
    }

    out[i] = common::to_scalar<Scalar>(pieces[i]);
  }

  return out;
}

//==============================================================================
template <typename Scalar>
math::Isometry3<Scalar> to_isometry3(
    const std::string& str, const std::string& rotation_type)
{
  math::Isometry3<Scalar> out = math::Isometry3<Scalar>::Identity();

  // Parse as 6-D vector
  const math::VectorX<Scalar> vec = to_vector_x<Scalar>(str);
  if (vec.size() != 6) {
    DART_ERROR("Failed to convert [{}] to isometry3", str);
    return math::Isometry3<Scalar>::Identity();
  }

  // Parse rotation part
  if (rotation_type == "intrinsic") {
    out.linear() = math::eulerXYZToMatrix(vec.template tail<3>());
  } else if (rotation_type == "extrinsic") {
    out.linear() = math::eulerZYXToMatrix(vec.template tail<3>().reverse());
  } else {
    DART_ERROR(
        "Unsupported rotation type [{}]. Assuming intrinsic.", rotation_type);
    out.linear() = math::eulerXYZToMatrix(vec.template tail<3>());
  }

  // Parse translation part
  out.translation() = vec.template head<3>();

  return out;
}

} // namespace io
} // namespace dart

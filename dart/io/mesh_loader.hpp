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

#include "dart/common/resource_retriever.hpp"
#include "dart/common/uri.hpp"
#include "dart/io/export.hpp"
#include "dart/math/geometry/tri_mesh.hpp"

namespace dart {
namespace io {

template <typename S>
class MeshLoader
{
public:
  /// Constructor
  MeshLoader() = default;

  /// Destructor
  virtual ~MeshLoader() = default;

  /// Loads math::TriMesh given URI and resource retriever
  virtual std::shared_ptr<math::TriMesh<S>> load(
      const common::Uri& uri,
      common::ResourceRetrieverPtr resource_retriever = nullptr)
      = 0;

  /// Returns true if the MeshLoader can handle the mesh file format of given
  /// extension
  virtual bool can_load_extension(const std::string& extension) const = 0;
};

using MeshLoaderf = MeshLoader<float>;
using MeshLoaderd = MeshLoader<double>;

#if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE
extern template class DART_IO_API MeshLoader<double>;
#endif

#if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT
extern template class DART_IO_API MeshLoader<float>;
#endif

} // namespace io
} // namespace dart

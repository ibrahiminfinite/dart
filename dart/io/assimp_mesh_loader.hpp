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

#if DART_HAVE_assimp

  #pragma once

  #include <assimp/scene.h>

  #include "dart/io/mesh_loader.hpp"
  #include "dart/math/type.hpp"

namespace dart {
namespace io {

template <typename S>
class AssimpMeshLoader : public MeshLoader<S>
{
public:
  /// Constructor
  AssimpMeshLoader() = default;

  /// Destructor
  ~AssimpMeshLoader() override = default;

  // Documentation inherited
  std::shared_ptr<math::TriMesh<S>> load(
      const common::Uri& uri,
      common::ResourceRetrieverPtr resource_retriever = nullptr) override;

  // Documentation inherited
  bool can_load_extension(const std::string& extension) const override;

private:
  /// Loads math::TriMesh given Assimp scene
  std::shared_ptr<math::TriMesh<S>> load(const aiScene* ai_scene);

  /// Loads math::TriMesh recursively traveling Assimp nodes
  void load_recurse(
      const std::shared_ptr<math::TriMesh<S>>& mesh,
      const aiScene* ai_scene,
      const aiNode* ai_node,
      const math::Isometry3<S>& parent_node_pose);
};

using AssimpMeshLoaderf = AssimpMeshLoader<float>;
using AssimpMeshLoaderd = AssimpMeshLoader<double>;

  #if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE
extern template class DART_IO_API AssimpMeshLoader<double>;
  #endif

  #if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT
extern template class DART_IO_API AssimpMeshLoader<float>;
  #endif

} // namespace io
} // namespace dart

  #include "dart/io/detail/assimp_mesh_loader_impl.hpp"

#endif

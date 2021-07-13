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
#include "dart/common/logging.hpp"
#include "dart/math/geometry/tri_mesh.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace collision {
namespace detail {

template <typename S>
class OdeMesh : public OdeGeom<S> {
public:
  /// Constructor
  OdeMesh(
      const OdeObject<S>* parent,
      const math::TriMesh<S>* scene,
      const math::Vector3<S>& scale = math::Vector3<S>::Ones());

  /// Destructor
  ~OdeMesh() override;

  // Documentation inherited
  void update_engine_data() override;

private:
  void fill_arrays(
      const math::TriMesh<S>* scene,
      const math::Vector3<S>& scale = math::Vector3<S>::Ones());

private:
  /// Array of vertex values.
  std::vector<S> m_vertices;

  /// Array of normals values.
  std::vector<S> m_normals;

  /// Array of index values.
  std::vector<int> m_indices;

  /// ODE trimesh data.
  dTriMeshDataID m_ode_tri_mesh_data_id;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, OdeMesh)

//==============================================================================
template <typename S>
OdeMesh<S>::OdeMesh(
    const OdeObject<S>* parent,
    const math::TriMesh<S>* scene,
    const math::Vector3<S>& scale)
  : OdeGeom<S>(parent), m_ode_tri_mesh_data_id(nullptr)
{
  // Fill vertices, normals, and indices in the ODE friendly data structures.
  fill_arrays(scene, scale);

  /// This will hold the vertex data of the triangle mesh
  if (!m_ode_tri_mesh_data_id) {
    m_ode_tri_mesh_data_id = dGeomTriMeshDataCreate();
  }

  // Build the ODE triangle mesh
  if constexpr (std::is_same_v<S, double>) {
    dGeomTriMeshDataBuildDouble1(
        m_ode_tri_mesh_data_id,
        m_vertices.data(),
        3 * sizeof(S),
        static_cast<int>(m_vertices.size() / 3),
        m_indices.data(),
        static_cast<int>(m_indices.size()),
        3 * sizeof(int),
        m_normals.data());
  } else if constexpr (std::is_same_v<S, float>) {
    dGeomTriMeshDataBuildSingle1(
        m_ode_tri_mesh_data_id,
        m_vertices.data(),
        3 * sizeof(S),
        static_cast<int>(m_vertices.size() / 3),
        m_indices.data(),
        static_cast<int>(m_indices.size()),
        3 * sizeof(int),
        m_normals.data());
  } else {
    DART_ERROR(
        "Unsupported scalar type [{}]. Assuming double.", typeid(S).name());
    dGeomTriMeshDataBuildDouble1(
        m_ode_tri_mesh_data_id,
        m_vertices.data(),
        3 * sizeof(S),
        static_cast<int>(m_vertices.size() / 3),
        m_indices.data(),
        static_cast<int>(m_indices.size()),
        3 * sizeof(int),
        m_normals.data());
  }

  this->m_geom_id = dCreateTriMesh(
      nullptr, m_ode_tri_mesh_data_id, nullptr, nullptr, nullptr);
}

//==============================================================================
template <typename S>
OdeMesh<S>::~OdeMesh()
{
  dGeomDestroy(this->m_geom_id);

  if (m_ode_tri_mesh_data_id) {
    dGeomTriMeshDataDestroy(m_ode_tri_mesh_data_id);
  }
}

//==============================================================================
template <typename S>
void OdeMesh<S>::update_engine_data()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void OdeMesh<S>::fill_arrays(
    const math::TriMesh<S>* scene, const math::Vector3<S>& scale)
{
  m_vertices.resize(scene->get_vertices().size() * 3);
  m_normals.resize(scene->get_vertex_normals().size() * 3);
  m_indices.resize(scene->get_triangles().size() * 3);

  auto index = 0u;
  for (const auto& vertex : scene->get_vertices()) {
    m_vertices[index++] = vertex[0] * scale[0];
    m_vertices[index++] = vertex[1] * scale[1];
    m_vertices[index++] = vertex[2] * scale[2];
  }

  index = 0u;
  for (const auto& normal : scene->get_vertex_normals()) {
    m_normals[index++] = normal[0];
    m_normals[index++] = normal[1];
    m_normals[index++] = normal[2];
  }

  index = 0u;
  for (const auto& triangle : scene->get_triangles()) {
    m_indices[index++] = static_cast<int>(triangle[0]);
    m_indices[index++] = static_cast<int>(triangle[1]);
    m_indices[index++] = static_cast<int>(triangle[2]);
  }
}

} // namespace detail
} // namespace collision
} // namespace dart

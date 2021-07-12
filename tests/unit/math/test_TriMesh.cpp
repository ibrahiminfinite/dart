/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <gtest/gtest.h>

#include "dart/math/geometry/tri_mesh.hpp"

using namespace dart;
using namespace math;

//==============================================================================
TEST(TriMeshTests, DefaultConstructor)
{
  auto mesh = TriMeshd();
  EXPECT_FALSE(mesh.has_triangles());
  EXPECT_FALSE(mesh.has_triangle_normals());
  EXPECT_TRUE(mesh.is_empty());
}

//==============================================================================
TEST(TriMeshTests, set_triangles)
{
  auto mesh = TriMeshd();
  EXPECT_TRUE(mesh.is_empty());

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  auto triangles = TriMeshd::Triangles();
  triangles.emplace_back(0, 1, 2);

  mesh.set_triangles(vertices, triangles);
  EXPECT_TRUE(mesh.has_triangles());
  EXPECT_FALSE(mesh.has_triangle_normals());
  EXPECT_FALSE(mesh.is_empty());
  EXPECT_EQ(mesh.get_vertices(), vertices);
  EXPECT_EQ(mesh.get_triangles(), triangles);

  mesh.compute_vertex_normals();
  EXPECT_TRUE(mesh.has_triangles());
  EXPECT_TRUE(mesh.has_triangle_normals());
  EXPECT_FALSE(mesh.is_empty());

  mesh.clear();
  EXPECT_FALSE(mesh.has_triangles());
  EXPECT_FALSE(mesh.has_triangle_normals());
  EXPECT_TRUE(mesh.is_empty());
}

//==============================================================================
TEST(TriMeshTests, Operators)
{
  auto mesh1 = TriMeshd();
  auto mesh2 = TriMeshd();

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  auto triangles = TriMeshd::Triangles();
  triangles.emplace_back(0, 1, 2);

  mesh1.set_triangles(vertices, triangles);
  EXPECT_EQ(mesh1.get_vertices().size(), 3);
  EXPECT_EQ(mesh1.get_triangles().size(), 1);
  mesh2.set_triangles(vertices, triangles);
  EXPECT_EQ(mesh2.get_vertices().size(), 3);
  EXPECT_EQ(mesh2.get_triangles().size(), 1);

  auto mesh3 = mesh1 + mesh2;
  EXPECT_EQ(mesh3.get_vertices().size(), 6);
  EXPECT_EQ(mesh3.get_triangles().size(), 2);
  EXPECT_FALSE(mesh3.has_triangle_normals());
  EXPECT_FALSE(mesh3.has_vertex_normals());

  mesh1.compute_vertex_normals();
  EXPECT_TRUE(mesh1.has_triangle_normals());
  EXPECT_TRUE(mesh1.has_vertex_normals());
  EXPECT_FALSE(mesh2.has_triangle_normals());
  EXPECT_FALSE(mesh2.has_vertex_normals());
  EXPECT_FALSE((mesh1 + mesh2).has_triangle_normals());
  EXPECT_FALSE((mesh1 + mesh2).has_vertex_normals());

  mesh2.compute_vertex_normals();
  EXPECT_TRUE(mesh1.has_triangle_normals());
  EXPECT_TRUE(mesh1.has_vertex_normals());
  EXPECT_TRUE(mesh2.has_triangle_normals());
  EXPECT_TRUE(mesh2.has_vertex_normals());
  EXPECT_TRUE((mesh1 + mesh2).has_triangle_normals());
  EXPECT_TRUE((mesh1 + mesh2).has_vertex_normals());

  mesh1 += mesh2;
  EXPECT_EQ(mesh1.get_vertices().size(), 6);
  EXPECT_EQ(mesh1.get_triangles().size(), 2);
}

//==============================================================================
TEST(TriMeshTests, generate_convex_hull)
{
  auto mesh = TriMeshd();
  EXPECT_TRUE(mesh.is_empty());

  auto emptyConvexHull = mesh.generate_convex_hull();
  ASSERT_NE(emptyConvexHull, nullptr);
  EXPECT_TRUE(emptyConvexHull->is_empty());

  auto vertices = TriMeshd::Vertices();
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(1, 0, 0);
  vertices.emplace_back(0, 1, 0);
  vertices.emplace_back(0, 0, 1);
  mesh.set_triangles(vertices, {});

  auto convexHull = mesh.generate_convex_hull();
  ASSERT_NE(convexHull, nullptr);
  EXPECT_EQ(convexHull->get_vertices().size(), vertices.size());
  EXPECT_EQ(convexHull->get_triangles().size(), 4);
}

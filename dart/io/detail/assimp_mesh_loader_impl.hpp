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

  #include <assimp/IOStream.hpp>
  #include <assimp/IOSystem.hpp>
  #include <assimp/Importer.hpp>
  #include <assimp/cfileio.h>
  #include <assimp/cimport.h>
  #include <assimp/postprocess.h>
  #include <assimp/scene.h>

  #include "dart/common/LocalResourceRetriever.hpp"
  #include "dart/common/Resource.hpp"
  #include "dart/common/logging.hpp"
  #include "dart/common/string.hpp"
  #include "dart/io/assimp_mesh_loader.hpp"

namespace dart {
namespace io {

namespace {

//==============================================================================
template <typename S>
math::Vector<S, 3> convert(const aiVector3D& ai_vector)
{
  return math::Vector<S, 3>(ai_vector.x, ai_vector.y, ai_vector.z);
}

//==============================================================================
template <typename S>
math::Isometry3<S> convert(const aiMatrix4x4& ai_matrix)
{
  math::Isometry3<S> out = math::Isometry3<S>::Identity();

  out(0, 0) = ai_matrix.a1;
  out(0, 1) = ai_matrix.a2;
  out(0, 2) = ai_matrix.a3;
  out(0, 3) = ai_matrix.a4;

  out(1, 0) = ai_matrix.b1;
  out(1, 1) = ai_matrix.b2;
  out(1, 2) = ai_matrix.b3;
  out(1, 3) = ai_matrix.b4;

  out(2, 0) = ai_matrix.c1;
  out(2, 1) = ai_matrix.c2;
  out(2, 2) = ai_matrix.c3;
  out(2, 3) = ai_matrix.c4;

  // Assumes the last row is [0, 0, 0, 1]

  return out;
}

//==============================================================================
class AssimpInputResourceAdaptor : public Assimp::IOStream
{
public:
  /**
   * @brief Constructor.
   * @param[in] resource: The resource loaded for the mesh file.
   */
  explicit AssimpInputResourceAdaptor(
      const std::shared_ptr<common::Resource>& resource)
    : m_resource(resource)
  {
    assert(resource);
  }

  /**
   * @brief Destructor.
   */
  ~AssimpInputResourceAdaptor() override = default;

  /**
   * @brief Reads from the file.
   */
  std::size_t Read(void* buffer, std::size_t size, std::size_t count) override
  {
    return m_resource->read(reinterpret_cast<char*>(buffer), size, count);
  }

  /// \brief Not implemented. This is a read-only stream.
  std::size_t Write(
      const void* /*buffer*/,
      std::size_t /*size*/,
      std::size_t /*count*/) override
  {
    DART_WARN("Write is not implemented. This is a read-only stream.");
    return 0;
  }

  /**
   * @brief Sets the read/write cursor of the file.
   *
   * Note that the offset is negative for aiOrigin_END.
   */
  aiReturn Seek(std::size_t offset, aiOrigin in_origin) override
  {
    common::Resource::SeekType origin;
    switch (in_origin) {
      case aiOrigin_CUR:
        origin = common::Resource::SEEKTYPE_CUR;
        break;

      case aiOrigin_END:
        origin = common::Resource::SEEKTYPE_END;
        break;

      case aiOrigin_SET:
        origin = common::Resource::SEEKTYPE_SET;
        break;

      default:
        DART_WARN(
            "Invalid origin. Expected aiOrigin_CUR, aiOrigin_END, or "
            "aiOrigin_SET.");
        return aiReturn_FAILURE;
    }

    if (m_resource->seek(offset, origin)) {
      return aiReturn_SUCCESS;
    } else {
      return aiReturn_FAILURE;
    }
  }

  /**
   * @brief Returns the current position of the read/write cursor
   */
  std::size_t Tell() const override
  {
    return m_resource->tell();
  }

  /**
   * @brief Returns filesize.
   */
  std::size_t FileSize() const override
  {
    return m_resource->getSize();
  }

  /**
   * @brief Not implemented. This is a read-only stream.
   */
  void Flush() override
  {
    DART_WARN("Flush is not implemented. This is a read-only stream.");
  }

private:
  /** The resource loaded for the mesh. */
  std::shared_ptr<common::Resource> m_resource;
};

aiFileIO createFileIO(Assimp::IOSystem* adaptor);

class AssimpInputResourceRetrieverAdaptor : public Assimp::IOSystem
{
public:
  explicit AssimpInputResourceRetrieverAdaptor(
      const std::shared_ptr<common::ResourceRetriever>& resource_retriever)
    : m_resource_retriever(resource_retriever)
  {
    // Do nothing
  }

  ~AssimpInputResourceRetrieverAdaptor() override = default;

  /**
   * @brief Tests for the existence of a file at the given path.
   * @param[in] file: Path to the file.
   * @return True if there is a file with this path, else false.
   */
  bool Exists(const char* file) const override
  {
    return m_resource_retriever->exists(file);
  }

  /**
   * @brief Returns the system specific directory separator.
   * @return System specific directory separator.
   */
  char getOsSeparator() const override
  {
    // URIs always use forward slash as a delimeter.
    return '/';
  }

  /**
   * @brief Opens a new file with a given path.
   *
   * When the access to the file is finished, call Close() to release all
   * associated resources (or the virtual dtor of the IOStream).
   *
   * @param[in] file: Path to the file
   * @param[in] mode: Desired file I/O mode. Required are: "wb", "w", "wt",
   * "rb", "r", "rt".
   * @return New IOStream interface allowing the lib to access the underlying
   * file.
   */
  Assimp::IOStream* Open(const char* file, const char* mode = "rb") override
  {
    if (mode != std::string("r") && mode != std::string("rb")
        && mode != std::string("rt")) {
      DART_WARN("Failed to open a mesh file '{}'.", file);
      return nullptr;
    }

    if (const auto resource = m_resource_retriever->retrieve(file)) {
      return new AssimpInputResourceAdaptor(resource);
    } else {
      DART_WARN("Failed to retrieve a mesh from '{}'.", file);
      return nullptr;
    }
  }

  /**
   * @brief Closes the given file and releases all resources associated with it.
   * @param[in] file: The file instance previously created by Open().
   */
  void Close(Assimp::IOStream* file) override
  {
    delete file;
  }

private:
  /** The resource retriever to use to load mesh files. */
  std::shared_ptr<common::ResourceRetriever> m_resource_retriever;
};

//==============================================================================
Assimp::IOSystem* getIOSystem(aiFileIO* io)
{
  return reinterpret_cast<Assimp::IOSystem*>(io->UserData);
}

//==============================================================================
Assimp::IOStream* getIOStream(aiFile* file)
{
  return reinterpret_cast<Assimp::IOStream*>(file->UserData);
}

//==============================================================================
void fileFlushProc(aiFile* file)
{
  getIOStream(file)->Flush();
}

//==============================================================================
std::size_t fileReadProc(
    aiFile* file, char* buffer, std::size_t size, std::size_t count)
{
  return getIOStream(file)->Read(buffer, size, count);
}

//==============================================================================
aiReturn fileSeekProc(aiFile* file, std::size_t offset, aiOrigin origin)
{
  return getIOStream(file)->Seek(offset, origin);
}

//==============================================================================
std::size_t fileSizeProc(aiFile* file)
{
  return getIOStream(file)->FileSize();
}

//==============================================================================
std::size_t fileTellProc(aiFile* file)
{
  return getIOStream(file)->Tell();
}

//==============================================================================
std::size_t fileWriteProc(
    aiFile* file, const char* buffer, std::size_t size, std::size_t count)
{
  return getIOStream(file)->Write(buffer, size, count);
}

//==============================================================================
aiFile* fileOpenProc(aiFileIO* io, const char* path, const char* mode)
{
  Assimp::IOStream* stream = getIOSystem(io)->Open(path, mode);
  if (!stream)
    return nullptr;

  aiFile* out = new aiFile;
  out->FileSizeProc = &fileSizeProc;
  out->FlushProc = &fileFlushProc;
  out->ReadProc = &fileReadProc;
  out->SeekProc = &fileSeekProc;
  out->TellProc = &fileTellProc;
  out->WriteProc = &fileWriteProc;
  out->UserData = reinterpret_cast<char*>(stream);
  return out;
}

//==============================================================================
void fileCloseProc(aiFileIO* io, aiFile* file)
{
  getIOSystem(io)->Close(getIOStream(file));
  delete file;
}

//==============================================================================
aiFileIO createFileIO(Assimp::IOSystem* system)
{
  aiFileIO out;
  out.OpenProc = &fileOpenProc;
  out.CloseProc = &fileCloseProc;
  out.UserData = reinterpret_cast<char*>(system);
  return out;
}

} // namespace

//==============================================================================
template <typename S>
std::shared_ptr<math::TriMesh<S>> AssimpMeshLoader<S>::load(
    const common::Uri& url,
    std::shared_ptr<common::ResourceRetriever> resource_retriever)
{
  if (!resource_retriever) {
    resource_retriever = std::make_shared<common::LocalResourceRetriever>();
  }

  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API. Then wrap the
  // IOSystem in an aiFileIO from Assimp's C API.
  AssimpInputResourceRetrieverAdaptor systemIO(resource_retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

  // Import the file.
  const aiScene* ai_scene = aiImportFileExWithProperties(
      url.toString().c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!ai_scene) {
    DART_ERROR("Failed loading mesh '{}'.", url.toString());
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  const std::string extension
      = common::to_lower(common::get_extension(url.toString()));
  if (extension == "dae" || extension == "zae") {
    ai_scene->mRootNode->mTransformation = aiMatrix4x4();
  }

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  ai_scene = aiApplyPostProcessing(ai_scene, aiProcess_PreTransformVertices);

  // If the import failed, report it
  if (!ai_scene) {
    DART_ERROR(" Failed pre-transforming vertices.");
    return nullptr;
  }

  // Release resources
  aiReleasePropertyStore(propertyStore);

  auto mesh = load(ai_scene);

  return mesh;
}

//==============================================================================
template <typename S>
bool AssimpMeshLoader<S>::can_load_extension(const std::string& extension) const
{
  if (extension == "stl" || extension == "dae" || extension == "obj") {
    return true;
  }
  return false;
}

//==============================================================================
template <typename S>
std::shared_ptr<math::TriMesh<S>> AssimpMeshLoader<S>::load(
    const aiScene* ai_scene)
{
  auto mesh = std::make_shared<math::TriMesh<S>>();

  const aiNode* ai_root_node = ai_scene->mRootNode;

  // There will always be at least the root node if the import was successful
  DART_ASSERT(ai_root_node);

  load_recurse(mesh, ai_scene, ai_root_node, math::Isometry3<S>::Identity());

  return mesh;
}

//==============================================================================
template <typename S>
void AssimpMeshLoader<S>::load_recurse(
    const std::shared_ptr<math::TriMesh<S>>& mesh,
    const aiScene* ai_scene,
    const aiNode* ai_node,
    const math::Isometry3<S>& parent_node_pose)
{
  const math::Isometry3<S> tf
      = parent_node_pose * convert<S>(ai_node->mTransformation);

  for (auto i = 0u; i < ai_node->mNumMeshes; ++i) {
    const aiMesh* ai_mesh = ai_scene->mMeshes[ai_node->mMeshes[i]];
    const auto base_index = mesh->get_num_vertices();

    // Vertices
    mesh->reserve_vertices(ai_mesh->mNumVertices);
    for (auto j = 0u; j < ai_mesh->mNumVertices; ++j) {
      mesh->add_vertex(tf * convert<S>(ai_mesh->mVertices[j]));
    }

    // Normals
    if (ai_mesh->mNormals) {
      mesh->reserve_vertex_normals(ai_mesh->mNumVertices);
      for (auto j = 0u; j < ai_mesh->mNumVertices; ++j) {
        mesh->add_vertex_normal(
            tf.rotation() * convert<S>(ai_mesh->mNormals[j]));
      }
    }

    // Faces
    mesh->reserve_triangles(ai_mesh->mNumFaces);
    for (auto j = 0u; j < ai_mesh->mNumFaces; ++j) {
      const aiFace& face = ai_mesh->mFaces[j];
      // TODO(JS): Support more elements other than triangles
      // (i.e., mNumIndices != 3)
      DART_ASSERT(face.mNumIndices == 3);
      mesh->add_triangle(typename math::TriMesh<S>::Triangle(
          face.mIndices[0] + base_index,
          face.mIndices[1] + base_index,
          face.mIndices[2] + base_index));
    }
  }

  for (auto i = 0u; i < ai_node->mNumChildren; ++i) {
    load_recurse(mesh, ai_scene, ai_node->mChildren[i], tf);
  }
}

} // namespace io
} // namespace dart

#endif

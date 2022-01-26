/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_COMMON_OBJECTPOOL_HPP_
#define DART_COMMON_OBJECTPOOL_HPP_

#include <mutex>

#include "dart/common/MemoryAllocator.hpp"
#include "dart/common/MemoryAllocatorDebugger.hpp"
#include "dart/common/detail/ObjectMemoryStack.hpp"

namespace dart::common {

/// Memory allocator optimized for allocating many objects of the same or
/// similar sizes
template <typename T>
class ObjectPool
{
public:
  using Debug = MemoryAllocatorDebugger<ObjectPool>;

  /// Constructor
  ///
  /// \param[in] baseAllocator: (optional) Base memory allocator.
  explicit ObjectPool(
      MemoryAllocator& allocator = MemoryAllocator::GetDefault());

  /// Destructor
  ~ObjectPool();

  /// Returns the base allocator
  [[nodiscard]] const MemoryAllocator& getMemoryAllocator() const;

  /// Returns the base allocator
  [[nodiscard]] MemoryAllocator& getMemoryAllocator();

  template <typename... Args>
  [[nodiscard]] T* construct(Args&&... args) noexcept;

  void destroy(T* object) noexcept;

  // Documentation inherited
  void print(std::ostream& os = std::cout, int indent = 0) const;

private:
  // Documentation inherited
  [[nodiscard]] T* allocate() noexcept;

  // Documentation inherited
  void deallocate(T* pointer);

  bool createMemoryBlock(size_t requestedSize);

  /// The base memory allocator to allocate memory chunk
  MemoryAllocator& mMemoryAllocator;

  /// Mutex for for mNumAllocatedMemoryBlocks, mNumMemoryBlocks,
  /// mFreeMemoryUnits, and mAllocatedMemoryBlocks.
  mutable std::mutex mMutex;

  detail::ObjectMemoryBlock<T>* mFirstMemoryBlock{nullptr};

  detail::ObjectMemoryBlock<T>* mLastMemoryBlock{nullptr};

  detail::ObjectMemoryStack<T> mFreeObjects;
};

} // namespace dart::common

#include "dart/common/detail/ObjectPool-impl.hpp"

#endif // DART_COMMON_OBJECTPOOL_HPP_

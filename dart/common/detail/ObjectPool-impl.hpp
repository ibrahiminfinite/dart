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

#include <cstring>
#include <limits>

#include "dart/common/Console.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Memory.hpp"
#include "dart/common/ObjectPool.hpp"

namespace dart::common {

//==============================================================================
template <typename T>
ObjectPool<T>::ObjectPool(MemoryAllocator& baseAllocator)
  : mBaseAllocator(baseAllocator)
{
  static_assert(
      8 <= sizeof(MemoryUnit),
      "sizeof(MemoryUnit) should be equal to or greater than 8.");

  mNumAllocatedMemoryBlocks = 0;

  mMemoryBlocksSize = 64;
  mMemoryBlocks = mBaseAllocator.allocateAs<MemoryBlock>(mMemoryBlocksSize);
  const size_t allocatedSize = mMemoryBlocksSize * sizeof(MemoryBlock);
  std::memset(mMemoryBlocks, 0, allocatedSize);

  mFreeMemoryUnit = nullptr;
}

//==============================================================================
template <typename T>
ObjectPool<T>::~ObjectPool()
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  for (int i = 0; i < mNumAllocatedMemoryBlocks; ++i)
  {
    mBaseAllocator.deallocate(
        mMemoryBlocks[i].mMemoryUnits, mMemoryBlocks[i].mSize);
  }

  mBaseAllocator.deallocate(
      mMemoryBlocks, mMemoryBlocksSize * sizeof(MemoryBlock));
}

//==============================================================================
template <typename T>
const MemoryAllocator& ObjectPool<T>::getBaseAllocator() const
{
  return mBaseAllocator;
}

//==============================================================================
template <typename T>
MemoryAllocator& ObjectPool<T>::getBaseAllocator()
{
  return mBaseAllocator;
}

//==============================================================================
template <typename T>
template <typename... Args>
T* ObjectPool<T>::construct(Args&&... args) noexcept
{
  void* pointer = allocate();
  if (!pointer)
  {
    return nullptr;
  }

  // Call constructor. Return nullptr if failed.
  try
  {
    ::new (pointer) T(std::forward<Args>(args)...);
  }
  catch (...)
  {
    deallocate(pointer);
    return nullptr;
  }

  return reinterpret_cast<T*>(pointer);
}

//==============================================================================
template <typename T>
void ObjectPool<T>::print(std::ostream& os, int indent) const
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  if (indent == 0)
  {
    os << "[ObjectPool]\n";
  }
  const std::string spaces(indent, ' ');
  if (indent != 0)
  {
    // os << spaces << "type: " << getType() << "\n";
  }
  os << spaces << "base_allocator:\n";
  mBaseAllocator.print(os, indent + 2);
}

//==============================================================================
template <typename T>
void* ObjectPool<T>::allocate() noexcept
{
  constexpr size_t unitSize = sizeof(T);

  // Lock the mutex
  std::lock_guard<std::mutex> lock(mMutex);

  if (MemoryUnit* unit = mFreeMemoryUnit)
  {
    mFreeMemoryUnit = unit->mNext;
    return unit;
  }

  if (mNumAllocatedMemoryBlocks == mMemoryBlocksSize)
  {
    MemoryBlock* currentMemoryBlocks = mMemoryBlocks;
    mMemoryBlocksSize += 64;
    mMemoryBlocks = mBaseAllocator.allocateAs<MemoryBlock>(mMemoryBlocksSize);
    std::memcpy(
        mMemoryBlocks,
        currentMemoryBlocks,
        mNumAllocatedMemoryBlocks * sizeof(MemoryBlock));
    std::memset(
        mMemoryBlocks + mNumAllocatedMemoryBlocks, 0, 64 * sizeof(MemoryBlock));
  }

  MemoryBlock* newBlock = mMemoryBlocks + mNumAllocatedMemoryBlocks;

  const auto bytesToAllocate = 8 * unitSize /* + std::alignment_of_v<T>*/;
  void* startPointer = mBaseAllocator.allocate_aligned(
      bytesToAllocate, std::alignment_of_v<T>);
  const size_t startAddress = reinterpret_cast<size_t>(startPointer);

  void* memoryUnitsBegin = reinterpret_cast<void*>(startAddress);
  char* memoryUnitsBeginChar = static_cast<char*>(memoryUnitsBegin);

  newBlock->mMemoryUnits = static_cast<MemoryUnit*>(memoryUnitsBegin);

  const size_t unitCount = bytesToAllocate / unitSize;
  // TODO(JS): Replace unitCount with 8 and then mNextSize;

  DART_ASSERT(unitCount > 0);

  for (size_t i = 0u; i < unitCount - 1; ++i)
  {
    void* unitPointer = static_cast<void*>(memoryUnitsBeginChar + unitSize * i);
    void* nextUnitPointer
        = static_cast<void*>(memoryUnitsBeginChar + unitSize * (i + 1));
    MemoryUnit* unit = static_cast<MemoryUnit*>(unitPointer);
    MemoryUnit* nextUnit = static_cast<MemoryUnit*>(nextUnitPointer);
    unit->mNext = nextUnit;
  }

  void* lastUnitPointer
      = static_cast<void*>(memoryUnitsBeginChar + unitSize * (unitCount - 1));
  MemoryUnit* lastUnit = static_cast<MemoryUnit*>(lastUnitPointer);
  lastUnit->mNext = nullptr;

  mFreeMemoryUnit = newBlock->mMemoryUnits->mNext;
  mNumAllocatedMemoryBlocks++;

  return newBlock->mMemoryUnits;
}

//==============================================================================
template <typename T>
void ObjectPool<T>::deallocate(void* pointer)
{
  // Cannot deallocate nullptr
  if (pointer == nullptr)
  {
    return;
  }

  // sizeof(T);

  DART_NOT_IMPLEMENTED;
}

} // namespace dart::common

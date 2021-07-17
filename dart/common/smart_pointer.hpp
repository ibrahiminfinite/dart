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

#include <memory>

#include "dart/common/export.hpp"

namespace dart::common {

template <typename ObjectT, typename ObjectContainerT>
class SharedPtr final
{
public:
  /// Constructs a SharedPtr with no managed object.
  SharedPtr() noexcept : m_object(nullptr)
  {
    // Do nothing
  }

  /// Constructs a SharedPtr with no managed object.
  constexpr SharedPtr(std::nullptr_t) noexcept : m_object(nullptr)
  {
    // Do nothing
  }

  /// Constructs a shared pointer with @c object as the pointer to the managed
  /// object.
  ///
  /// @tparam ObjectV: Object type that can be convertible to ObjectT*
  /// @param[in] object: The object to be managed by this shared pointer.
  /// @param[in] host: The host instance that holds the object as a shared_ptr.
  template <typename ObjectV>
  explicit SharedPtr(ObjectV* object, std::shared_ptr<ObjectContainerT> host)
    : m_object(object), m_container(std::move(host))
  {
    // Do nothing
  }

  ~SharedPtr()
  {
    // Do nothing
  }

  SharedPtr& operator=(const SharedPtr& other) noexcept
  {
    m_object = other.m_object;
    m_container = other.m_container;
    return *this;
  }

  /// @{ @name Modifiers

  void reset() noexcept
  {
    m_object = nullptr;
    m_container.reset();
  }

  /// @}

  /// @{ @name Observers

  ObjectT* get() const
  {
    return m_object;
  }

  /// Implicit conversion
  operator ObjectT*() const noexcept
  {
    return m_object;
  }

  /// Dereferencing operator
  ObjectT& operator*() const noexcept
  {
    return *m_object;
  }

  /// Dereferencing operator
  ObjectT* operator->() const noexcept
  {
    return m_object;
  }

  long use_count() const noexcept
  {
    return m_container.use_count();
  }

  /// Checks if this shared pointer stores a non-null pointer.
  explicit operator bool() const noexcept
  {
    return (get() != nullptr);
  }

  std::shared_ptr<ObjectContainerT> get_shared_host() const noexcept
  {
    return m_container;
  }

  /// @}

private:
  ObjectT* m_object;
  std::shared_ptr<ObjectContainerT> m_container;
};

template <typename ObjectT, typename ObjectContainerT>
class WeakPtr final
{
public:
  /// Constructs an empty weak pointer.
  WeakPtr() noexcept : m_object(nullptr)
  {
    // Do nothing
  }

  /// Constructs a SharedPtr with no managed object.
  constexpr WeakPtr(const WeakPtr& other) noexcept
    : m_object(other.m_object), m_container(other.m_container)
  {
    // Do nothing
  }

  template <typename ObjectV>
  constexpr WeakPtr(const SharedPtr<ObjectV, ObjectContainerT>& other) noexcept
    : m_object(other), m_container(other.get_shared_host())
  {
    // Do nothing
  }

  WeakPtr& operator=(const WeakPtr& other) noexcept
  {
    m_object = other.m_object;
    m_container = other.m_container;
    return *this;
  }

  /// @{ @name Modifiers

  void reset()
  {
    m_object = nullptr;
    m_container.reset();
  }

  /// @}

  /// @{ @name Observers

  /// Returns the number of SharedPtr objects that manage the object.
  long use_count() const noexcept
  {
    return m_container.use_count();
  }

  /// Checks whether the referenced object was already deleted.
  bool expired() const noexcept
  {
    return (use_count() == 0);
  }

  /// Creates a SharedPtr that manages the referenced object
  SharedPtr<ObjectT, ObjectContainerT> lock() const noexcept
  {
    if (auto locked = m_container.lock()) {
      return SharedPtr<ObjectT, ObjectContainerT>(m_object, std::move(locked));
    }

    return SharedPtr<ObjectT, ObjectContainerT>();
  }

  /// Provides owner-based ordering of weak pointer.
  template <typename ObjectY>
  bool owner_before(const WeakPtr<ObjectY, ObjectContainerT>& other) const
      noexcept
  {
    return (m_object < other.m_object);
  }

  /// Provides owner-based ordering of weak pointer.
  template <typename ObjectY>
  bool owner_before(const SharedPtr<ObjectY, ObjectContainerT>& other) const
      noexcept
  {
    return (m_object < other.m_object);
  }

  /// @}

private:
  ObjectT* m_object;
  std::weak_ptr<ObjectContainerT> m_container;
};

} // namespace dart::common

#include "dart/common/detail/smart_pointer_impl.hpp"

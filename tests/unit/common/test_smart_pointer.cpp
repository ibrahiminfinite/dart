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

#include <gtest/gtest.h>

#include "dart/common/common.hpp"

using namespace dart::common;

class Component;
class Container;

using ComponentPtr = SharedPtr<Component, Container>;
using ComponentWeakPtr = WeakPtr<Component, Container>;

class Container : public std::enable_shared_from_this<Container>
{
public:
  static std::shared_ptr<Container> Create();

  Container();

  ~Container();

  ComponentPtr create_component();

  ComponentWeakPtr create_component_as_weak();

private:
  std::vector<std::shared_ptr<Component>> m_comps;
};

class Component
{
public:
  friend class Container;

  Component(Container* parent = nullptr);

  ~Component();

  Container* get_parent();

  std::shared_ptr<Container> get_shared_holder();

private:
  Container* m_parent;
};

std::shared_ptr<Container> Container::Create()
{
  return std::shared_ptr<Container>(new Container());
}

Container::Container()
{
  DART_DEBUG("Container::Container()");
}

Container::~Container()
{
  DART_DEBUG("Container::~Container()");
}

ComponentPtr Container::create_component()
{
  auto new_comp = std::shared_ptr<Component>(new Component(this));
  m_comps.push_back(new_comp);
  return ComponentPtr(new_comp.get(), shared_from_this());
}

ComponentWeakPtr Container::create_component_as_weak()
{
  return ComponentWeakPtr(create_component());
}

Component::Component(Container* parent) : m_parent(parent)
{
  DART_DEBUG("Component::Component()");
}

Component::~Component()
{
  DART_DEBUG("Component::~Component()");
}

Container* Component::get_parent()
{
  return m_parent;
}

std::shared_ptr<Container> Component::get_shared_holder()
{
  return m_parent->shared_from_this();
}

//==============================================================================
TEST(SmartPointerTest, SharedPtrConstructors)
{
#ifndef NDEBUG
  set_log_level(LogLevel::LL_DEBUG);
#endif

  // Default constructor
  ComponentPtr comp1;
  EXPECT_TRUE(comp1 == nullptr);
  EXPECT_TRUE(comp1.use_count() == 0);
  EXPECT_TRUE(comp1.get() == nullptr);
  EXPECT_TRUE(!comp1);
  EXPECT_TRUE(!comp1.get_shared_host());

  // Construct from nullptr
  ComponentPtr comp2(nullptr);
  EXPECT_TRUE(comp2 == nullptr);
  EXPECT_TRUE(comp2.use_count() == 0);
  EXPECT_TRUE(comp2.get() == nullptr);
  EXPECT_TRUE(!comp2);
  EXPECT_TRUE(!comp2.get_shared_host());
}

//==============================================================================
TEST(SmartPointerTest, SmartPointerLifetime)
{
#ifndef NDEBUG
  set_log_level(LogLevel::LL_DEBUG);
#endif

  {
    // Create a container holding it in std::shared_ptr.
    std::shared_ptr<Container> container = Container::Create();
    EXPECT_TRUE(container != nullptr);

    // Create a component that shares the ownership of the parent container
    ComponentPtr comp = container->create_component();
    EXPECT_TRUE(comp != nullptr);

    // Reset the shared pointer of container
    container.reset();
    EXPECT_TRUE(container == nullptr);

    // The component is still alive because it has the shared ownership of the
    // container.
    EXPECT_TRUE(comp != nullptr);
  }

  {
    // Create a container holding it in std::shared_ptr.
    std::shared_ptr<Container> container = Container::Create();
    EXPECT_TRUE(container != nullptr);

    // Create a component that doesn't share the ownership of the parent
    // container
    ComponentWeakPtr comp_weak = container->create_component();
    EXPECT_TRUE(comp_weak.lock() != nullptr);

    // Reset the shared pointer of container
    container.reset();
    EXPECT_TRUE(container == nullptr);

    // The component is also destructed when the container is destructed because
    // it doesn't have the shared ownership of the container.
    EXPECT_TRUE(comp_weak.lock() == nullptr);
  }
}

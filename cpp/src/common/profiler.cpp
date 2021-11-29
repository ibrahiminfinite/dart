/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/profiler.hpp"

#include "dart/common/macro.hpp"

namespace dart::common {

//==============================================================================
ProfileNode::ProfileNode(const std::string& name, ProfileNode* parent_node)
  : m_name(name),
    m_total_call_count(0u),
    m_stopwatch(false),
    m_recursion_count(0),
    m_parent_node(parent_node),
    m_sibling_node(nullptr),
    m_child_node(nullptr)
{
  reset();
}

//==============================================================================
ProfileNode::~ProfileNode()
{
  // Do nothing
}

//==============================================================================
const std::string& ProfileNode::get_name()
{
  return m_name;
}

//==============================================================================
ProfileNode* ProfileNode::get_or_create_child_node(const std::string& name)
{
  ProfileNode* child = m_child_node.get();
  while (child) {
    if (child->m_name == name) {
      return child;
    }
    child = child->m_sibling_node.get();
  }

  ProfileNode* new_child_node = new ProfileNode(name, this);
  new_child_node->m_sibling_node = std::move(m_child_node);
  m_child_node.reset(new_child_node);

  return new_child_node;
}

//==============================================================================
ProfileNode* ProfileNode::get_mutable_parent_node()
{
  return m_parent_node;
}

//==============================================================================
ProfileNode* ProfileNode::get_mutable_sibling_node()
{
  return m_sibling_node.get();
}

//==============================================================================
ProfileNode* ProfileNode::get_mutable_child_node()
{
  return m_child_node.get();
}

//==============================================================================
void ProfileNode::begin_block()
{
  m_total_call_count++;

  if (m_recursion_count == 0) {
    m_stopwatch.start();
  }

  m_recursion_count++;
}

//==============================================================================
bool ProfileNode::end_block()
{
  m_recursion_count--;

  if (m_recursion_count == 0 && m_total_call_count > 0) {
    m_stopwatch.stop();
  }

  return (m_recursion_count == 0);
}

//==============================================================================
void ProfileNode::reset()
{
  m_total_call_count = 0u;
  m_stopwatch.stop();
  m_stopwatch.reset();

  if (m_child_node) {
    m_child_node->reset();
  }

  if (m_sibling_node) {
    m_sibling_node->reset();
  }
}

//==============================================================================
Profiler::Profiler()
  : m_root_node("Root", nullptr),
    m_current_node(&m_root_node),
    m_frame_count(0u)
{
  // Do nothing
}

//==============================================================================
Profiler::~Profiler()
{
  // Do nothing
}

//==============================================================================
void Profiler::begin_block(const std::string& name)
{
  DART_ASSERT(m_current_node);
  if (name != m_current_node->get_name()) {
    m_current_node = m_current_node->get_or_create_child_node(name);
  }

  m_current_node->begin_block();
}

//==============================================================================
void Profiler::end_block()
{
  DART_ASSERT(m_current_node);
  if (m_current_node->end_block()) {
    m_current_node = m_current_node->get_mutable_parent_node();
  }
  DART_ASSERT(m_current_node);
}

//==============================================================================
void Profiler::print(std::ostream& os, int indent) const
{
  DART_NOT_IMPLEMENTED;
  DART_UNUSED(os, indent);
}

//==============================================================================
std::ostream& operator<<(std::ostream& os, const Profiler& profiler)
{
  profiler.print(os);
  return os;
}

//==============================================================================
ScopedProfiler::ScopedProfiler(const std::string& name, Profiler* profiler)
  : m_profiler(profiler)
{
  DART_ASSERT(profiler);
  m_profiler->begin_block(name);
}

//==============================================================================
ScopedProfiler::~ScopedProfiler()
{
  m_profiler->end_block();
}

} // namespace dart::common

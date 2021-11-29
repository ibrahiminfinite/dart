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

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "dart/common/export.hpp"
#include "dart/common/stopwatch.hpp"

#if DART_ENABLE_PROFILING

  #define DART_PROFILE(name, profiler)                                         \
    ScopedProfiler profile_sample(name, profiler)

namespace dart::common {

class DART_COMMON_API ProfileNode final
{
public:
  ProfileNode(const std::string& name, ProfileNode* parent_node);

  ~ProfileNode();

  const std::string& get_name();

  ProfileNode* get_or_create_child_node(const std::string& name);

  ProfileNode* get_mutable_parent_node();

  ProfileNode* get_mutable_sibling_node();

  ProfileNode* get_mutable_child_node();

  void begin_block();

  bool end_block();

  void reset();

private:
  struct ThreadProfiler
  {
  };

  std::string m_name;

  unsigned int m_total_call_count;

  StopwatchMS m_stopwatch;

  int m_recursion_count;

  ProfileNode* m_parent_node;

  std::unique_ptr<ProfileNode> m_sibling_node;

  std::unique_ptr<ProfileNode> m_child_node;

  std::map<std::thread::id, ThreadProfiler> m_map;

  mutable std::mutex m_mute;
};

class DART_COMMON_API ProfileNodeIterator final
{
public:
  ProfileNodeIterator()
  {
    // Do nothing
  }

  ~ProfileNodeIterator()
  {
    // Do nothing
  }

private:
};

class DART_COMMON_API Profiler final
{
public:
  using Node = ProfileNode;

  Profiler();

  ~Profiler();

  void begin_block(const std::string& name);

  void end_block();

  void print(std::ostream& os = std::cout, int indent = 0) const;

  friend std::ostream& operator<<(std::ostream& os, const Profiler& profiler);

private:
  using ClockType = std::chrono::high_resolution_clock;

  Node m_root_node;

  Node* m_current_node;

  unsigned int m_frame_count;

  std::chrono::time_point<ClockType> m_start_time;
};

class DART_COMMON_API ScopedProfiler final
{
public:
  ScopedProfiler(const std::string& name, Profiler* profiler);

  ~ScopedProfiler();

private:
  Profiler* m_profiler;
};

} // namespace dart::common

#else

  #define DART_PROFILE(name, profiler)

#endif

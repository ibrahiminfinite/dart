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

#ifndef DART_COMMON_PROFILER_HPP_
#define DART_COMMON_PROFILER_HPP_

#include <cstdio>
#include <iomanip>
#include <ostream>
#include <vector>

namespace dart {
namespace common {

class ProfilerNode;

class Profiler
{
public:
  static Profiler& getInstance()
  {
    static Profiler instance;
    return instance;
  }

  void reg(const ProfilerNode* prof)
  {
    mNodes.push_back(prof);
  }

  int enter()
  {
    return mIndent++;
  }

  void leave()
  {
    mIndent--;
  }

  std::ostream& print(std::ostream& o, int fnwidth) const;

  Profiler(const Profiler&) = delete;
  Profiler(Profiler&&) = delete;
  Profiler& operator=(const Profiler&) = delete;

private:
  std::vector<const ProfilerNode*> mNodes;

  int mIndent;

  Profiler()
  {
    // Do nothing
  }

  const std::string formatTime(double v) const
  {
    char buf[1024];
    if (v < 1e-6)
      sprintf(buf, "%8.2lfns", 1e9 * v);
    else if (v < 1e-3)
      sprintf(buf, "%8.2lfus", 1e6 * v);
    else if (v < 1)
      sprintf(buf, "%8.2lfms", 1e3 * v);
    else
      sprintf(buf, "%8.2lfs ", v);
    return std::string(buf);
  }
};

inline const Profiler& getInstance()
{
  return Profiler::getInstance();
}

class ProfilerNode
{
public:
  ProfilerNode(const char* fn, const int line, const char* func)
    : mCalls(0),
      mTotalTime(0),
      mDepth(0),
      mFilename(fn),
      mPrettyFunction(func),
      mLineNumber(line)
  {
    Profiler::getInstance().reg(this);
  }

  void enter()
  {
    mCalls++;
    if (mDepth > 0)
      mTotalTime += stop(); // suspend
    else
      mIndent = Profiler::getInstance().enter();
    mDepth++;
    start();
  }

  void leave()
  {
    mTotalTime += stop();
    mDepth--;
    if (mDepth > 0)
      start(); // resume
    else
      Profiler::getInstance().leave();
  }

  const std::string& filename() const
  {
    return mFilename;
  }

  int getLineNumber() const
  {
    return mLineNumber;
  }

  const std::string function() const
  {
    return std::string(mIndent, ' ') + mPrettyFunction;
  }

  int calls() const
  {
    return mCalls;
  }

  double totalTime() const
  {
    return mTotalTime;
  }

private:
  timespec mStartmark;
  int mCalls;
  double mTotalTime;
  int mDepth;
  int mIndent;

  void start()
  {
    clock_gettime(CLOCK_MONOTONIC_RAW, &mStartmark);
  }

  double stop()
  {
    timespec stopmark;
    clock_gettime(CLOCK_MONOTONIC_RAW, &stopmark);
    int nsec = stopmark.tv_nsec - mStartmark.tv_nsec;
    int sec = stopmark.tv_sec - mStartmark.tv_sec;
    return sec + 1e-9 * nsec;
  }

  const std::string mFilename;
  const std::string mPrettyFunction;
  const int mLineNumber;
};

template <typename T>
void profileGate(
    const char* fn, const int line, const char* func, bool enter, T*)
{
  static ProfilerNode prof(fn, line, func);
  if (enter)
    prof.enter();
  else
    prof.leave();
}

inline std::ostream& Profiler::print(std::ostream& o, int fnwidth) const
{
  o << "\n" << std::left << std::setw(fnwidth) << "function";
  o << "line	calls	   tottime	avgtime\n\n";
  for (const ProfilerNode* p : mNodes)
  {
    o << std::left << std::setfill('.') << std::setw(fnwidth) << p->function()
      << std::setfill(' ') << " " << std::setw(8) << p->getLineNumber() << " "
      << std::setw(8) << p->calls() << " " << formatTime(p->totalTime()) << " "
      << formatTime(p->totalTime() / p->calls()) << "\n";
  }
  return o;
}

inline std::ostream& operator<<(std::ostream& os, const Profiler& prof)
{
  return prof.print(os, 120);
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_PROFILER_HPP_

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

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "dart/common/config.hpp"

using namespace dart;
using namespace dart::common;

struct SubConfigA : Config
{
  std::string name;
};

struct RootConfig : Config
{
  int scalar_int;
  double scalar_double;
  std::string scalar_string;

  std::vector<SubConfigA> list_vector;
  std::set<SubConfigA> list_set;
  std::unordered_set<SubConfigA> list_unordered_set;

#if DART_HAVE_yaml_cpp
  bool load(const YAML::Node& node) override
  {
    bool success = true;
    success &= load_node(node, "scalar_int", &scalar_int);
    success &= load_node(node, "scalar_double", &scalar_double);
    success &= load_node(node, "scalar_string", &scalar_string);
    return success;
  }
#endif
};

//==============================================================================
TEST(ConfigTest, Basics)
{
  auto config = Config();
}

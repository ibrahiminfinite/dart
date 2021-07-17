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

#include "dart/common/config.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/metaprogramming.hpp"

namespace dart::common {

namespace detail {

#if DART_HAVE_yaml_cpp

// Scalar
// - RawData
// - Config
// List
// - of Scalars
// - of Lists
// - of Maps
// Map

template <typename T>
struct LoadScalarImpl;
template <typename T>
struct LoadListImpl;
template <typename T>
struct LoadDictImpl;
template <typename T>
struct LoadNodeImpl;

template <typename T>
struct LoadScalarImpl
{
  static bool run(const YAML::Node& node, const std::string& key, T* value)
  {
    // Config
    if constexpr (std::is_base_of_v<T, Config>) {
      return value->load(node[key]);
      // Sequence by std::vector
    }
    if constexpr (is_base_of_template_v<std::vector, Config>) {
      return false;
    } else {
      try {
        *value = node[key].as<T>();
        return true;
      } catch (const YAML::TypedBadConversion<T>& e) {
        (void)e;
        return false;
      }
    }

    return true;
  }
};

template <typename T>
struct LoadListImpl
{
  static bool run(const YAML::Node& node, const std::string& key, T* value)
  {
    return true;
  }
};

template <typename T>
struct LoadDictImpl
{
  static bool run(const YAML::Node& node, const std::string& key, T* value)
  {
    return true;
  }
};

template <typename T>
struct LoadYamlNodeImpl
{
  static bool run(const YAML::Node& node, const std::string& key, T* value)
  {
    if constexpr (is_base_of_template_v<std::vector, T>) {
      return LoadListImpl<T>::run(node, key, value);
    }
    return true;
  }
};

#endif

} // namespace detail

template <typename T>
bool Config::load_node(const YAML::Node& node, const std::string& key, T* value)
{
  return detail::LoadYamlNodeImpl<T>::run(node, key, value);
}

} // namespace dart::common

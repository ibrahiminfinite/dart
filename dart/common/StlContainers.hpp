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

#ifndef DART_COMMON_STLCONTAINERS_HPP_
#define DART_COMMON_STLCONTAINERS_HPP_

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dart/common/StlAllocator.hpp"

namespace dart::common {

template <class T, class Allocator = StlAllocator<T>>
using vector = std::vector<T, StlAllocator<T>>;

template <
    class Key,
    class Compare = std::less<Key>,
    class Allocator = StlAllocator<Key>>
using set = std::set<Key, Compare, Allocator>;

template <
    class Key,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = StlAllocator<Key>>
using unordered_set = std::unordered_set<Key, Hash, KeyEqual, Allocator>;

template <
    class Key,
    class T,
    class Compare = std::less<Key>,
    class Allocator = StlAllocator<std::pair<const Key, T>>>
using map = std::map<Key, T, Compare, Allocator>;

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = StlAllocator<std::pair<const Key, T>>>
using unordered_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

} // namespace dart::common

#endif // DART_COMMON_STLCONTAINERS_HPP_

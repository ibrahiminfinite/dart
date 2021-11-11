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

#include <array>

namespace dart::math::detail {

//==============================================================================
template <typename... T>
constexpr auto accumulate(T... args)
{
  return (args + ... + 0);
}

//==============================================================================
template <int N, int i, int j, int... Args>
struct compute_indices_impl
{
  static constexpr std::array<int, sizeof...(Args) + 1> run()
  {
    return compute_indices_impl<N - 1, i + j, Args..., i + j>::get();
  }
};

//==============================================================================
template <int i, int j, int... Args>
struct compute_indices_impl<1, i, j, Args...>
{
  static constexpr std::array<int, sizeof...(Args) + 1> run()
  {
    return {{0, Args...}};
  }
};

//==============================================================================
template <int... Args>
constexpr std::array<int, sizeof...(Args)> compute_indices()
{
  return compute_indices_impl<sizeof...(Args), 0, Args...>::run();
}

} // namespace dart::math::detail

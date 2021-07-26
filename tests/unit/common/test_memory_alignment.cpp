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

#include "dart/common/compiler.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/memory_alignment.hpp"

using namespace dart::common;

//==============================================================================
TEST(MemoryAlignmentTest, UtilityFunctions)
{
  EXPECT_FALSE(is_power_of_two(0u));
  EXPECT_TRUE(is_power_of_two(1u));
  EXPECT_TRUE(is_power_of_two(2u));
  EXPECT_FALSE(is_power_of_two(3u));
  EXPECT_TRUE(is_power_of_two(4u));
  EXPECT_FALSE(is_power_of_two(5u));
  EXPECT_FALSE(is_power_of_two(6u));
  EXPECT_FALSE(is_power_of_two(7u));
  EXPECT_TRUE(is_power_of_two(8u));
  EXPECT_FALSE(is_power_of_two(9u));
  EXPECT_FALSE(is_power_of_two(10u));

  EXPECT_EQ(next_power_of_2(0u), 1);
  EXPECT_EQ(next_power_of_2(1u), 1);
  EXPECT_EQ(next_power_of_2(2u), 2);
  EXPECT_EQ(next_power_of_2(3u), 4);
  EXPECT_EQ(next_power_of_2(4u), 4);
  EXPECT_EQ(next_power_of_2(5u), 8);
  EXPECT_EQ(next_power_of_2(6u), 8);
  EXPECT_EQ(next_power_of_2(7u), 8);
  EXPECT_EQ(next_power_of_2(8u), 8);
  EXPECT_EQ(next_power_of_2(9u), 16);
  EXPECT_EQ(next_power_of_2(10u), 16);

  EXPECT_EQ(log2ui(1u), 0);
  EXPECT_EQ(log2ui(2u), 1);
  EXPECT_EQ(log2ui(3u), 1);
  EXPECT_EQ(log2ui(4u), 2);
  EXPECT_EQ(log2ui(5u), 2);
  EXPECT_EQ(log2ui(6u), 2);
  EXPECT_EQ(log2ui(7u), 2);
  EXPECT_EQ(log2ui(8u), 3);
  EXPECT_EQ(log2ui(9u), 3);
  EXPECT_EQ(log2ui(10u), 3);

  EXPECT_EQ(log2ui_ceil(1u), 0);
  EXPECT_EQ(log2ui_ceil(2u), 1);
  EXPECT_EQ(log2ui_ceil(3u), 2);
  EXPECT_EQ(log2ui_ceil(4u), 2);
  EXPECT_EQ(log2ui_ceil(5u), 3);
  EXPECT_EQ(log2ui_ceil(6u), 3);
  EXPECT_EQ(log2ui_ceil(7u), 3);
  EXPECT_EQ(log2ui_ceil(8u), 3);
  EXPECT_EQ(log2ui_ceil(9u), 4);
  EXPECT_EQ(log2ui_ceil(10u), 4);
}

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

#include <string>

#include "dart/config.hpp"

namespace dart::common {

std::string to_upper(std::string str);
void to_upper_in_place(std::string& str);
std::string to_lower(std::string str);
void to_lower_in_place(std::string& str);

std::string trim(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

std::string trim_left(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

std::string trim_right(
    const std::string& s, const std::string& whitespaces = " \n\r\t");

std::vector<std::string> split(
    const std::string& s, const std::string& delims = " ");

std::string to_string(bool v);
std::string to_string(char v);
std::string to_string(int v);
std::string to_string(long v);
std::string to_string(long long v);
std::string to_string(unsigned v);
std::string to_string(unsigned long v);
std::string to_string(unsigned long long v);
std::string to_string(float v);
std::string to_string(double v);
std::string to_string(long double v);

bool to_bool(const std::string& str);
char to_char(const std::string& str);
int to_int(const std::string& str);
unsigned int to_uint(const std::string& str);
long to_long(const std::string& str);
long long to_long_long(const std::string& str);
float to_float(const std::string& str);
double to_double(const std::string& str);

template <typename S>
S to_scalar(const std::string& str);

} // namespace dart::common

#include "dart/common/detail/string_impl.hpp"

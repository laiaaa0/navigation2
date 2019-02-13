// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/lifecycle.hpp"
#include <cstring>
#include "gtest/gtest.h"

using nav2_util::Split;
using nav2_util::Tokens;

TEST(Split, SplitFunction)
{
  ASSERT_EQ(Split("", ':'), Tokens({""}));
  ASSERT_EQ(Split("foo", ':'), Tokens{"foo"});
  ASSERT_EQ(Split("foo:bar", ':'), Tokens({"foo", "bar"}));
  ASSERT_EQ(Split("foo:bar:", ':'), Tokens({"foo", "bar", ""}));
  ASSERT_EQ(Split(":", ':'), Tokens({"", ""}));
  ASSERT_EQ(Split("foo::bar", ':'), Tokens({"foo", "", "bar"}));
}

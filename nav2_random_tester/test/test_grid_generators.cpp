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

#include "gtest/gtest.h"

#include <vector>
#include "nav2_random_tester/grid_generators.hpp"

namespace nav2_random_tester
{

// Grid GenerateRandomGrid(size_t x, size_t y, double)
// {
//   Grid g;
//   g.x = x;
//   g.y = y;
//   return g;
// }

}

using namespace nav2_random_tester;

// These should not compile. Uncomment and test individually if mucking with
// constructors or assignment operators
// TEST(grid, copynotallowed) {
//   Grid a(2, 3);
//   Grid b(a);
// }
//
// TEST(grid, assignmentnotallowed) {
//   Grid a(2, 3);
//   Grid b(5, 6);
//   a = b;
// }

TEST(grid, construction)
{
  size_t size_x = 10;
  size_t size_y = 20;
  Grid grid(size_x, size_y);
  ASSERT_EQ(grid.size(), std::make_tuple(size_x, size_y));
}

TEST(grid, GridSize_construction)
{
  size_t size_x = 10;
  size_t size_y = 20;
  Grid grid(Grid::GridSize(size_x, size_y));
  ASSERT_EQ(grid.size_x(), size_x);
  ASSERT_EQ(grid.size_y(), size_y);
}

TEST(grid, assignment)
{
  Grid g(2, 3);
  g.cell(1,1) = 5.0;
  ASSERT_EQ(g.cell(1,1), 5.0);
}

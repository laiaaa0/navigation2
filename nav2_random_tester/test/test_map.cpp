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

#include "nav2_random_tester/grid_generators.hpp"

using namespace nav2_random_tester;

class Map
{
public:
  Map(double x, double y, double resolution) :
    x(x), y(y), resolution(resolution) {}
private:
  double x;
  double y;
  double resolution;
  // Grid map_grid;
};

TEST(creation, creation) {
  Map m(10,10,0.1);
}

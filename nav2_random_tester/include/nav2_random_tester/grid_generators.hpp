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

#ifndef NAV2_RANDOM_TESTER__GRID_GENERATORS_HPP_
#define NAV2_RANDOM_TESTER__GRID_GENERATORS_HPP_

#include <tuple>

namespace nav2_random_tester
{

class Grid
{
public:
  typedef std::tuple<size_t, size_t> GridSize;

  Grid(size_t size_x, size_t size_y)
    : x(size_x), y(size_y) {allocate_memory();}
  Grid(GridSize size)
    : x(std::get<0>(size)), y(std::get<1>(size)) {allocate_memory();}
  virtual ~Grid() {delete grid_data;}

  size_t size_x() {return x;}
  size_t size_y() {return y;}
  GridSize size() {return GridSize(x, y);}

protected:
  void allocate_memory() {grid_data = new double[x*y];}
  size_t x;
  size_t y;
  double * grid_data;
};

Grid GenerateRandomGrid(size_t x, size_t y, double density);

}  // namespace nav2_random_tester

#endif  // NAV2_RANDOM_TESTER__GRID_GENERATORS_HPP_

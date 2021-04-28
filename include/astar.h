#pragma once
#include <iostream>
#include <math.h>
#include <memory>
#include <random>
#include <stdio.h>
#include <vector>
#include <unordered_set>
#include "state.h"
#include "motionPrimitives.h"


bool astar(
  StatePtr startNode,
  StatePtr goalNode,
  vector<StatePtr>& path,
  const vector<MP>& mprims,
  const Map* map);
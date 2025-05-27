#pragma once
#include "Eigen/Core"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
//
#include <ompl/base/spaces/SE2StateSpace.h>
#include <yaml-cpp/yaml.h>
//
#include <boost/heap/d_ary_heap.hpp>
#include <boost/program_options.hpp>
// OMPL
#include "dynobench/dyno_macros.hpp"
#include "dynobench/motions.hpp"
#include "dynoplan/ompl/robots.h"
#include "ompl/base/ScopedState.h"
#include <fcl/fcl.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include "dynobench/robot_models_base.hpp"
// #include "dynoplan/tdbastar/tdbastar.hpp"

namespace dynoplan {
bool pibt(std::vector<dynobench::TrajWrapper> traj_wrappers,
          dynobench::Model_robot &robot, dynobench::Trajectory &traj_out,
          Eigen::Ref<Eigen::VectorXd> x0);

} // namespace dynoplan

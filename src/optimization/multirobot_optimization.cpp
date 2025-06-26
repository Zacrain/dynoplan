#include <string>
#include "dynobench/motions.hpp"
#include <vector>
#include <dynoplan/optimization/ocp.hpp>
#include <dynobench/multirobot_trajectory.hpp>
#include <dynoplan/optimization/multirobot_optimization.hpp>


bool execute_optimizationMultiRobot(const std::string &env_file,
                           const std::string &initial_guess_file,
                           const std::string &output_file,
                           const std::string &dynobench_base,
                           bool sum_robots_cost) {

  using namespace dynoplan;
  using namespace dynobench;

  Options_trajopt options_trajopt;
  Problem problem(env_file);

  MultiRobotTrajectory init_guess_multi_robot;
  init_guess_multi_robot.read_from_yaml(initial_guess_file.c_str());

  // Check for empty trajectories (trivial solutions)
  bool has_empty_trajectory = false;
  for (const auto& traj : init_guess_multi_robot.trajectories) {
    if (traj.actions.empty()) {
      has_empty_trajectory = true;
      break;
    }
  }

  if (has_empty_trajectory) {
    std::cout << "Skipping optimization for trivial solution (empty actions)" << std::endl;
    // Simply copy the input to output for trivial cases
    init_guess_multi_robot.to_yaml_format(output_file.c_str());
    return true;
  }

  std::vector<int> goal_times(init_guess_multi_robot.trajectories.size());

  std::transform(init_guess_multi_robot.trajectories.begin(),
                 init_guess_multi_robot.trajectories.end(), goal_times.begin(),
                 [](const Trajectory &traj) { return traj.states.size(); });

  // Trajectory init_guess;  // Seems to be unused.

  std::cout << "goal times are " << std::endl;

  for (auto &t : goal_times) {
    std::cout << t << std::endl;
  }

  if (sum_robots_cost) {
    std::cout
        << "warning: new approach where each robot tries to reach the goal fast"
        << std::endl;
    problem.goal_times = goal_times;
  }

  else {
    std::cout
        << "warning: old apprach, robots will reach the goals at the same time "
        << std::endl;
  }

  options_trajopt.solver_id = 1; // static_cast<int>(SOLVER::traj_opt);
  options_trajopt.control_bounds = 1;
  options_trajopt.use_warmstart = 1;
  options_trajopt.weight_goal = 100;
  options_trajopt.max_iter = 50;
  problem.models_base_path = dynobench_base + std::string("models/");

  Result_opti result;
  Trajectory sol;

  dynobench::Trajectory init_guess_joint =
      init_guess_multi_robot.transform_to_joint_trajectory();
  init_guess_joint.to_yaml_format("/tmp/check2.yaml");

  // Ensure the joint trajectory has the correct start and goal from the problem
  // The transform_to_joint_trajectory() function doesn't preserve this information
  if (problem.start.size() > 0) {
    init_guess_joint.start = problem.start;
  }
  if (problem.goal.size() > 0) {
    init_guess_joint.goal = problem.goal;
  }

  trajectory_optimization(problem, init_guess_joint, options_trajopt, sol,
                          result);
  if (!result.feasible) {
    std::cout << "optimization infeasible" << std::endl;
    return false;
  }

  std::cout << "optimization done! " << std::endl;
  std::vector<int> index_time_goals;

  if (problem.goal_times.size()) {
    index_time_goals = sol.multi_robot_index_goal;
  } else {
    size_t num_robots = init_guess_multi_robot.get_num_robots();
    index_time_goals = std::vector<int>(num_robots, sol.states.size());
  }

  MultiRobotTrajectory multi_out = from_joint_to_indiv_trajectory(
      sol, init_guess_multi_robot.get_nxs(), init_guess_multi_robot.get_nus(),
      index_time_goals);

  // Preserve start/goal and feasibility information from the optimized joint trajectory
  // The from_joint_to_indiv_trajectory function only copies states/actions, not metadata
  std::vector<int> nxs = init_guess_multi_robot.get_nxs();
  size_t start_idx = 0;
  for (size_t i = 0; i < multi_out.trajectories.size(); i++) {
    // Set start and goal for each robot from the joint trajectory's start/goal
    if (sol.start.size() > 0) {
      multi_out.trajectories[i].start = sol.start.segment(start_idx, nxs[i]);
    }
    if (sol.goal.size() > 0) {
      multi_out.trajectories[i].goal = sol.goal.segment(start_idx, nxs[i]);
    }

    // Copy feasibility and cost information from the joint trajectory
    multi_out.trajectories[i].cost = sol.cost;
    multi_out.trajectories[i].feasible = sol.feasible;
    multi_out.trajectories[i].traj_feas = sol.traj_feas;
    multi_out.trajectories[i].goal_feas = sol.goal_feas;
    multi_out.trajectories[i].start_feas = sol.start_feas;
    multi_out.trajectories[i].col_feas = sol.col_feas;
    multi_out.trajectories[i].x_bounds_feas = sol.x_bounds_feas;
    multi_out.trajectories[i].u_bounds_feas = sol.u_bounds_feas;

    // Copy distance metrics
    multi_out.trajectories[i].max_jump = sol.max_jump;
    multi_out.trajectories[i].max_collision = sol.max_collision;
    multi_out.trajectories[i].goal_distance = sol.goal_distance;
    multi_out.trajectories[i].start_distance = sol.start_distance;
    multi_out.trajectories[i].x_bound_distance = sol.x_bound_distance;
    multi_out.trajectories[i].u_bound_distance = sol.u_bound_distance;

    start_idx += nxs[i];
  }

  multi_out.to_yaml_format("/tmp/check5.yaml");
  multi_out.to_yaml_format(output_file.c_str());

  return true;

}

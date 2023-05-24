// #include <fstream>
#include "planner_manager.h"
#include <thread>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "destory manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(cv::Mat& map)
  {
    /* read algorithm parameters */
    // 速度和加速度限制， 需要同步
    pp_.max_vel_ = 1.5;
    pp_.max_acc_ = 6.0;
    pp_.feasibility_tolerance_ = 0.0;
    pp_.polyTraj_piece_length = 1.5;
    pp_.planning_horizon_ = 7.5;
    pp_.use_multitopology_trajs = false;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(map);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam();
    ploy_traj_opt_->setEnvironment(grid_map_);
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector2d &start_pt, const Eigen::Vector2d &start_vel,
      const Eigen::Vector2d &start_acc, const Eigen::Vector2d &local_target_pt,
      const Eigen::Vector2d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    Time t_start = Now();
    chrono::duration<double> t_init, t_opt;

    static int count = 0;
    // cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;
    // if ((start_pt - local_target_pt).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;
    cout << "init estimate ts:: " << ts << " s" << endl;

    poly_traj::MinJerkOpt init_MJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, init_MJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = init_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());

    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, init_MJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = Now() - t_start;

    std::vector<Eigen::Vector2d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));

    t_start = Now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector2d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    // printf("BBBB");

    if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      poly_traj::Trajectory initTraj = init_MJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 2, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        ploy_traj_opt_->setUseMultitopologyTrajs(true);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector2d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
      }

      t_opt = Now() - t_start;

      if (trajs.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
      }

      // visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {

      poly_traj::Trajectory initTraj = init_MJO.getTraj();

      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 2, PN - 1);
      Eigen::Matrix<double, 2, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = Now() - t_start;
    }

    /*** STEP 3: Store and display results ***/
    printf("Success=\033[42m %s",  (flag_success ? "yes \033[0m\n" : "no \033[0m\n"));
    if (flag_success)
    {
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).count();
      count_success++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).count() * 1000, t_init.count() * 1000, t_opt.count() * 1000, sum_time / count_success * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_time / count_success << endl;

      setLocalTrajFromOpt(best_MJO, touch_goal);
      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      // visualization_->displayOptimalList(cstr_pts, 0);

      continous_failures_count_ = 0;
    }
    else
    {
      cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      // visualization_->displayFailedList(cstr_pts, 0);
      setLocalTrajFromOpt(ploy_traj_opt_->getMinJerkOpt(), touch_goal);
      continous_failures_count_++;
    }

    return flag_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector2d &start_pt, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
      const Eigen::Vector2d &local_target_pt, const Eigen::Vector2d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &init_MJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix<double, DIME_SIZE, 3> headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_duration_vec;
      int piece_nums;
      constexpr double init_total_duration = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector2d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        // 确定式内点生成
        if (innerPs.cols() != 0)
        {
          printf("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_duration_vec.resize(1);
        piece_duration_vec(0) = init_total_duration;
      }
      else
      {
        // 随机式内点生成
        innerPs.resize(2, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt) * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_duration_vec.resize(2);
        piece_duration_vec = Eigen::Vector2d(init_total_duration / 2, init_total_duration / 2);
      }

      /* generate the init of init trajectory */
      init_MJO.reset(headState, tailState, piece_nums);
      init_MJO.generate(innerPs, piece_duration_vec);
      poly_traj::Trajectory initTraj = init_MJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_duration = init_total_duration / (double)piece_nums;
      piece_duration_vec.resize(piece_nums);
      piece_duration_vec = Eigen::VectorXd::Constant(piece_nums, ts);


      innerPs.resize(2, piece_nums - 1);
      int id = 0;
      double t_s = piece_duration, t_e = init_total_duration - piece_duration / 2;
      for (double t = t_s; t < t_e; t += piece_duration)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }

      if (id != piece_nums - 1)
      {
        printf("Should not happen! innerPs calculation\n");
        return false;
      }
      init_MJO.reset(headState, tailState, piece_nums);
      init_MJO.generate(innerPs, piece_duration_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.global_t_last_local_target < 0.0)
      {
        printf("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_local_traj = toSec(Now()) - traj_.local_traj.start_time;
      double t_to_local_end = traj_.local_traj.duration - passed_t_on_local_traj;
      if (t_to_local_end < 0)
      {
        printf("t_to_local_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_local_target = t_to_local_end +
                                 (traj_.global_traj.global_t_local_target - traj_.global_traj.global_t_last_local_target);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix<double, DIME_SIZE, 3> headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_duration_vec = Eigen::VectorXd::Constant(piece_nums, t_to_local_target / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector2d::Zero();

      double t = piece_duration_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_local_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_local_traj);
        }
        else if (t <= t_to_local_target)
        {
          double glb_t = t - t_to_local_end + traj_.global_traj.global_t_last_local_target - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          printf("Should not happen! x_x 0x88 t=%.2f, t_to_local_end=%.2f, t_to_local_target=%.2f", t, t_to_local_end, t_to_local_target);
        }

        t += piece_duration_vec(i + 1);
      }

      init_MJO.reset(headState, tailState, piece_nums);
      init_MJO.generate(innerPs, piece_duration_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizon, const Eigen::Vector2d &start_pt,
      const Eigen::Vector2d &global_end_pt, Eigen::Vector2d &local_target_pos,
      Eigen::Vector2d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;
cout<<__LINE__<<" get local target"<<endl;
    traj_.global_traj.global_t_last_local_target = traj_.global_traj.global_t_local_target;

    double t_step = planning_horizon / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;

    for (t = traj_.global_traj.global_t_local_target;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      cout<<__LINE__<<" get local target"<<endl;
      Eigen::Vector2d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
          cout<<__LINE__<<" get local target"<<endl;
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizon)
      {
        local_target_pos = pos_t;
        traj_.global_traj.global_t_local_target = t;
        break;
      }
    }
cout<<__LINE__<<"get local target"<<endl;
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.global_t_local_target = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }
cout<<__LINE__<<"get local target"<<endl;
    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector2d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      cout<<"set local traj"<<endl;
      traj_.setLocalTraj(traj, pts_to_check, toSec(Now()));
    }

    return ret;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector2d stop_pos)
  {
    auto ZERO = Eigen::Vector2d::Zero();
    Eigen::Matrix<double, 2, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 3);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel,
      const Eigen::Vector2d &start_acc, const std::vector<Eigen::Vector2d> &waypoints,
      const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 2, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        printf("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        printf("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    traj_.setGlobalTraj(globalMJO.getTraj(), toSec(Now()));

    return true;
  }

} // namespace ego_planner

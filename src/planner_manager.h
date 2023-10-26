#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include "grid_map.h"
#include "plan_container.hpp"
#include "poly_traj_optimizer.h"
#include "poly_traj_utils.hpp"

namespace ego_planner
{

// Fast Planner Manager
// Key algorithms of mapping and planning are called

class EGOPlannerManager
{
    // SECTION stable
public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(TMapData &map);
    bool computeInitState(
        const Eigen::Vector2d &start_pt, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &start_acc, const Eigen::Vector2d &local_target_pt,
        const Eigen::Vector2d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO);
    bool reboundReplan(
        const Eigen::Vector2d &start_pt, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pt,
        const Eigen::Vector2d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal);
    bool calculateGlobalTrajWaypoints(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel,
                                      const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pos,
                                      const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel,
        const Eigen::Vector2d &start_acc, const std::vector<Eigen::Vector2d> &waypoints,
        const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);
    void getLocalTarget(
        const double planning_horizon,
        const Eigen::Vector2d &start_pt, const Eigen::Vector2d &global_end_pt,
        Eigen::Vector2d &local_target_pos, Eigen::Vector2d &local_target_vel,
        bool &touch_goal);
    bool EmergencyStop(Eigen::Vector2d stop_pos);

    bool setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal);

    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;
    std::vector<Eigen::Vector2d> astar_path_;

private:
    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};

public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
};
} // namespace ego_planner

#endif
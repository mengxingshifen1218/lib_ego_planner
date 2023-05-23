#include "planner_manager.h"
#include <thread>

using namespace std;
/* planning utils */
ego_planner::EGOPlannerManager::Ptr planner_manager_;

// TODO 指定
Eigen::Vector2d start_pt_, start_vel_, start_acc_; // start state
Eigen::Vector2d final_goal_;                       // goal state

Eigen::Vector2d local_target_pt_, local_target_vel_; // local target state
Eigen::Vector2d odom_pos_, odom_vel_, odom_acc_;     // odometry state

double planning_horizon_ = 7.5;
bool have_new_target_ = false;
bool touch_goal_ = false;

bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{

    planner_manager_->getLocalTarget(
        planning_horizon_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);
    cout<<"local target: "<< local_target_pt_.transpose()<<endl;
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    return plan_success;
}

bool planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
{

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init = false;
    // // 第一次调用执行状态
    // if (timesOfConsecutiveStateCalls().first == 1)
    //     flag_random_poly_init = false;
    // else
    //     flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
        if (callReboundReplan(true, flag_random_poly_init))
        {
            return true;
        }
    }
    return false;
}

int main()
{
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules();
    start_pt_  = Eigen::Vector2d(0.0, 0.0);
    start_vel_ = Eigen::Vector2d(0.0, 0.0);
    start_acc_ = Eigen::Vector2d(0.0, 0.0);

    final_goal_ = Eigen::Vector2d(4.0, 4.0);
    Time start = Now();

    planFromGlobalTraj(1);

    Time end = Now();
    chrono::duration<double> t_init = end - start;

    printf("optimize cost: %f s  \n", t_init.count());
    return 0;
}
// bool planFromLocalTraj(const int trial_times /*=1*/)
// {

//     LocalTrajData *info = &planner_manager_->traj_.local_traj;
//     double t_cur = toSec(Now()) - info->start_time;

//     start_pt_ = info->traj.getPos(t_cur);
//     start_vel_ = info->traj.getVel(t_cur);
//     start_acc_ = info->traj.getAcc(t_cur);

//     bool success = callReboundReplan(false, false);

//     if (!success)
//     {
//         success = callReboundReplan(true, false);
//         if (!success)
//         {
//             for (int i = 0; i < trial_times; i++)
//             {
//                 success = callReboundReplan(true, true);
//                 if (success)
//                     break;
//             }
//             if (!success)
//             {
//                 return false;
//             }
//         }
//     }

//     return true;
// }
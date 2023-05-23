#include <eigen3/Eigen/Core>
#include <iostream>

using namespace std;
int main()
{
    Eigen::Matrix<double, 2, 3> headState, tailState;
    Eigen::MatrixXd innerPs;
    Eigen::VectorXd piece_dur_vec;
    Eigen::Vector2d start_pt, start_vel, start_acc;
    Eigen::Vector2d local_target_pt, local_target_vel;
    headState << start_pt, start_vel, start_acc;
    tailState << local_target_pt, local_target_vel, Eigen::Vector2d::Zero();
    cout << "headState: " << headState << endl;
}
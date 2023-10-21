#include "maps.h"
#include "planner_manager.h"
#include <thread>

using namespace std;
using namespace cv;
/* planning utils */
ego_planner::EGOPlannerManager::Ptr planner_manager_;

// TODO 指定
Eigen::Vector2d start_pt_, start_vel_, start_acc_; // start state
Eigen::Vector2d final_goal_;                       // goal state

Eigen::Vector2d local_target_pt_, local_target_vel_; // local target state
Eigen::Vector2d odom_pos_, odom_vel_, odom_acc_;     // odometry state

double planning_horizon_ = 7.5;
bool have_new_target_    = false;
bool touch_goal_         = false;

constexpr bool use_img_flag = true;
bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{

    // planner_manager_->getLocalTarget(
    //     planning_horizon_, start_pt_, final_goal_,
    //     local_target_pt_, local_target_vel_,
    //     touch_goal_);
    local_target_pt_  = final_goal_;
    local_target_vel_ = start_vel_;
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

    start_pt_  = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init = false;
    // // 第一次调用执行状态
    // if (timesOfConsecutiveStateCalls().first == 1)
    //     flag_random_poly_init = false;
    // else
    //     flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++) {
        if (callReboundReplan(true, flag_random_poly_init)) {
            return true;
        }
    }
    return false;
}

Mat image = cv::Mat::ones(1280, 720, CV_8UC3);


Point start_pt(-1, -1);
Point end_pt(-1, -1);
bool is_optimized = false;
void initMap(cv::Mat &img)
{
    if (use_img_flag == true) {
        image = cv::imread("../misc/map.png");
    } else {
        int width  = img.cols;
        int height = img.rows;
        img.setTo(cv::Scalar(255, 255, 255));

        cv::rectangle(img, cv::Rect(100 + width / 2.0, 100 + height / 4.0, 200, 200), cv::Scalar(255, 0, 0), -1);

        cv::rectangle(img, cv::Rect(100 + height / 3.0, 100, 100, 200), cv::Scalar(255, 0, 0), -1);
        cv::circle(img, Point(200, 350), 100, cv::Scalar(0, 0, 255), -1);

        cv::circle(img, Point(600, 200), 100, cv::Scalar(0, 0, 255), -1);

        cv::circle(img, Point(480, 500), 100, cv::Scalar(0, 0, 255), -1);
    }

    is_optimized = false;
    return;
}
void onMouse(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN) { // 如果左键按下，选起点
        cout << "press left and set start at (" << x << ", " << y << ")" << std::endl;
        // circle(image, start_pt, 5, cv::Scalar(255, 255, 255), -1);
        initMap(image);
        start_pt = Point(x, y);
    } else if (event == EVENT_RBUTTONDOWN) { // 如果右键按下，选终点
        cout << "press right and set end  at (" << x << ", " << y << ")" << std::endl;
        // circle(image, end_pt, 5, cv::Scalar(255, 255, 255), -1);
        initMap(image);
        end_pt = Point(x, y);
    } else if (event == EVENT_MBUTTONDOWN) {
        std::cout << "Middle button down and clear start and end" << std::endl;
        start_pt = Point(-1, -1);
        end_pt   = Point(-1, -1);

        initMap(image);

        imshow("Select points", image);
    }
}

bool isPointWithinImage(const cv::Mat &image, const Point &point)
{
    return point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows;
}

int main(int argc, char *argv[])
{
    cv::Mat temp;
    initMap(temp);

    if (image.cols == 0 || image.rows == 0) {
        std::cerr << "image: cols:" << image.cols << " rows:" << image.rows << std::endl;
        return 1;
    }

    std::cerr << "image: cols:" << image.cols << " rows:" << image.rows << " channel: " << image.channels()
              << " type: " << image.type() << " " << CV_8UC1 << std::endl;

    std::cerr << "参数数量:" << argc << std::endl;
    if (argc > 1 && argc != 5) {
        std::cerr << "请输入两个点的坐标(x1, y1, x2, y2)作为命令行参数！" << std::endl;
        std::cerr << "./ego_planner_test 100 100 1050 350" << std::endl;
        return 1;
    } else if (argc > 1 && argc == 5) {
        start_pt.x = std::stoi(argv[1]);
        start_pt.y = std::stoi(argv[2]);
        end_pt.x   = std::stoi(argv[3]);
        end_pt.y   = std::stoi(argv[4]);

        std::cout << "第一个点的坐标为: (" << start_pt.x << ", " << start_pt.y << ")" << std::endl;
        std::cout << "第二个点的坐标为: (" << end_pt.x << ", " << end_pt.y << ")" << std::endl;

        if (!isPointWithinImage(image, start_pt) || !isPointWithinImage(image, end_pt)) {
            std::cerr << "点的坐标超出图像范围！" << std::endl;
            return 1;
        }
    }

    cv::Mat grey_mat;
    cv::cvtColor(image, grey_mat, cv::COLOR_BGR2GRAY);

    std::cerr << "grey_mat: cols:" << grey_mat.cols << " rows:" << grey_mat.rows << " channel: " << grey_mat.channels()
              << " type: " << grey_mat.type() << " " << CV_8UC1 << std::endl;

    TMapParam mapParam;
    mapParam.height     = grey_mat.rows;
    mapParam.width      = grey_mat.cols;
    mapParam.resolution = 0.05;
    mapParam.xMin       = 0.0;
    mapParam.yMin       = 0.0;
    TMapData planMap(mapParam, 0);
    std::cerr << "image: height:" << mapParam.height << " width:" << mapParam.width << std::endl;
    std::memcpy(&planMap.map[0], grey_mat.data, mapParam.height * mapParam.width);
    // planMap.WritePgm("../misc/ego_plan.pgm");

    namedWindow("Select points");
    setMouseCallback("Select points", onMouse, nullptr);

    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(planMap);


    while (true)
    {
        imshow("Select points", image);

        if (start_pt.x != -1 && end_pt.x != -1 && is_optimized == false)
        { // 如果都选择好了起始点和终点
            printf("optimize start: \n");
            printf("start: %d %d %f %f \n", start_pt.x, start_pt.y, mapParam.idx2x(start_pt.x), mapParam.idx2y(start_pt.y));
            printf("end  : %d %d %f %f \n", end_pt.x, end_pt.y, mapParam.idx2x(end_pt.x), mapParam.idx2y(end_pt.y));

            odom_pos_ = Eigen::Vector2d(mapParam.idx2x(start_pt.x), mapParam.idx2y(start_pt.y));
            odom_vel_ = Eigen::Vector2d(0.0, 0.0);
            start_acc_ = Eigen::Vector2d(0.0, 0.0);

            final_goal_ = Eigen::Vector2d(mapParam.idx2x(end_pt.x), mapParam.idx2y(end_pt.y));
            ego_planner::Time start = ego_planner::Now();
            bool result = planFromGlobalTraj(1);
            // if (planFromGlobalTraj(1))
            // {
            auto data = &planner_manager_->traj_.local_traj;
            Eigen::VectorXd durs = data->traj.getDurations();
            double total_duration = data->traj.getTotalDuration();
            // cout << total_duration << " total_duration" << endl;
            // cout << data->duration << " total_duration" << endl;
            // Drawing 2: path points
            for (double t = 0; t < total_duration; t = t + 0.1)
            {
                Eigen::Vector2d pos = data->traj.getPos(t);
                // cout << t << " " << pos.transpose() << endl;
                circle(image, Point(mapParam.x2idx(pos[0]), mapParam.x2idx(pos[1])), 3, cv::Scalar(0, 0, 255), -1);
            }

            int piece_num = data->traj.getPieceNum();
            cout << "piece_num " << piece_num << endl;
            Eigen::MatrixXd pos = data->traj.getPositions();
            // cout<<pos<<endl;
            // Drawing 2: segment points
            for (int i = 0; i < piece_num - 1; i++)
            {
                // cout<<pos.col(i + 1)<<endl;
                circle(image, Point(mapParam.x2idx(pos(0, i + 1)), mapParam.y2idx(pos(1, i + 1))), 8, cv::Scalar(20, 255, 255), 2);
            }
            // Drawing 3: check points
            for (size_t i = 0; i < data->pts_chk.size(); i++)
            {
                for (size_t j = 0; j < data->pts_chk[i].size(); ++j)
                {
                    Eigen::Vector2d pos = data->pts_chk[i][j].second;
                    circle(image, Point(mapParam.x2idx(pos[0]), mapParam.x2idx(pos[1])), 2, cv::Scalar(0, 0, 0), -1);
                }
            }
            // }else{
            //     cout<<"plan failed"<<endl;
            // }
            if (result == true)
            {
                cv::putText(image, "Success", cv::Point(20, 40), cv::FONT_HERSHEY_TRIPLEX, 1.8, cv::Scalar(255, 200, 200), 2);
            }
            else
            {
                cv::putText(image, "Failed ", cv::Point(20, 40), cv::FONT_HERSHEY_TRIPLEX, 1.8, cv::Scalar(255, 200, 200), 2);
            }

            ego_planner::Time end = ego_planner::Now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            std::cout << "程序耗时: " << duration.count() << " 毫秒" << std::endl;


            // line(image, start_pt, end_pt, Scalar(0, 0, 255), 2);
            circle(image, start_pt, 5, cv::Scalar(0, 255, 0), -1);
            circle(image, end_pt, 5, cv::Scalar(255, 255, 0), -1);
            // 生成文件名
            // std::string currentTime = getCurrentTimeAsString();
            // std::string filename = "/tmp/image_" + currentTime + ".jpg";

            // cv::imwrite(filename, image);  // 将图像保存为JPEG

            // std::cout << "图像已保存为 " << filename << std::endl;
            imshow("Select points", image);
            is_optimized = true;
            // waitKey(0);
            // break;
        }
        else if (start_pt.x != -1)
        {
            circle(image, start_pt, 5, cv::Scalar(0, 255, 0), -1);
        }
        else if (end_pt.x != -1)
        {
            circle(image, end_pt, 5, cv::Scalar(0, 255, 0), -1);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // cout << "wait for target" << endl;
        }
        if (waitKey(10) == 27) { // ESC键退出程序
            break;
        }
    }

    return 0;
}
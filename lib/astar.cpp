/**
 * @file astar.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#include "astar.h"

#include <assert.h>
#include <math.h>

#include <algorithm>
#include <limits>

#include "common_alg.h"

#define ASTAR_PATH_DOWN_SAMPLE_DIS 3.5

#define SHOW_EXTENDED_GRID_MAP 0
#define SHOW_EXTENDED_MASK     0
#define USE_MY_EXTEND_METHOD   0
#define SHOW_REPATHPLAN_MAP    0

#define MULTI_PLAN_WITH_GRAY_AREA 1

// #define CUR_POSE_FILL_WHITE_RADIUS_SIZE (MIN_PREREACH_TARGET_DIS / DEFAULT_RESOLUTION - 0.5)
#define CUR_POSE_FILL_WHITE_RADIUS_SIZE 3

#define ASTAR_COST_TABLE_SIZE 4
#define ASTAR_DILATE_SIZE     2
#define ASTAR_CAN_LINE_COST   (ASTAR_COST_TABLE_SIZE)

#define VIRTUAL_OBST_REF 120
#define HIGH_WEIGHT_REF  202
#define LOW_WEIGHT_REF   203

#define IS_KEY_POINT_PATH 1

namespace LDCV {

const int8_t sDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int8_t sDy[8] = {0, 0, -1, 1, 1, -1, 1, -1};

#define CONNECT_SIZE 4   // only 4 or 8

const int sCost[8] = {1, 1, 1, 1, 2, 2, 2, 2};   // 四联通代价1 八联通代价2

const int AStarCostTable[ASTAR_COST_TABLE_SIZE + 1]  = {0, 8, 4, 2, 1};
const int AStarCostTableS[ASTAR_COST_TABLE_SIZE + 1] = {0, 2, 1, 1, 1};
#define ASTAR_COST_EXTEND_MASK \
    { -1, 1, -width_, width_ /*,-1-width_,1-width_,-1+width_,1+width_*/ }

#define GRAY_COST 12800

CAStar::CAStar(int width, int height) {
    width_  = width;
    height_ = height;

    map_bytes_ = new uint8_t[width_ * height_];
    close_     = new CCloseMap(width_, height_);

    heap_str_ = new CHeap(width_, height_);
    heap_end_ = new CHeap(width_, height_);
}

CAStar::~CAStar() {
    if (map_bytes_) {
        delete[] map_bytes_;
        map_bytes_ = NULL;
    }

    if (close_) {
        delete close_;
        close_ = NULL;
    }

    if (heap_str_) {
        delete heap_str_;
        heap_str_ = NULL;
    }

    if (heap_end_) {
        delete heap_end_;
        heap_end_ = NULL;
    }
}

int CAStar::CalHStar(Point start, Point end) { return abs(end.x - start.x) + abs(end.y - start.y); }

float CAStar::GetDistance(Point start, Point end) {
    return sqrt(static_cast<float>(end.x - start.x) * (end.x - start.x) +
                static_cast<float>(end.y - start.y) * (end.y - start.y));
}

bool CAStar::CanPass(int x, int y, uint8_t area_gray_ref) {
    if (map_bytes_[y * width_ + x] <= area_gray_ref) {
        return false;
    }
    return true;
}

bool CAStar::IsLineCrossWithMap(Point p1, Point p2, uint8_t area_gray_ref) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    if (abs(dx) > abs(dy)) {
        Point left, right, cur_point;
        if (p1.x < p2.x) {
            left  = p1;
            right = p2;
        } else {
            left  = p2;
            right = p1;
        }
        if (dx > 0) {
            double deta_y   = (left.y - right.y) * 1.0 / abs(dx);
            double double_y = right.y;
            for (int i = 0; i < dx; i++) {
                // double double_y = right.y + (left.y - right.y) * i* 1.00 / abs(dx);
                int int_y = double_y >= 0 ? static_cast<int>(double_y + 0.5) : static_cast<int>(double_y - 0.5);
                double_y += deta_y;
                cur_point = Point(right.x - i, int_y);
                if (!CanPass(cur_point.x, cur_point.y, area_gray_ref)) {
                    obstacle_point_ = Point(cur_point.x, cur_point.y);
                    return false;
                }
            }
        } else {
            double deta_y   = (right.y - left.y) * 1.00 / abs(dx);
            double double_y = left.y;
            for (int i = 0; i < abs(dx); i++) {
                // double double_y = left.y + (right.y - left.y) * i* 1.00 / abs(dx);
                int int_y = double_y >= 0 ? static_cast<int>(double_y + 0.5) : static_cast<int>(double_y - 0.5);
                double_y += deta_y;
                cur_point = Point(left.x + i, int_y);
                if (!CanPass(cur_point.x, cur_point.y, area_gray_ref)) {
                    obstacle_point_ = Point(cur_point.x, cur_point.y);
                    return false;
                }
            }
        }
    } else {
        Point top, down, cur_point;
        if (p1.y > p2.y) {
            top  = p1;
            down = p2;
        } else {
            top  = p2;
            down = p1;
        }
        if (dy > 0) {
            double deta_x   = (down.x - top.x) * 1.00 / abs(dy);
            double double_x = top.x;
            for (int i = 0; i < dy; i++) {
                // double double_x = top.x + (down.x - top.x) * i* 1.00 / abs(dy);
                int int_x = double_x >= 0 ? static_cast<int>(double_x + 0.5) : static_cast<int>(double_x - 0.5);
                double_x += deta_x;
                cur_point = Point(int_x, top.y - i);
                if (!CanPass(cur_point.x, cur_point.y, area_gray_ref)) {
                    obstacle_point_ = Point(cur_point.x, cur_point.y);
                    return false;
                }
            }
        } else {
            double deta_x   = (top.x - down.x) * 1.00 / abs(dy);
            double double_x = down.x;
            for (int i = 0; i < abs(dy); i++) {
                int int_x = double_x >= 0 ? static_cast<int>(double_x + 0.5) : static_cast<int>(double_x - 0.5);
                double_x += deta_x;
                cur_point = Point(int_x, static_cast<int>(down.y + i));

                if (!CanPass(cur_point.x, cur_point.y, area_gray_ref)) {
                    obstacle_point_ = Point(cur_point.x, cur_point.y);
                    return false;
                }
            }
        }
    }
    return true;
}

bool CAStar::fillWhiteInRTMap(uint8_t *RT_map_bytes, Point cur_position, int fill_radius) {
    // 两米的范围内  正方形 边界判断
    int dx = fill_radius, dy = fill_radius;
    Point left_down = Point(cur_position.x - dx, cur_position.y - dy);
    Point right_up  = Point(cur_position.x + dx, cur_position.y + dy);
    assert(left_down.x < width_);

    if (left_down.x < 0 || left_down.x > width_ - 1) {
        left_down.x = (left_down.x < 0) ? 0 : width_ - 1;
    }
    if (left_down.y < 0 || left_down.y > height_ - 1) {
        left_down.y = (left_down.y < 0) ? 0 : height_ - 1;
    }

    if (right_up.x < 0 || right_up.x > width_ - 1) {
        right_up.x = (right_up.x < 0) ? 0 : width_ - 1;
    }
    if (right_up.y < 0 || right_up.y > height_ - 1) {
        right_up.y = (right_up.y < 0) ? 0 : height_ - 1;
    }

    // leftUp.x = (leftUp.x < 0) ? 一定要判断越界问题
    for (int j = left_down.y; j <= right_up.y; j++) {
        for (int i = left_down.x; i <= right_up.x; i++) {
            RT_map_bytes[j * width_ + i] = 255;
        }
    }

    return true;
}

bool CAStar::fillWhiteInRTMap(uint8_t *RT_map_bytes, Point start) {
    int iter                                 = 1;
    bool stop_flag                           = false;
    RT_map_bytes[start.y * width_ + start.x] = 255;
    while (!stop_flag) {
        for (int y = start.y - iter; y <= start.y + iter; y++) {
            for (int x = start.x - iter; x <= start.x + iter; x++) {
                if (x < 0 || x > width_ - 1 || y < 0 || y > height_ - 1) {
                    continue;
                }

                if (x == start.x - iter || x == start.x + iter || y == start.y - iter || y == start.y + iter) {
                    if (RT_map_bytes[y * width_ + x] < 255) {
                        RT_map_bytes[y * width_ + x] = 255;
                    } else {
                        stop_flag = true;
                        break;
                    }
                }
            }
            if (stop_flag) {
                break;
            }
        }
        iter++;
    }

    return true;
}

CAStar::AstarResult CAStar::FindPathWithRadius(std::vector<Point> *track_points, float &max_radius_from_start,
                                               Point &start, Point &end, Mat &astar_trap_area) {
    std::vector<Point> vtrack;
    CAStar::AstarResult result = FindPathNearest(start, end, vtrack, max_radius_from_start, astar_trap_area);
    *track_points              = vtrack;

    return result;
}

CAStar::AstarResult CAStar::FindPathWithRadiusFast(std::vector<Point> *track_points, float &max_radius_from_start,
                                                   Point &start, Point &end, Mat &astar_trap_area) {
    std::vector<Point> vtrack;
    CAStar::AstarResult result = FindPath(start, end, vtrack, max_radius_from_start, astar_trap_area);

    *track_points = vtrack;
    return result;
}

bool CAStar::FindPathWithMultiTarget(Point rect_min_with_border, Point rect_max_with_border, Point &start,
                                     std::vector<Point> &track_points, std::vector<Point> &target_list,
                                     int *nearest_id) {
    rect_min_with_border.x = std::max(rect_min_with_border.x, 0);
    rect_min_with_border.y = std::max(rect_min_with_border.y, 0);

    rect_max_with_border.x = std::min(rect_max_with_border.x, width_);
    rect_max_with_border.y = std::min(rect_max_with_border.y, height_);

    if (!IsPointIn(start, rect_min_with_border, rect_max_with_border)) {
        return false;
    }

    if (target_list.size() == 0) {
        return false;
    }
    for (size_t i = 0; i < target_list.size(); i++) {
        if ((target_list[i].x < rect_min_with_border.x) || (target_list[i].y < rect_min_with_border.y) ||
            (target_list[i].x > rect_max_with_border.x) || (target_list[i].y > rect_max_with_border.y)) {
            return false;
        }
    }

    *nearest_id = -1;

    close_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y - rect_min_with_border.y);
    heap_str_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y - rect_min_with_border.y);

    CAStarPoint *start_astar_point = new CAStarPoint();
    start_astar_point->Pos         = start;
    // start_astar_point->G = 0;
    // start_astar_point->H = 0;
    start_astar_point->F = 0;

    Point tmp(start.x - rect_min_with_border.x, start.y - rect_min_with_border.y);
    close_->AddToClose(9, 1, &tmp);
    heap_str_->AddPoint(start_astar_point);

    for (size_t i = 0; i < target_list.size(); i++) {
        Point temp(target_list[i].x - rect_min_with_border.x, target_list[i].y - rect_min_with_border.y);
        close_->AddToClose(9, 2, &temp);
    }

    bool reach_target = false;
    Point pos_M1, pos_M2;

    while ((heap_str_->GetCount() > 0) && (!reach_target)) {
        CAStarPoint *cur = heap_str_->PopPoint();

        for (int i = 0; i < CONNECT_SIZE; i++) {
            Point next_pos(cur->Pos.x + sDx[i], cur->Pos.y + sDy[i]);

            if (CanPass(next_pos.x, next_pos.y, 0)) {
                int ret;
                Point temp(next_pos.x - rect_min_with_border.x, next_pos.y - rect_min_with_border.y);
                // int8_t m_index = close_->GetIndex(cur->Pos);
                int8_t m_index = 1;
                if (!(ret = close_->AddToClose(i, m_index, &temp))) {
                    CAStarPoint *tmp_point = new CAStarPoint();

                    // 四连通 && 八连通代价
                    tmp_point->G = cur->G + sCost[i];

                    int index = map_bytes_[next_pos.y * width_ + next_pos.x];
                    if (index <= ASTAR_COST_TABLE_SIZE) {
                        // 贴近障碍物代价
                        tmp_point->G += AStarCostTable[index];
                    } else if (index == 127) {
                        // 灰色区域代价
                        tmp_point->G += GRAY_COST;
                    }

                    // 多目标A*的启发代价恒定为0
                    tmp_point->H = 0;

                    tmp_point->F   = tmp_point->G + tmp_point->H;
                    tmp_point->Pos = next_pos;
                    // tmp_point->Previous = cur;
                    heap_str_->AddPoint(tmp_point);
                } else {
                    if ((ret == 1) && (!reach_target)) {
                        reach_target = true;
                        pos_M1       = cur->Pos;
                        pos_M2       = next_pos;
                    } else {
                        continue;
                    }
                }
            }
        }
        delete cur;
    }

    heap_str_->FreeList();

#ifdef OPENCV
    // 调试图层 遍历完成后的图层 可以查看搜索失败的原因
    cv::Mat d = cv::Mat(height_, width_, CV_8U, map_bytes_);
    cv::Mat e = close_->GetOpencvMat();
#endif

    if (reach_target) {
        std::vector<Point> tmp_points;
        std::vector<Point> start_points;
        for (size_t i = 0; i < target_list.size(); i++) {
            if ((pos_M2.x == target_list[i].x) && (pos_M2.y == target_list[i].y)) {
                *nearest_id = i;
            }
        }

        start_points.push_back(pos_M2);
        {
            Point m_now = pos_M1;
            while ((m_now.x != start.x) || (m_now.y != start.y)) {
                start_points.push_back(m_now);
                Point temp(m_now.x - rect_min_with_border.x, m_now.y - rect_min_with_border.y);
                int8_t m_dxdy = close_->GetDxDy(temp);
                m_now.x -= sDx[m_dxdy];
                m_now.y -= sDy[m_dxdy];
            }
            start_points.push_back(m_now);
        }

        while (start_points.size() > 0) {
            tmp_points.push_back(start_points.back());
            start_points.pop_back();
        }

        // clean track_points;
        std::vector<Point>().swap(track_points);
        if (IS_KEY_POINT_PATH) {
            track_points.push_back(tmp_points[0]);
            for (size_t i = 1; i < tmp_points.size() - 1; i++) {
                if (!IsLineCrossWithMap(track_points.back(), tmp_points[i], ASTAR_CAN_LINE_COST)) {
                    track_points.push_back(tmp_points[i]);
                }
            }
            track_points.push_back(tmp_points[tmp_points.size() - 1]);
        } else {
            track_points = tmp_points;
        }

        return true;
    }
    return false;
}

/**
 * @brief 多目标A*搜索 时间优化 1/2 内存资源增多
 * 多目标A*搜索本质上就是一个以起点开始的广度优先搜索，只不过添加了远近顺序权值而已
 * 这里有问题就是无法对靠近障碍物的情况进行优先排序，只是曼哈顿距离的排序
 *
 * @param rect_min_with_border
 * @param rect_max_with_border
 * @param start
 * @param track_points
 * @param target_list
 * @param nearest_id
 * @return true
 * @return false
 */
bool CAStar::FindPathWithMultiTargetFast(Point rect_min_with_border, Point rect_max_with_border, Point &start,
                                         std::vector<Point> &track_points, std::vector<Point> &target_list,
                                         int *nearest_id) {
    rect_min_with_border.x = std::max(rect_min_with_border.x, 0);
    rect_min_with_border.y = std::max(rect_min_with_border.y, 0);
    rect_max_with_border.x = std::min(rect_max_with_border.x, width_);
    rect_max_with_border.y = std::min(rect_max_with_border.y, height_);

    if (!IsPointIn(start, rect_min_with_border, rect_max_with_border)) {
        return false;
    }

    if (target_list.size() == 0) {
        return false;
    }

    for (size_t i = 0; i < target_list.size(); i++) {
        if ((target_list[i].x < rect_min_with_border.x) || (target_list[i].y < rect_min_with_border.y) ||
            (target_list[i].x > rect_max_with_border.x) || (target_list[i].y > rect_max_with_border.y)) {
            return false;
        }
    }

    *nearest_id = -1;

    close_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y - rect_min_with_border.y);
    // heap_str_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y -
    // rect_min_with_border.y);

    Point tmp(start.x - rect_min_with_border.x, start.y - rect_min_with_border.y);
    close_->AddToClose(9, 1, &tmp);
    // heap_str_->AddPoint(start_astar_point);

    // 队列初始化
    int h   = 0;   // 队头索引
    int r   = 0;   // 队尾索引
    int sum = 0;

    // 预分配内存的队列比deque队列要快
    // 如果这个队列能随A*创建不用每次开辟与释放还会更快
    int size_e = (rect_max_with_border.x - rect_min_with_border.x + 1) *
                 (rect_max_with_border.y - rect_min_with_border.y + 1);
    int16_t *xlist = new int16_t[size_e];
    int16_t *ylist = new int16_t[size_e];
    xlist[h]       = start.x;
    ylist[h]       = start.y;

    // deque<Point> list;
    // list.push_back(start_astar_point->Pos);

    sum++;

    // 将目标点放入图层
    for (size_t i = 0; i < target_list.size(); i++) {
        Point temp(target_list[i].x - rect_min_with_border.x, target_list[i].y - rect_min_with_border.y);
        close_->AddToClose(9, 2, &temp);
    }

    bool reach_target = false;
    Point pos_M1, pos_M2;
    Point cur_pose;
    Point next_pos;
    Point temp;
    int ret;
    while ((sum) && (!reach_target)) {
        // 从队尾取值，队尾索引++
        cur_pose.x = xlist[h];
        cur_pose.y = ylist[h];
        h         = (h + 1) % size_e;

        // cur_pose = list.front();
        // list.pop_front();

        sum--;   // 剩余数量--

        for (int i = 0; i < CONNECT_SIZE; i++) {
            next_pos.x      = cur_pose.x + sDx[i];
            next_pos.y      = cur_pose.y + sDy[i];
            int8_t m_index = 1;

            //// 这里只判定>0 !!!!!
            //// 有问题 !!!! 规划会贴近障碍物
            // if (map_bytes_[next_pos.y * width_ + next_pos.x] > 0)
            if (map_bytes_[next_pos.y * width_ + next_pos.x] == 255) {
                temp.x = next_pos.x - rect_min_with_border.x;
                temp.y = next_pos.y - rect_min_with_border.y;
                if (!(ret = close_->AddToClose(i, m_index, &temp))) {
                    // 放入队头，队头索引++
                    r        = (r + 1) % size_e;
                    xlist[r] = next_pos.x;
                    ylist[r] = next_pos.y;

                    // list.push_back(next_pos);

                    sum++;   // 剩余数量++

                } else {
                    if ((ret == 1) && (!reach_target)) {
                        reach_target = true;
                        pos_M1       = cur_pose;
                        pos_M2       = next_pos;
                    } else {
                        continue;
                    }
                }
            }
        }
    }

    delete[] xlist;
    delete[] ylist;

    if (reach_target) {
        std::vector<Point> tmp_points;
        std::vector<Point> start_points;
        for (size_t i = 0; i < target_list.size(); i++) {
            if ((pos_M2.x == target_list[i].x) && (pos_M2.y == target_list[i].y)) {
                *nearest_id = i;
            }
        }

        start_points.push_back(pos_M2);
        {
            Point m_now = pos_M1;
            while ((m_now.x != start.x) || (m_now.y != start.y)) {
                start_points.push_back(m_now);
                Point temp(m_now.x - rect_min_with_border.x, m_now.y - rect_min_with_border.y);
                int8_t m_dxdy = close_->GetDxDy(temp);
                m_now.x -= sDx[m_dxdy];
                m_now.y -= sDy[m_dxdy];
            }
            start_points.push_back(m_now);
        }

        while (start_points.size() > 0) {
            tmp_points.push_back(start_points.back());
            start_points.pop_back();
        }

        track_points.clear();
        if (IS_KEY_POINT_PATH) {
            track_points.push_back(tmp_points[0]);
            for (size_t i = 1; i < tmp_points.size() - 1; i++) {
                if (!IsLineCrossWithMap(track_points.back(), tmp_points[i], ASTAR_CAN_LINE_COST)) {
                    track_points.push_back(tmp_points[i]);
                }
            }
            track_points.push_back(tmp_points[tmp_points.size() - 1]);
        } else {
            track_points = tmp_points;
        }

        return true;
    }

    return false;
}

bool CAStar::FindNearestPointToTargetMap(Point rect_min_with_border, Point rect_max_with_border, Point start,
                                         const Mat *target_map, Point &output_point, Mat &start_search_area,
                                         Mat &astar_trap_area, float *max_radius_from_start,
                                         std::vector<Point> *track_points) {
    // check width height
    if (target_map == NULL || target_map->cols != width_ || target_map->rows != height_ ||
        start_search_area.cols != width_ || start_search_area.rows != height_ || astar_trap_area.cols != width_ ||
        astar_trap_area.rows != height_) {
        assert(false);
        return false;
    }

    rect_min_with_border.x = std::max(rect_min_with_border.x, 0);
    rect_min_with_border.y = std::max(rect_min_with_border.y, 0);
    rect_max_with_border.x = std::min(rect_max_with_border.x, width_);
    rect_max_with_border.y = std::min(rect_max_with_border.y, height_);

    uint8_t *start_search_area_data = &start_search_area.data[0];
    uint8_t *astar_trap_area_data   = &astar_trap_area.data[0];

    memset(start_search_area_data, 0, width_ * height_);
    memcpy(astar_trap_area_data, map_bytes_, width_ * height_);

    CCommonAlg::circleMat(255, start.x, start.y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, start_search_area_data, width_,
                          height_, 0, width_, 0, height_);
    CCommonAlg::circleMat(0, start.x, start.y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, astar_trap_area_data, width_, height_,
                          0, width_, 0, height_);

    if (!IsPointIn(start, rect_min_with_border, rect_max_with_border)) {
        return false;
    }

    close_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y - rect_min_with_border.y);
    heap_str_->Init(rect_max_with_border.x - rect_min_with_border.x, rect_max_with_border.y - rect_min_with_border.y);

    CAStarPoint *start_astar_point = new CAStarPoint();
    start_astar_point->Pos         = start;
    start_astar_point->F           = 0;

    Point tmp(start.x - rect_min_with_border.x, start.y - rect_min_with_border.y);

    close_->AddToClose(9, 1, &tmp);
    heap_str_->AddPoint(start_astar_point);

    for (int i = rect_min_with_border.y; i <= rect_max_with_border.y; i++) {
        int i_w = i * target_map->cols;
        for (int j = rect_min_with_border.x; j <= rect_max_with_border.x; j++) {
            int index = j + i_w;
            if (target_map->data[index] == 255) {
                Point temp(j - rect_min_with_border.x, i - rect_min_with_border.y);
                close_->AddToClose(9, 2, &temp);
            }
        }
    }

    int maxR = (std::numeric_limits<int>::min)();
    CAStarPoint maxR_cur, cur_str;

    bool reach_target = false;
    Point pos_M1, pos_M2;

    while ((heap_str_->GetCount() > 0) && (!reach_target)) {
        {
            CAStarPoint *cur = heap_str_->PopPoint();

            for (int i = 0; i < CONNECT_SIZE; i++) {
                Point next_pos(cur->Pos.x + sDx[i], cur->Pos.y + sDy[i]);
                int8_t m_index = 1;

                if (CanPass(next_pos.x, next_pos.y, 0)) {
                    int ret;
                    Point temp(next_pos.x - rect_min_with_border.x, next_pos.y - rect_min_with_border.y);
                    if (!(ret = close_->AddToClose(i, m_index, &temp))) {
                        CAStarPoint *tmp_point = new CAStarPoint();

                        tmp_point->G = cur->G + sCost[i];

                        tmp_point->R = cur->R + sCost[i];

                        int index = map_bytes_[next_pos.y * width_ + next_pos.x];

                        astar_trap_area_data[next_pos.y * width_ + next_pos.x]   = 0;
                        start_search_area_data[next_pos.y * width_ + next_pos.x] = 255;

                        if (index <= ASTAR_COST_TABLE_SIZE) {
                            tmp_point->G += AStarCostTable[index];
                        } else if (index == 127) {
                            tmp_point->G += GRAY_COST;
                        } else if (index == HIGH_WEIGHT_REF) {
                            tmp_point->G += 5;
                        } else if (index == LOW_WEIGHT_REF) {
                            tmp_point->G += 3;
                        }
                        tmp_point->H   = 0;
                        tmp_point->F   = tmp_point->G + tmp_point->H;
                        tmp_point->Pos = next_pos;
                        heap_str_->AddPoint(tmp_point);

                        cur_str = *tmp_point;
                        if (cur_str.R > maxR) {
                            maxR     = cur_str.R;
                            maxR_cur = cur_str;
                        }
                    } else {
                        if ((ret == 1) && (!reach_target)) {
                            reach_target = true;
                            pos_M1       = cur->Pos;
                            pos_M2       = next_pos;
                        } else {
                            continue;
                        }
                    }
                }
            }
            delete cur;
        }
    }

    heap_str_->FreeList();

    if (reach_target) {
        if (track_points != nullptr) {
            std::vector<Point> tmp_points;
            std::vector<Point> start_points;

            start_points.push_back(pos_M2);
            {
                Point m_now = pos_M1;
                while ((m_now.x != start.x) || (m_now.y != start.y)) {
                    start_points.push_back(m_now);
                    Point temp(m_now.x - rect_min_with_border.x, m_now.y - rect_min_with_border.y);
                    int8_t m_dxdy = close_->GetDxDy(temp);
                    m_now.x -= sDx[m_dxdy];
                    m_now.y -= sDy[m_dxdy];
                }
                start_points.push_back(m_now);
            }

            while (start_points.size() > 0) {
                tmp_points.push_back(start_points.back());
                start_points.pop_back();
            }

            track_points->clear();
            if (IS_KEY_POINT_PATH) {
                track_points->push_back(tmp_points[0]);
                for (size_t i = 1; i < tmp_points.size() - 1; i++) {
                    if (!IsLineCrossWithMap(track_points->back(), tmp_points[i], ASTAR_CAN_LINE_COST)) {
                        track_points->push_back(tmp_points[i]);
                    }
                }
                track_points->push_back(tmp_points[tmp_points.size() - 1]);
            } else {
                for (size_t i = 0; i < tmp_points.size(); i++) {
                    track_points->push_back(tmp_points[i]);
                }
            }
        }
        output_point = pos_M2;
        return true;
    } else {
        if (max_radius_from_start != nullptr) {
            *max_radius_from_start = sqrt((maxR_cur.Pos.x - start.x) * (maxR_cur.Pos.x - start.x) +
                                          (maxR_cur.Pos.y - start.y) * (maxR_cur.Pos.y - start.y));
        }
    }

    return false;
}

bool CAStar::IsPointIn(Point &point, Point &point_min, Point &point_max) {
    if ((point.x <= point_min.x) || (point.y <= point_min.y) || (point.x >= point_max.x) || (point.y >= point_max.y)) {
        return false;
    }
    return true;
}

CAStar::AstarResult CAStar::FindPathNearest(Point &start, Point &end, std::vector<Point> &track_points,
                                            float &max_radius_from_start, Mat &astar_trap_area) {
    uint8_t *astar_trap_area_data = &astar_trap_area.data[0];
    memcpy(astar_trap_area_data, map_bytes_, width_ * height_);

    CCommonAlg::circleMat(0, start.x, start.y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, astar_trap_area_data, width_, height_,
                          0, width_, 0, height_);

    max_radius_from_start = -1;
    Point zero(0, 0);
    Point max_point(width_ - 1, height_ - 1);

    if (!IsPointIn(start, zero, max_point)) {
        for (int i = 0; i < 5; i++) {
        }
        return ASR_START_TRAP;
    }
    if (!IsPointIn(end, zero, max_point)) {
        for (int i = 0; i < 5; i++) {
        }
        return ASR_END_TRAP;
    }
    // if (!CanPass(start.x, start.y, 0)) {
    // //return ASR_START_TRAP;
    // map_bytes_[start.x + start.y*width_] = 255;
    //}

    close_->Init(width_, height_);

    heap_str_->Init(width_, height_);
    CAStarPoint *start_astar_point = new CAStarPoint();
    start_astar_point->Pos         = start;
    start_astar_point->G           = 0;
    start_astar_point->R           = 0;
    start_astar_point->H           = CalHStar(start, end);
    start_astar_point->F           = start_astar_point->G + start_astar_point->H;
    close_->AddToClose(9, 1, &start);
    heap_str_->AddPoint(start_astar_point);

    heap_end_->Init(width_, height_);
    CAStarPoint *end_astar_point = new CAStarPoint();
    end_astar_point->Pos         = end;
    end_astar_point->G           = 0;
    end_astar_point->H           = CalHStar(end, start);
    end_astar_point->F           = end_astar_point->G + end_astar_point->H;
    close_->AddToClose(9, 2, &end);
    heap_end_->AddPoint(end_astar_point);

    bool reach_target  = false;
    AstarResult result = ASR_SUCCESS;
    Point pos_M1, pos_M2;

    int maxR = (std::numeric_limits<int>::min)();
    int minH = (std::numeric_limits<int>::max)();
    CAStarPoint minH_cur, maxR_cur, cur;

    int search_count = 0;

    while ((heap_str_->GetCount() > 0) && (!reach_target)) {
        CAStarPoint *cur_str = heap_str_->PopPoint();
        search_count++;
        for (int i = 0; i < CONNECT_SIZE; i++) {
            Point next_pos(cur_str->Pos.x + sDx[i], cur_str->Pos.y + sDy[i]);
            // int8_t m_index = close_->GetIndex(cur_str->Pos);
            int8_t m_index = 1;

            if (CanPass(next_pos.x, next_pos.y, 0)) {
                int ret;
                if (!(ret = close_->AddToClose(i, m_index, &next_pos))) {
                    CAStarPoint *tmp_point = new CAStarPoint();
                    tmp_point->G           = cur_str->G + sCost[i];

                    tmp_point->R = cur_str->R + sCost[i];

                    astar_trap_area_data[next_pos.y * width_ + next_pos.x] = 0;

                    int index = map_bytes_[next_pos.y * width_ + next_pos.x];

                    if (index <= ASTAR_COST_TABLE_SIZE) {
                        tmp_point->G += AStarCostTable[index];
                    } else if (index == 127) {
                        tmp_point->G += GRAY_COST;
                    }

                    tmp_point->H   = CalHStar(next_pos, end);
                    tmp_point->F   = tmp_point->G + tmp_point->H;
                    tmp_point->Pos = next_pos;
                    // tmp_point->Previous = cur_str;
                    heap_str_->AddPoint(tmp_point);

                    cur = *tmp_point;
                    if (cur.R > maxR) {
                        maxR     = cur.R;
                        maxR_cur = cur;
                    }
                    if (cur.H < minH) {
                        minH     = cur.H;
                        minH_cur = cur;
                    }

                    // lstAStarPoints.push_back(*tmp_point);
                } else {
                    if ((ret == 1) && (!reach_target)) {
                        reach_target = true;
                        result       = ASR_SUCCESS;
                        pos_M1       = cur_str->Pos;
                        pos_M2       = next_pos;
                    } else {
                        continue;
                    }
                }
            }
        }
        if (heap_end_->GetCount() > 0) {
            CAStarPoint *cur = heap_end_->PopPoint();
            for (int i = 0; i < CONNECT_SIZE; i++) {
                Point next_pos(cur->Pos.x + sDx[i], cur->Pos.y + sDy[i]);
                // int8_t m_index = close_->GetIndex(cur_str->Pos);
                int8_t m_index = 2;

                if (CanPass(next_pos.x, next_pos.y, 0)) {
                    int ret;
                    if (!(ret = close_->AddToClose(i, m_index, &next_pos))) {
                        CAStarPoint *tmp_point = new CAStarPoint();

                        tmp_point->G = cur->G + sCost[i];

                        int index = map_bytes_[next_pos.y * width_ + next_pos.x];
                        if (index <= ASTAR_COST_TABLE_SIZE) {
                            tmp_point->G += AStarCostTable[index];
                        } else if (index == 127) {
                            tmp_point->G += GRAY_COST;
                        }
                        tmp_point->H   = CalHStar(next_pos, start);
                        tmp_point->F   = tmp_point->G + tmp_point->H;
                        tmp_point->Pos = next_pos;
                        // tmp_point->Previous = cur_str;
                        heap_end_->AddPoint(tmp_point);
                    } else {
                        if ((ret == 1) && (!reach_target)) {
                            reach_target = true;
                            pos_M2       = cur->Pos;
                            pos_M1       = next_pos;
                        } else {
                            continue;
                        }
                    }
                }
            }
            delete cur;
        } else {
            // int x1 = end.x - start.x;
            // int y1 = end.y - start.y;
            // int x2 = cur_str->Pos.x - end.x;
            // int y2 = cur_str->Pos.y - end.y;
            // if (fabs(x1*y2 - x2*y1)<100 && (x1*x2>0) && (y1*y2>0))
            if ((search_count > 100000) && (!reach_target)) {
                pos_M1 = cur_str->Pos;
                delete cur_str;
                result = ASR_END_TRAP;   // 终点被困
                break;
            }
        }

        delete cur_str;
    }

    std::vector<Point> tmp_points;

    if (reach_target) {
        std::vector<Point> start_points;
        Point m_now = pos_M1;
        while ((m_now.x != start.x) || (m_now.y != start.y)) {
            start_points.push_back(m_now);
            int8_t m_dxdy = close_->GetDxDy(m_now);
            m_now.x -= sDx[m_dxdy];
            m_now.y -= sDy[m_dxdy];
        }
        start_points.push_back(m_now);
        while (start_points.size() > 0) {
            tmp_points.push_back(start_points.back());
            start_points.pop_back();
        }
        std::vector<Point> end_points;
        m_now = pos_M2;
        while ((m_now.x != end.x) || (m_now.y != end.y)) {
            end_points.push_back(m_now);
            int8_t m_dxdy = close_->GetDxDy(m_now);
            m_now.x -= sDx[m_dxdy];
            m_now.y -= sDy[m_dxdy];
        }
        end_points.push_back(m_now);

        for (size_t i = 0; i < end_points.size(); i++) {
            tmp_points.push_back(end_points[i]);
        }
    } else {
        max_radius_from_start = sqrt((maxR_cur.Pos.x - start.x) * (maxR_cur.Pos.x - start.x) +
                                     (maxR_cur.Pos.y - start.y) * (maxR_cur.Pos.y - start.y));

        std::vector<Point> start_points;
        Point m_now(minH_cur.Pos.x, minH_cur.Pos.y);
        while ((m_now.x != start.x) || (m_now.y != start.y)) {
            start_points.push_back(m_now);
            int8_t m_dxdy = close_->GetDxDy(m_now);
            m_now.x -= sDx[m_dxdy];
            m_now.y -= sDy[m_dxdy];
        }
        start_points.push_back(m_now);
        while (start_points.size() > 0) {
            tmp_points.push_back(start_points.back());
            start_points.pop_back();
        }
        // 终点被困
        if (heap_end_->GetCount() <= 0) {
            result = ASR_END_TRAP;
        } else {
            result = ASR_START_TRAP;
        }
    }

    if (tmp_points.size() > 0) {
        track_points.clear();
        if (IS_KEY_POINT_PATH) {
            track_points.push_back(tmp_points[0]);
            for (size_t i = 1; i < tmp_points.size() - 1; i++) {
                if (!IsLineCrossWithMap(track_points.back(), tmp_points[i], ASTAR_CAN_LINE_COST)) {
                    track_points.push_back(tmp_points[i]);
                }
            }
            track_points.push_back(tmp_points[tmp_points.size() - 1]);
        } else {
            track_points = tmp_points;
        }
    }

    heap_str_->FreeList();
    heap_end_->FreeList();
    return result;
}

CAStar::AstarResult CAStar::FindPath(Point &start, Point &end, std::vector<Point> &track_points,
                                     float &max_radius_from_start, Mat &astar_trap_area) {
    uint8_t *astar_trap_area_data = &astar_trap_area.data[0];
    memcpy(astar_trap_area_data, map_bytes_, width_ * height_);
    // start point must be black !!! to leave trap area
    // astar_trap_area_data[start.y * width_ + start.x] = 0;
    CCommonAlg::circleMat(0, start.x, start.y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, astar_trap_area_data, width_, height_,
                          0, width_, 0, height_);

    max_radius_from_start = -1;
    Point zero(0, 0);
    Point max_point(width_ - 1, height_ - 1);

    if (!IsPointIn(start, zero, max_point)) {
        for (int i = 0; i < 5; i++) {
        }
        return ASR_START_TRAP;
    }
    if (!IsPointIn(end, zero, max_point)) {
        for (int i = 0; i < 5; i++) {
        }
        return ASR_END_TRAP;
    }
    // if (!CanPass(start.x, start.y, 0)) {
    //    return ASR_START_TRAP;
    // map_bytes_[start.x + start.y*width_] = 255;
    //}
    // if (!CanPass(end.x, end.y, 0)) {
    //  return ASR_END_TRAP;
    //}

    close_->Init(width_, height_);
    // std::vector<CAStarPoint> lstAStarPoints;

    heap_str_->Init(width_, height_);

    CAStarPoint *start_astar_point = new CAStarPoint();
    start_astar_point->Pos         = start;
    start_astar_point->G           = 0;
    start_astar_point->R           = 0;
    start_astar_point->H           = CalHStar(start, end);
    start_astar_point->F           = start_astar_point->G + start_astar_point->H;

    close_->AddToClose(9, 1, &start);
    heap_str_->AddPoint(start_astar_point);
    // lstAStarPoints.push_back(*start_astar_point);

    heap_end_->Init(width_, height_);

    CAStarPoint *end_astar_point = new CAStarPoint();
    end_astar_point->Pos         = end;
    end_astar_point->G           = 0;
    end_astar_point->H           = CalHStar(end, start);
    end_astar_point->F           = end_astar_point->G + end_astar_point->H;

    close_->AddToClose(9, 2, &end);
    heap_end_->AddPoint(end_astar_point);

    bool reach_target  = false;
    AstarResult result = ASR_SUCCESS;
    Point pos_M1, pos_M2;

    int MaxR = 0;
    Point maxR_Pos;

    while ((heap_str_->GetCount() > 0) && (heap_end_->GetCount() > 0) && (!reach_target)) {
        CAStarPoint *cur_str = heap_str_->PopPoint();
        for (int i = 0; i < CONNECT_SIZE; i++) {
            Point next_pos(cur_str->Pos.x + sDx[i], cur_str->Pos.y + sDy[i]);
            // int8_t m_index = close_->GetIndex(cur_str->Pos);
            int8_t m_index = 1;

            if (CanPass(next_pos.x, next_pos.y, 0)) {
                int ret;
                if (!(ret = close_->AddToClose(i, m_index, &next_pos))) {
                    CAStarPoint *tmp_point = new CAStarPoint();
                    tmp_point->G           = cur_str->G + sCost[i];
                    tmp_point->R           = cur_str->R + sCost[i];

                    if (tmp_point->R > MaxR) {
                        MaxR     = tmp_point->R;
                        maxR_Pos = next_pos;
                    }

                    astar_trap_area_data[next_pos.y * width_ + next_pos.x] = 0;

                    int index = map_bytes_[next_pos.y * width_ + next_pos.x];

                    if (index <= ASTAR_COST_TABLE_SIZE) {
                        tmp_point->G += AStarCostTable[index];
                    } else if (index == 127) {
                        tmp_point->G += GRAY_COST;
                    }

                    tmp_point->H   = CalHStar(next_pos, end);
                    tmp_point->F   = tmp_point->G + tmp_point->H;
                    tmp_point->Pos = next_pos;
                    // tmp_point->Previous = cur_str;
                    heap_str_->AddPoint(tmp_point);
                    // lstAStarPoints.push_back(*tmp_point);
                } else {
                    if ((ret == 1) && (!reach_target)) {
                        reach_target = true;
                        // result = ASR_SUCCESS;
                        pos_M1 = cur_str->Pos;
                        pos_M2 = next_pos;
                    } else {
                        continue;
                    }
                }
            }
        }

        CAStarPoint *cur_end = heap_end_->PopPoint();
        for (int i = 0; i < CONNECT_SIZE; i++) {
            Point next_pos(cur_end->Pos.x + sDx[i], cur_end->Pos.y + sDy[i]);
            // int8_t m_index = close_->GetIndex(cur_str->Pos);
            int8_t m_index = 2;

            if (CanPass(next_pos.x, next_pos.y, 0)) {
                int ret;
                if (!(ret = close_->AddToClose(i, m_index, &next_pos))) {
                    CAStarPoint *tmp_point = new CAStarPoint();

                    tmp_point->G = cur_end->G + sCost[i];
                    int index = map_bytes_[next_pos.y * width_ + next_pos.x];
                    if (index <= ASTAR_COST_TABLE_SIZE) {
                        tmp_point->G += AStarCostTable[index];
                    } else if (index == 127) {
                        tmp_point->G += GRAY_COST;
                    }
                    tmp_point->H   = CalHStar(next_pos, start);
                    tmp_point->F   = tmp_point->G + tmp_point->H;
                    tmp_point->Pos = next_pos;
                    // tmp_point->Previous = cur_str;
                    heap_end_->AddPoint(tmp_point);
                } else {
                    if ((ret == 1) && (!reach_target)) {
                        reach_target = true;
                        pos_M2       = cur_end->Pos;
                        pos_M1       = next_pos;
                    } else {
                        continue;
                    }
                }
            }
        }

        delete cur_str;
        delete cur_end;
    }

    if ((heap_end_->GetCount() == 0) && (!reach_target)) {
        result = ASR_END_TRAP;   // 终点被困
    }

    if ((heap_str_->GetCount() == 0) && (!reach_target)) {
        result = ASR_START_TRAP;   // 终点被困
        max_radius_from_start =
                sqrt((maxR_Pos.x - start.x) * (maxR_Pos.x - start.x) + (maxR_Pos.y - start.y) * (maxR_Pos.y - start.y));
    }

    std::vector<Point> tmp_points;

    if (reach_target) {
        std::vector<Point> start_points;
        Point m_now = pos_M1;
        while ((m_now.x != start.x) || (m_now.y != start.y)) {
            start_points.push_back(m_now);
            int8_t m_dxdy = close_->GetDxDy(m_now);
            m_now.x -= sDx[m_dxdy];
            m_now.y -= sDy[m_dxdy];
        }
        start_points.push_back(m_now);
        while (start_points.size() > 0) {
            tmp_points.push_back(start_points.back());
            start_points.pop_back();
        }
        std::vector<Point> end_points;
        m_now = pos_M2;
        while ((m_now.x != end.x) || (m_now.y != end.y)) {
            end_points.push_back(m_now);
            int8_t m_dxdy = close_->GetDxDy(m_now);
            m_now.x -= sDx[m_dxdy];
            m_now.y -= sDy[m_dxdy];
        }
        end_points.push_back(m_now);

        for (size_t i = 0; i < end_points.size(); i++) {
            tmp_points.push_back(end_points[i]);
        }
    }

    if (tmp_points.size() > 0) {
        track_points.clear();
        if (IS_KEY_POINT_PATH) {
            track_points.push_back(tmp_points[0]);
            for (size_t i = 1; i < tmp_points.size() - 1; i++) {
                if (!IsLineCrossWithMap(track_points.back(), tmp_points[i], ASTAR_CAN_LINE_COST)) {
                    track_points.push_back(tmp_points[i]);
                }
            }
            track_points.push_back(tmp_points[tmp_points.size() - 1]);
        } else {
            track_points = tmp_points;
        }
    }

    heap_str_->FreeList();
    heap_end_->FreeList();

    return result;
}

bool CAStar::updateMap(const Mat *map_bytes, Point *cur_pose) {
    if (map_bytes == NULL || width_ != map_bytes->cols || height_ != map_bytes->rows) {
        assert(false);
        return false;
    }

    memcpy(map_bytes_, reinterpret_cast<uint8_t *>(&map_bytes->data[0]), width_ * height_ * sizeof(uint8_t));

    // 起点涂白。
    if (cur_pose) {
        int x = cur_pose->x;
        int y = cur_pose->y;
        CCommonAlg::circleMat(255, x, y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, map_bytes_, width_, height_, 0, width_, 0,
                              height_);
    }
    AddBlackBorder();
    return true;
}

void CAStar::updateErrorMapGrayCost(const uint8_t *error_map_data, int channel_num) {
    int size = width_ * height_;
    for (int i = 0; i < size; i++) {
        if (error_map_data[i * channel_num] < 255 && map_bytes_[i] > 0) {
            map_bytes_[i] = 127;
        }
    }
}

bool CAStar::updateMap(const Mat *map_bytes, int width, int height, int min_x, int max_x, int min_y, int max_y,
                       Point cur_pose, bool do_black_erode, Mat *map_out) {
    if (width_ != width || height_ != height || map_bytes == NULL || width_ != map_bytes->cols ||
        height_ != map_bytes->rows || (map_out != NULL && (width_ != map_out->cols || height_ != map_out->rows))) {
        assert(false);
        return false;
    }

    memcpy(map_bytes_, reinterpret_cast<uint8_t *>(&map_bytes->data[0]), width * height * sizeof(uint8_t));
    min_x = min_x < 0 ? 0 : min_x;
    max_x = max_x > width - 1 ? width - 1 : max_x;
    min_y = min_y < 0 ? 0 : min_y;
    max_y = max_y > height - 1 ? height - 1 : max_y;

    int di[]       = ASTAR_COST_EXTEND_MASK;
    size_t di_size = sizeof(di) / sizeof(di[0]);
    if (do_black_erode) {
        for (int w = 0; w < ASTAR_DILATE_SIZE; w++) {
            for (int yy = min_y + 1; yy <= max_y - 1; yy++) {
                int yy_width = yy * width;
                for (int xx = min_x + 1; xx <= max_x - 1; xx++) {
                    int i = yy_width + xx;
                    if (map_bytes_[i] == w) {
                        for (size_t j = 0; j < di_size; j++) {
                            if (map_bytes_[i + di[j]] > VIRTUAL_OBST_REF) {
                                map_bytes_[i + di[j]] = w + 1;
                            }
                        }
                    }
                }
            }
        }

        for (int yy = min_y; yy <= max_y; yy++) {
            int yy_width = yy * width;
            for (int xx = min_x; xx <= max_x; xx++) {
                int i = yy_width + xx;
                if (map_bytes_[i] <= VIRTUAL_OBST_REF) {
                    map_bytes_[i] = 0;
                }
            }
        }
    }
    for (int w = 0; w < ASTAR_COST_TABLE_SIZE; w++) {
        for (int yy = min_y + 1; yy <= max_y - 1; yy++) {
            int yy_width = yy * width;
            for (int xx = min_x + 1; xx <= max_x - 1; xx++) {
                int i = yy_width + xx;
                if (map_bytes_[i] == w) {
                    for (size_t j = 0; j < di_size; j++) {
                        if (map_bytes_[i + di[j]] == 255) {
                            map_bytes_[i + di[j]] = w + 1;
                        }
                    }
                }
            }
        }
    }

    // 起点涂白。
    int x = cur_pose.x;
    int y = cur_pose.y;
    CCommonAlg::circleMat(255, x, y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, map_bytes_, width, height, 0, width, 0, height);

    if (map_out && (map_out->size() == (uint32_t)width * height)) {
        memcpy(reinterpret_cast<uint8_t *>(&map_out->data[0]), map_bytes_, width * height * sizeof(uint8_t));
    }
    AddBlackBorder();

    return true;
}

bool CAStar::updateMap(const Mat *map_bytes, const Mat *map_src, int width, int height, int min_x, int max_x, int min_y,
                       int max_y, Point cur_pose) {
    if (width_ != width || height_ != height || map_bytes == NULL || width_ != map_bytes->cols ||
        height_ != map_bytes->rows || map_src == NULL || width_ != map_src->cols || height_ != map_src->rows) {
        assert(false);
        return false;
    }

    memcpy(map_bytes_, reinterpret_cast<uint8_t *>(&map_src->data[0]), width * height * sizeof(uint8_t));

    min_x = min_x < 0 ? 0 : min_x;
    max_x = max_x > width - 1 ? width - 1 : max_x;
    min_y = min_y < 0 ? 0 : min_y;
    max_y = max_y > height - 1 ? height - 1 : max_y;

    int di[]       = ASTAR_COST_EXTEND_MASK;
    size_t di_size = sizeof(di) / sizeof(di[0]);

    for (int w = 0; w < ASTAR_DILATE_SIZE; w++) {
        for (int yy = min_y + 1; yy <= max_y - 1; yy++) {
            int yy_width = yy * width;
            for (int xx = min_x + 1; xx <= max_x - 1; xx++) {
                int i = yy_width + xx;
                if (map_bytes_[i] == w) {
                    for (size_t j = 0; j < di_size; j++) {
                        if (map_bytes_[i + di[j]] == 255) {
                            map_bytes_[i + di[j]] = w + 1;
                        }
                    }
                }
            }
        }
    }

    for (int yy = min_y; yy <= max_y; yy++) {
        int yy_width = yy * width;
        for (int xx = min_x; xx <= max_x; xx++) {
            int i = yy_width + xx;
            if (map_bytes_[i] <= ASTAR_DILATE_SIZE) {
                map_bytes_[i] = 0;
            }
        }
    }

    for (int w = 0; w < ASTAR_COST_TABLE_SIZE; w++) {
        for (int yy = min_y + 1; yy <= max_y - 1; yy++) {
            int yy_width = yy * width;
            for (int xx = min_x + 1; xx <= max_x - 1; xx++) {
                int i = yy_width + xx;
                if (map_bytes_[i] == w) {
                    for (size_t j = 0; j < di_size; j++) {
                        if (map_bytes_[i + di[j]] == 255) {
                            map_bytes_[i + di[j]] = w + 1;
                        }
                    }
                }
            }
        }
    }

    const uint8_t *map_bytes_data = &map_bytes->data[0];

    for (int yy = min_y; yy <= max_y; yy++) {
        int yy_width = yy * width;
        for (int xx = min_x; xx <= max_x; xx++) {
            int i = yy_width + xx;
            if (map_bytes_data[i] == 0) {
                map_bytes_[i] = 0;
            }
        }
    }

    // 起点涂白。
    int x = cur_pose.x;
    int y = cur_pose.y;
    CCommonAlg::circleMat(255, x, y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, map_bytes_, width, height, 0, width, 0, height);

    AddBlackBorder();

    return true;
}

// mapX mapZ mapG
bool CAStar::updateMapWithEdge(const Mat *map_bytes, const Mat *map_src, const Mat *map_edge, int width, int height,
                               int min_x, int max_x, int min_y, int max_y, Point cur_pose) {
    if (width_ != width || height_ != height || map_bytes == NULL || width_ != map_bytes->cols ||
        height_ != map_bytes->rows || map_src == NULL || width_ != map_src->cols || height_ != map_src->rows ||
        map_edge == NULL || width_ != map_edge->cols || height_ != map_edge->rows) {
        assert(false);
        return false;
    }

    // 以障碍物地图为基准
    memcpy(map_bytes_, reinterpret_cast<uint8_t *>(&map_src->data[0]), width * height * sizeof(uint8_t));

    min_x = min_x < 0 ? 0 : min_x;
    max_x = max_x > width - 1 ? width - 1 : max_x;
    min_y = min_y < 0 ? 0 : min_y;
    max_y = max_y > height - 1 ? height - 1 : max_y;

    // 膨胀障碍物地图
    int di[]                    = ASTAR_COST_EXTEND_MASK;
    size_t di_size              = sizeof(di) / sizeof(di[0]);
    const uint8_t *map_edge_data = &map_edge->data[0];
    for (int w = 0; w < ASTAR_COST_TABLE_SIZE; w++) {
        for (int yy = min_y + 1; yy <= max_y - 1; yy++) {
            int yy_width = yy * width;
            for (int xx = min_x + 1; xx <= max_x - 1; xx++) {
                int i = yy_width + xx;
                if (map_bytes_[i] == w) {
                    for (size_t j = 0; j < di_size; j++) {
                        // 与上UR_MAPG_WAIT_CLEAN 的黑色？白色？？？？？？
                        // mapEdge_data[i + di[j]]应该==255
                        if ((map_bytes_[i + di[j]] == 255) && (map_edge_data[i + di[j]] == 0)) {
                            map_bytes_[i + di[j]] = w + 1;
                        }
                    }
                }
            }
        }
    }

    // 套用一个mask模板 把_mapBytes涂黑
    const uint8_t *map_bytes_data = &map_bytes->data[0];
    for (int yy = min_y; yy <= max_y; yy++) {
        int yy_width = yy * width;
        for (int xx = min_x; xx <= max_x; xx++) {
            int i = yy_width + xx;
            if (map_bytes_data[i] == 0) {
                map_bytes_[i] = 0;
            }
        }
    }

    // 起点涂白。
    int x = cur_pose.x;
    int y = cur_pose.y;
    CCommonAlg::circleMat(255, x, y, CUR_POSE_FILL_WHITE_RADIUS_SIZE, map_bytes_, width, height, 0, width, 0, height);

    // 地图边界涂抹黑色
    AddBlackBorder();

    return true;
}

bool CAStar::UpdateGrayCost(const Mat &gray_cost_map) {
    if (width_ != gray_cost_map.cols || height_ != gray_cost_map.rows) {
        assert(false);
        return false;
    }

    const uint8_t *gray_cost_data = &gray_cost_map.data[0];
    int size                      = width_ * height_;
    for (int i = 0; i < size; i++) {
        if (gray_cost_data[i] == 127 && map_bytes_[i] > 0) {
            map_bytes_[i] = 127;
        }
    }

    return true;
}

void CAStar::AddBlackBorder() {
    int temp = width_ * (height_ - 1);   // 插入最外围黑边框
    for (int i = 0; i < width_; i++) {
        map_bytes_[i]        = 0;
        map_bytes_[temp + i] = 0;
    }
    for (int i = 0; i < height_; i++) {
        map_bytes_[i * width_]              = 0;
        map_bytes_[i * width_ + width_ - 1] = 0;
    }
}

void CAStar::GetStartConnectArea(Point start, Mat &start_connect_area) {
    Point zero(0, 0);
    Point maxP(width_ - 1, height_ - 1);

    if (!IsPointIn(start, zero, maxP)) {
        return;
    }

    uint8_t *start_search_area_data = &start_connect_area.data[0];

    memset(start_search_area_data, 0, width_ * height_);

    close_->Init(width_, height_);
    heap_str_->Init(width_, height_);

    CAStarPoint *start_astar_point = new CAStarPoint();
    start_astar_point->Pos         = start;
    start_astar_point->F           = 0;

    Point tmp(start.x, start.y);

    close_->AddToClose(9, 1, &tmp);
    heap_str_->AddPoint(start_astar_point);

    bool reach_target = false;
    Point pos_M1, pos_M2;

    while ((heap_str_->GetCount() > 0) && (!reach_target)) {
        {
            CAStarPoint *cur = heap_str_->PopPoint();

            for (int i = 0; i < CONNECT_SIZE; i++) {
                Point next_pos(cur->Pos.x + sDx[i], cur->Pos.y + sDy[i]);
                int8_t m_index = 1;

                if (CanPass(next_pos.x, next_pos.y, 0)) {
                    int ret;
                    Point temp(next_pos.x, next_pos.y);
                    if (!(ret = close_->AddToClose(i, m_index, &temp))) {
                        CAStarPoint *tmp_point = new CAStarPoint();

                        tmp_point->G = cur->G + sCost[i];
                        int index = map_bytes_[next_pos.y * width_ + next_pos.x];

                        start_search_area_data[next_pos.y * width_ + next_pos.x] = 255;

                        if (index <= ASTAR_COST_TABLE_SIZE) {
                            tmp_point->G += AStarCostTableS[index];
                        } else if (index == 127) {
                            tmp_point->G += GRAY_COST;
                        }
                        tmp_point->H   = 0;
                        tmp_point->F   = tmp_point->G + tmp_point->H;
                        tmp_point->Pos = next_pos;
                        heap_str_->AddPoint(tmp_point);
                    } else {
                        if ((ret == 1) && (!reach_target)) {
                            reach_target = true;
                            pos_M1       = cur->Pos;
                            pos_M2       = next_pos;
                        } else {
                            continue;
                        }
                    }
                }
            }
            delete cur;
        }
    }

    heap_str_->FreeList();
}

}   // namespace LDCV

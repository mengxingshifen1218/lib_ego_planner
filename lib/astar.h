/**
 * @file astar.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#ifndef LDCV_ASTAR_H_
#define LDCV_ASTAR_H_

#include <string.h>

#include <vector>

#include "Mat.h"
#include "astar_data_structure.h"
#include "cheap.h"
#include "close_map.h"
#include "type_def.h"

namespace LDCV {

class CAStar {
 public:
    CAStar(int width, int height);
    ~CAStar();

    enum AstarResult { ASR_START_TRAP = -1, ASR_END_TRAP, ASR_SUCCESS };

    /**
     * @brief 单目标A*
     *
     * @param track_points 规划路径点
     * @param max_radius_from_start 起点搜索最大半径
     * @param start 起点
     * @param end 终点
     * @param astar_trap_area 从起点开始搜索过的区域，仅规划失败有意义
     * @return CAStar::AstarResult 规划结果
     */
    CAStar::AstarResult FindPathWithRadius(std::vector<Point> *track_points, float &max_radius_from_start, Point &start,
                                           Point &end, Mat &astar_trap_area);

    /**
     * @brief 单目标A*
     *
     * @param track_points 规划路径点
     * @param max_radius_from_start 起点搜索最大半径
     * @param start 起点
     * @param end 终点
     * @param astar_trap_area 从起点开始搜索过的区域
     * @return CAStar::AstarResult
     */
    CAStar::AstarResult FindPathWithRadiusFast(std::vector<Point> *track_points, float &max_radius_from_start,
                                               Point &start, Point &end, Mat &astar_trap_area);

    /**
     * @brief 多目标A*搜索
     *
     * @param rect_min_with_border 搜索框左下角点
     * @param rect_max_with_border 搜索框右上角点
     * @param start 起点
     * @param track_points 规划路径点
     * @param target_list 目标点集合
     * @param nearest_id 最近点索引
     * @return true
     * @return false
     */
    bool FindPathWithMultiTarget(Point rect_min_with_border, Point rect_max_with_border, Point &start,
                                 std::vector<Point> &track_points, std::vector<Point> &target_list, int *nearest_id);

    /**
     * @brief 多目标A*搜索 时间优化 内存资源增多
     *
     * @param rect_min_with_border 搜索框左下角点
     * @param rect_max_with_border 搜索框右上角点
     * @param start 起点
     * @param track_points 规划路径点
     * @param target_list 目标点集合
     * @param nearest_id 最近点索引
     * @return true
     * @return false
     */
    bool FindPathWithMultiTargetFast(Point rect_min_with_border, Point rect_max_with_border, Point &start,
                                     std::vector<Point> &track_points, std::vector<Point> &target_list,
                                     int *nearest_id);

    /**
     * @brief 目标地图A*搜索
     *
     * @param rect_min_with_border 搜索框左下角点
     * @param rect_max_with_border 搜索框右上角点
     * @param start 起点
     * @param target_map 目标地图
     * @param output_point 搜索到的
     * @param start_search_area 从起点开始搜索过的区域，涂白
     * @param astar_trap_area 从起点开始搜索过的区域，涂黑
     * @param max_radius_from_start 起点搜索最大半径
     * @param track_points 规划路径点
     * @return true
     * @return false
     */
    bool FindNearestPointToTargetMap(Point rect_min_with_border, Point rect_max_with_border, Point start,
                                     const Mat *target_map, Point &output_point, Mat &start_search_area,
                                     Mat &astar_trap_area, float *max_radius_from_start = nullptr,
                                     std::vector<Point> *track_points = nullptr);

    /**
     * @brief 单目标A*
     *
     * @param start 起点
     * @param end 终点
     * @param track_points 规划路径点
     * @param max_radius_from_start 起点搜索最大半径
     * @param astar_trap_area 从起点开始搜索过的区域
     * @return CAStar::AstarResult 规划结果
     */
    CAStar::AstarResult FindPath(Point &start, Point &end, std::vector<Point> &track_points,
                                 float &max_radius_from_start, Mat &astar_trap_area);

    /**
     * @brief 单目标A*
     *
     * @param start 起点
     * @param end 终点
     * @param track_points 规划路径点
     * @param max_radius_from_start 起点搜索最大半径
     * @param astar_trap_area 从起点开始搜索过的区域
     * @return CAStar::AstarResult 规划结果
     */
    CAStar::AstarResult FindPathNearest(Point &start, Point &end, std::vector<Point> &track_points,
                                        float &max_radius_from_start, Mat &astar_trap_area);

    /**
     * @brief 更新A*搜索地图
     *
     * @param map_bytes 传入地图数据
     * @param cur_pose 起点坐标
     * @return true
     * @return false
     */
    bool updateMap(const Mat *map_bytes, Point *cur_pose = NULL);

    /**
     * @brief 更新A*搜索地图
     * 
     * @param map_bytes 传入地图数据
     * @param width 地图宽度
     * @param height 地图高度
     * @param min_x 搜索区域x坐标最小值
     * @param max_x 搜索区域x坐标最大值
     * @param min_y 搜索区域y坐标最小值
     * @param max_y 搜索区域y坐标最大值
     * @param cur_pose 起点坐标
     * @param do_black_erode 是否膨胀黑色障碍物
     * @param map_out 输出处理后的地图
     * @return true 
     * @return false 
     */
    bool updateMap(const Mat *map_bytes, int width, int height, int min_x, int max_x, int min_y, int max_y,
                   Point cur_pose, bool do_black_erode = true, Mat *map_out = NULL);

    /**
     * @brief 更新A*搜索地图
     * 
     * @param map_bytes 传入掩膜地图,为0的区域不可达
     * @param map_src 传入基准地图数据
     * @param width 地图宽度
     * @param height 地图高度
     * @param min_x 搜索区域x坐标最小值
     * @param max_x 搜索区域x坐标最大值
     * @param min_y 搜索区域y坐标最小值
     * @param max_y 搜索区域y坐标最大值
     * @param cur_pose 起点坐标
     * @return true 
     * @return false 
     */
    bool updateMap(const Mat *map_bytes, const Mat *map_src, int width, int height, int min_x, int max_x, int min_y,
                   int max_y, Point cur_pose);

    /**
     * @brief 
     * 
     * @param map_bytes 传入掩膜地图,为0的区域不可达
     * @param map_src 传入基准地图数据
     * @param map_edge 地图数据，为0时代价向外膨胀
     * @param width 地图宽度
     * @param height 地图高度
     * @param min_x 搜索区域x坐标最小值
     * @param max_x 搜索区域x坐标最大值
     * @param min_y 搜索区域y坐标最小值
     * @param max_y 搜索区域y坐标最大值
     * @param cur_pose 起点坐标
     * @return true 
     * @return false 
     */
    bool updateMapWithEdge(const Mat *map_bytes, const Mat *map_src, const Mat *map_edge, int width, int height,
                           int min_x, int max_x, int min_y, int max_y, Point cur_pose);

    /**
     * @brief 更新slam灰色区域权重地图
     * 
     * @param gray_cost_map 
     * @return true 
     * @return false 
     */
    bool UpdateGrayCost(const Mat &gray_cost_map);

    /**
     * @brief 插入最外围黑边框
     * 
     */
    void AddBlackBorder();

    /**
     * @brief 获取起点连通域
     * 
     * @param start 起点坐标
     * @param start_connect_area 输出的起点连通区域地图
     */
    void GetStartConnectArea(Point start, Mat &start_connect_area);

    /**
     * @brief 更新error map灰色区域权重地图
     * 
     * @param error_map_data error地图
     * @param channel_num 地图通道数，永远为1
     */
    void updateErrorMapGrayCost(const uint8_t *error_map_data, int channel_num);

 private:
    int CalHStar(Point start, Point end);

    float GetDistance(Point start, Point end);

    bool CanPass(int x, int y, uint8_t area_gray_ref);

    bool IsLineCrossWithMap(Point p1, Point p2, uint8_t area_gray_ref = 0);

    bool fillWhiteInRTMap(uint8_t *RT_map_bytes, Point cur_position, int fill_radius);

    bool fillWhiteInRTMap(uint8_t *RT_map_bytes, Point cur_position);   // 往终点方向自动填白

    bool IsPointIn(Point &point, Point &point_min, Point &point_max);

 private:
    int width_;
    int height_;
    uint8_t *map_bytes_;

    Point obstacle_point_;
    std::vector<Point> last_path_list_;
    CCloseMap *close_;
    CHeap *heap_str_, *heap_end_;
};
}   // namespace LDCV
#endif   // LDCV_ASTAR_H_

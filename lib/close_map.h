/**
 * @file close_map.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef LDCV_CLOSE_MAP_H_
#define LDCV_CLOSE_MAP_H_

#include <assert.h>
#include <string.h>

#include <vector>

#include "type_def.h"
#include "astar_data_structure.h"

#ifdef OPENCV
    #include <opencv2/opencv.hpp>
#endif

namespace LDCV {

class CCloseMap {
 public:
    CCloseMap(int width, int height);

    ~CCloseMap();

    /**
     * @brief 初始化A*搜索close地图，并且将地图边缘设置为不可搜索
     * 
     * @param width 地图宽度
     * @param height 地图高度
     */
    void Init(int width, int height);

    /**
     * @brief 将当前点加入close地图，地图上每个节点包含
     * 上一节点位置和从是起点或终点搜索到这个点
     * 
     * @param dxdyNum 父节点位置与当前节点索引差值
     * @param m_index 从起点搜索为1，终点搜索到为2
     * @param point 搜索点
     * @return int 0：加入成功并且point不再map中 1：point从起点和终点同时拓展 2：已经加入并且不满足同时拓展
     */
    int AddToClose(int8_t dxdyNum, int8_t m_index, Point *point);

    /**
     * @brief 将当前点加入close地图
     * 
     * @param point 搜索点
     * @return true 加入成功
     * @return false 已经在close map中，添加失败
     */
    bool AddToClose(CAStarPoint *point);

    /**
     * @brief Get the Index object
     * 
     * @param point 搜索点
     * @return int8_t point在close map中的索引
     */
    int8_t GetIndex(Point &point);

    /**
     * @brief 获取当前point与其父节点之前的索引差值
     * 
     * @param point 搜索点
     * @return int8_t 索引差值
     */
    int8_t GetDxDy(Point &point);

    /**
     * @brief 判断当前point是否已经在close map中
     * 
     * @param point 搜索点
     * @return true 已经添加过
     * @return false 未添加
     */
    bool InClose(Point &point);

#ifdef OPENCV
    cv::Mat GetOpencvMat();
#endif

 private:
    uint8_t *close_ = NULL;
    int width_;
    int height_;
    int buffer_size_w_ = 0;
    int buffer_size_h_ = 0;
};

}   // namespace LDCV
#endif   // LDCV_CLOSE_MAP_H_

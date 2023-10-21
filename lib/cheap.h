/**
 * @file cheap.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#ifndef ASTAR_CHEAP_H_
#define ASTAR_CHEAP_H_

#include <iostream>
#include <vector>

#include "astar_data_structure.h"

namespace LDCV {

class CHeap {
 public:
    CHeap(int width, int height);
    ~CHeap();

    /**
     * @brief 清空list，手动释放内存
     * 
     * @return true 
     * @return false 
     */
    bool FreeList() {
        for (int i = 0; i < count_; i++) {
            delete (list_[i]);
        }
        count_ = 0;
        return 0;
    }

    /**
     * @brief 初始化最小堆
     * 
     * @param width 输入地图宽度
     * @param height 输入地图高度
     */
    void Init(int width, int height);

    /**
     * @brief 获取当前堆中节点个数
     * 
     * @return int 返回节点个数
     */
    int GetCount() { return count_; }

    /**
     * @brief 在末尾插入节点，并且上浮该节点重新排序
     * 
     * @param point 插入AStarPoint节点，以A*搜索点总代价排序
     */
    void AddPoint(CAStarPoint* point);

    /**
     * @brief 弹出根节点，末尾元素放到根节点后下沉重新排序
     * 
     * @return CAStarPoint* 
     */
    CAStarPoint* PopPoint();

    int count_        = 0;
    int width_        = 0;
    int height_       = 0;
    int buffer_size_w_ = 0;
    int buffer_size_h_ = 0;

 private:
    CAStarPoint** list_;
};
}   // namespace LDCV

#endif   // ASTAR_CHEAP_H_

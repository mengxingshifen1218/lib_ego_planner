/**
 * @file astar_data_structure.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-26
 * 
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 * 
 */
#ifndef LDCV_ASTAR_DATA_STRUCTURE_H_
#define LDCV_ASTAR_DATA_STRUCTURE_H_

#include "type_def.h"

namespace LDCV {

class CAStarPoint {
 public:
    CAStarPoint() {
        Pos = Point(0, 0);
        F   = 0;
        H   = 0;
        G   = 0;
        R   = 0;
    }

 public:
    int F = 0;
    int H = 0;
    int G = 0;
    int R = 0;

 public:
    Point Pos;
};
}   // namespace LDCV

#endif   // LDCV_ASTAR_DATA_STRUCTURE_H_

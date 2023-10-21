/**
 * @file Mat.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-26
 * 
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 * 
 */
#ifndef LDCV_MAT_H_
#define LDCV_MAT_H_

#include <vector>

#include "type_def.h"

namespace LDCV {

class Mat {
 public:
    Mat();
    Mat(int _rows, int _cols);
    Mat(int _rows, int _cols, uint8_t *data);
    Mat(const Mat &m);

    ~Mat();

    Mat clone();

    Mat &operator=(const Mat &m);
    uint32_t size();

 public:
    int rows = 0;
    int cols = 0;

    uint8_t *data = nullptr;

 private:
    bool using_inner_buffer_ = true;
};
}   // namespace LDCV

#endif  // LDCV_MAT_H_

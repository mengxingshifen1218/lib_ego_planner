/**
 * @file mat.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#include "Mat.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace LDCV {

Mat::Mat() {
    rows = 0;
    cols = 0;
    if (data && using_inner_buffer_) {
        delete[] data;
        data = nullptr;
    }
}

Mat::Mat(int _rows, int _cols) {
    rows = _rows;
    cols = _cols;

    if (data && using_inner_buffer_) {
        delete[] data;
        data = nullptr;
    }
    data = new uint8_t[rows * cols];
    memset(data, 0, rows * cols);
    using_inner_buffer_ = true;
}

Mat::Mat(int _rows, int _cols, uint8_t* _data) {
    rows = _rows;
    cols = _cols;

    if (data && using_inner_buffer_) {
        delete[] data;
        data = nullptr;
    }
    data                = _data;
    using_inner_buffer_ = false;
}

// opencv采用引用数据
Mat::Mat(const Mat& m) {
    rows = m.rows;
    cols = m.cols;

    if (data && using_inner_buffer_) {
        delete[] data;
        data = nullptr;
    }
    data = new uint8_t[rows * cols];
    memcpy(data, m.data, rows * cols);
    using_inner_buffer_ = true;
}

Mat::~Mat() {
    if (data && using_inner_buffer_) {
        delete[] data;
        data = nullptr;
    }
}

Mat Mat::clone() {
    Mat re(rows, cols);
    memcpy(re.data, data, rows * cols);
    return re;
}

uint32_t Mat::size() { return rows * cols; }

Mat& Mat::operator=(const Mat& m) {
    if (this == &m)
        return *this;

    // 深拷贝 采用clone函数
    if (m.rows != rows && m.cols != cols) {
        if (data && using_inner_buffer_) {
            delete[] data;
            data = nullptr;
        }
        rows                = m.rows;
        cols                = m.cols;
        data                = new uint8_t[rows * cols];
        using_inner_buffer_ = true;
    }
    memcpy(data, m.data, rows * cols);

    return *this;
}

}   // namespace LDCV

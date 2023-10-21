/**
 * @file close_map.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-26
 * 
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 * 
 */
#include "close_map.h"

namespace LDCV {
CCloseMap::CCloseMap(int width, int height) {
    width_  = width;
    height_ = height;

    buffer_size_w_ = width_;
    buffer_size_h_ = height_;

    close_ = new uint8_t[width_ * height_];
}

CCloseMap::~CCloseMap() {
    delete[] close_;
    close_ = NULL;
}

// 这个函数用于减少运算 没有必要全图搜索
void CCloseMap::Init(int width, int height) {
    assert(width <= buffer_size_w_ && height <= buffer_size_h_);
    width_  = width;
    height_ = height;
    memset(close_, 0, width * height);
    for (int i = 0; i < width; i++) {
        close_[i]                        = 3;
        close_[width_ * height_ - i - 1] = 3;
    }
    for (int j = 0; j < height; j++) {
        close_[j * width]           = 3;
        close_[(j + 1) * width - 1] = 3;
    }
}

int CCloseMap::AddToClose(int8_t dxdyNum, int8_t m_index, Point *point) {
    int temp = point->y * width_ + point->x;

    if (!(close_[temp])) {
        close_[temp] = (dxdyNum << 4) + m_index;
        return 0;
    }

    if ((close_[temp] & 0x0f) + m_index == 3) {
        return 1;
    } else {
        return 2;
    }
}

bool CCloseMap::AddToClose(CAStarPoint *point) {
    int temp = point->Pos.y * width_ + point->Pos.x;
    if (close_[temp] == 0) {
        close_[temp] = 1;
        return true;
    } else {
        return false;
    }
}

int8_t CCloseMap::GetIndex(Point &point) { return (int8_t)close_[point.y * width_ + point.x] & 0x0f; }

int8_t CCloseMap::GetDxDy(Point &point) { return (int8_t)close_[point.y * width_ + point.x] >> 4; }

#ifdef OPENCV
cv::Mat CCloseMap::GetOpencvMat() { return cv::Mat(height_, width_, CV_8U, close_).clone(); }
#endif

}  // namespace LDCV

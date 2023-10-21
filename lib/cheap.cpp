/**
 * @file cheap.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-09-26
 * 
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 * 
 */
#include "cheap.h"

#include <assert.h> /* assert */
#include <malloc.h>


namespace LDCV {

CHeap::CHeap(int width, int height) {
    width_        = width;
    height_       = height;
    list_         = reinterpret_cast<CAStarPoint**>(malloc(width * height * sizeof(CAStarPoint*)));
    buffer_size_w_ = width_;
    buffer_size_h_ = height_;
}

void CHeap::Init(int width, int height) {
    assert(width <= buffer_size_w_ && height <= buffer_size_h_);
    width_  = width;
    height_ = height;
    count_  = 0;
}

CHeap::~CHeap() {
    free(list_);
    list_ = NULL;
}

void CHeap::AddPoint(CAStarPoint* point) {
    bool error_ret = (list_ == NULL || count_ < 0 || count_ >= width_ * height_ /** (int)sizeof(CAStarPoint*)*/);
    if (error_ret) {
        assert(!error_ret);
        return;
    }

    list_[count_] = point;
    count_++;

    int inIndex = count_ - 1;
    while (inIndex > 0) {
        if (list_[inIndex]->F < list_[(inIndex - 1) / 2]->F) {
            CAStarPoint* temp        = list_[inIndex];
            list_[inIndex]           = list_[(inIndex - 1) / 2];
            list_[(inIndex - 1) / 2] = temp;
            inIndex                  = (inIndex - 1) / 2;
        } else {
            break;
        }
    }
}

CAStarPoint* CHeap::PopPoint() {
    CAStarPoint* res = list_[0];
    list_[0]         = list_[count_ - 1];
    count_--;
    int index    = 0;
    int maxIndex = count_ - 1;
    while ((index + 1) * 2 - 1 <= maxIndex) {
        int left  = (index + 1) * 2 - 1;
        int right = (index + 1) * 2;

        int min;
        if (right > maxIndex) {
            min = left;
        } else {
            min = list_[left]->F < list_[right]->F ? left : right;
        }
        if (list_[index]->F > list_[min]->F) {
            CAStarPoint* temp = list_[index];
            list_[index]      = list_[min];
            list_[min]        = temp;
            index             = min;
        } else {
            break;
        }
    }
    return res;
}
}   // namespace LDCV

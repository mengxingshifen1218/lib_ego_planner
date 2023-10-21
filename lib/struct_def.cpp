/**
 * @file struct_def.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#include "struct_def.h"

#include "type_def.h"

namespace LDCV {

PointF32::PointF32() {
    x = 0.0;
    y = 0.0;
}
PointF32::PointF32(PointI32 p) {
    x = p.x;
    y = p.y;
}
PointF32::PointF32(float _x, float _y) {
    x = _x;
    y = _y;
}

PointI32::PointI32() {
    x = 0;
    y = 0;
}
PointI32::PointI32(int32_t _x, int32_t _y) {
    x = _x;
    y = _y;
}
PointI32::PointI32(PointF32 fp) {
    x = (fp.x + 0.5);
    y = (fp.y + 0.5);
}

bool PointI32::operator==(const PointI32 &r) { return (x == r.x && y == r.y); }

bool PointI32::operator!=(const PointI32 &r) { return (x != r.x || y != r.y); }

void PointI32::operator=(const PointF32 &r) {
    x = r.x + 0.5;
    y = r.y + 0.5;
}

RectI32::RectI32() {
    x = 0;
    y = 0;
    w = 0;
    h = 0;
}
RectI32::RectI32(int _x, int _y, int _w, int _h) {
    x = _x;
    y = _y;
    w = _w;
    h = _h;
}

}   // namespace LDCV

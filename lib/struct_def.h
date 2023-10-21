/**
 * @file struct_def.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#ifndef LDCV_STRUCT_DEF_H_
#define LDCV_STRUCT_DEF_H_

#ifdef OPENCV
    #include "opencv2/opencv.hpp"
#endif

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace LDCV {

struct PointI32;
struct PointF32;
struct RectI32;

struct PointF32 {
    float x = 0.0;
    float y = 0.0;
    PointF32();
    explicit PointF32(PointI32 p);
    PointF32(float _x, float _y);
};
struct PointI32 {
    int x = 0;
    int y = 0;
    PointI32();
    PointI32(int _x, int _y);
    explicit PointI32(PointF32 fp);
    bool operator==(const PointI32 &r);
    bool operator!=(const PointI32 &r);
    void operator=(const PointF32 &r);
};
struct RectI32 {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    RectI32();
    RectI32(int _x, int _y, int _w, int _h);
};

struct LineSegment {
    float x1, y1, x2, y2, angle;
    int label;
};

struct Point4f {
    float x1, y1, x2, y2;
};

struct Vector4f {
    float x;
    float y;
    float z;
    float w;

    Vector4f() {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }

    Vector4f(float _x, float _y, float _z, float _w) {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }
};

struct Vector3d {
    double x;
    double y;
    double z;

    Vector3d() {
        x = 0;
        y = 0;
        z = 0;
    }

    Vector3d(float _x, float _y, float _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    Vector3d(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    Vector3d operator*(double n) {
        Vector3d res;
        res.x = this->x * n;
        res.y = this->y * n;
        res.z = this->z * n;
        return res;
    }

    double dot(Vector3d pt) { return (this->x * pt.x + this->y * pt.y + this->z * pt.z); }

    Vector3d cross(const Vector3d &b) {
        Vector3d res;
        res.x = this->y * b.z - b.y * this->z;
        res.y = this->z * b.x - this->x * b.z;
        res.z = this->x * b.y - this->y * b.x;
        return res;
    }
};

}   // namespace LDCV

#endif   // LDCV_STRUCT_DEF_H_

/**
 * @file common_alg.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#include "common_alg.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

using std::vector;
using std::pair;


#ifndef min
    #define min(a, b) ((a) < (b) ? (a) : (b))
#endif   // !min
#ifndef max
    #define max(a, b) ((a) > (b) ? (a) : (b))
#endif   // !max

namespace LDCV {

CCommonAlg::CCommonAlg() {}

CCommonAlg::~CCommonAlg() {}

double CCommonAlg::calcVectAngel(double x, double y) {
    double angel;
    if (x < 0) {
        angel = atan(y / x) + M_PI;
        return angel;
    } else if (x > 0) {
        angel = atan(y / x);
        if (angel < 0) {
            angel += 2 * M_PI;
        }
        return angel;
    } else if (y > 0) {
        return 0.5 * M_PI;
    } else {
        return 1.5 * M_PI;
    }
}

void CCommonAlg::rotateVect(double& x, double& y, double deta_angel) {
    double xx = x, yy = y;
    x = xx * cos(deta_angel) + yy * (-sin(deta_angel));
    y = xx * (sin(deta_angel)) + yy * cos(deta_angel);
}

void CCommonAlg::rotateVect(float& x, float& y, float deta_angel) {
    float xx = x, yy = y;
    x = xx * cos(deta_angel) + yy * (-sin(deta_angel));
    y = xx * (sin(deta_angel)) + yy * cos(deta_angel);
}

double CCommonAlg::calcAngelDiffAbs(double angel1, double angel2) {
    double angel_diff = fabs(angel1 - angel2);
    if (angel_diff > M_PI) {
        angel_diff = 2.0 * M_PI - angel_diff;
    }
    return angel_diff;
}

double CCommonAlg::calcAngelDiff(double angel1, double angel2) {
    double angel_diff = angel1 - angel2;
    while (angel_diff > M_PI) {
        angel_diff -= 2 * M_PI;
    }
    while (angel_diff < -M_PI) {
        angel_diff += 2 * M_PI;
    }

    return angel_diff;
}

void CCommonAlg::lineMat(unsigned char value, int x1, int y1, int x2, int y2, unsigned char* data, int width,
                         int height, bool domain_4, int channel_num) {
#if 0
    if (data) {
        int start_x = x1 < x2 ? x1 : x2;
        int end_x = x1 > x2 ? x1 : x2;
        int start_y = y1 < y2 ? y1 : y2;
        int end_y = y1 > y2 ? y1 : y2;
        if (x1 == x2) {
            for (int i = start_y; i <= end_y; i++) {
                if (i >= 0 && i < height&&x1 >= 0 && x1 < width)
                    data[(i*width + x1)*channel_num] = value;
            }
        } else if (y1 == y2) {
            for (int i = start_x; i <= end_x; i++) {
                if (y1 >= 0 && y1 < height&&i >= 0 && i < width)
                    data[(y1*width + i)*channel_num] = value;
            }
        } else {
            double theta = calcVectAngel(x2 - x1, y2 - y1);
            double deta_x = cos(theta);
            double deta_y = sin(theta);

            if (fabs(deta_x) > fabs(deta_y)) {
                double tmp = deta_x;
                deta_x = deta_x > 0 ? 1.0 : -1.0;
                deta_y = deta_y / tmp * deta_x;
            } else {
                double tmp = deta_y;
                deta_y = deta_y > 0 ? 1.0 : -1.0;
                deta_x = deta_x / tmp * deta_y;
            }

            double x = x1 + 0.5, y = y1 + 0.5;  // 四舍五入
            // int pre_x = x1, pre_y = y1;

            for (int order = 0; static_cast<int>(x) != x2 && static_cast<int>(y) != y2; order++) {
                if (static_cast<int>(y) >= 0 && static_cast<int>(y) < height &&
                    static_cast<int>(x) >= 0 && static_cast<int>(x) < width) {
                    data[(static_cast<int>(y)*width + static_cast<int>(x))*channel_num] = value;
                }

                if (domain_4) {
                    if (order % 2 == 0)
                        x += deta_x;
                    else
                        y += deta_y;
                } else {
                    x += deta_x;
                    y += deta_y;
                }
            }

            if (x2 >= 0 && x2 < width&& y2 >= 0 && y2 < height)
                data[(y2*width + x2)*channel_num] = value;
        }
    }
#else
    if (data) {
        {
            double theta  = calcVectAngel(x2 - x1, y2 - y1);
            double deta_x = cos(theta);
            double deta_y = sin(theta);
            bool x_or_y   = true;
            if (fabs(deta_x) > fabs(deta_y)) {
                x_or_y     = true;
                double tmp = deta_x;
                deta_x     = deta_x > 0 ? 1.0 : -1.0;
                deta_y     = deta_y / tmp * deta_x;
            } else {
                x_or_y     = false;
                double tmp = deta_y;
                deta_y     = deta_y > 0 ? 1.0 : -1.0;
                deta_x     = deta_x / tmp * deta_y;
            }

            double x = x1 + 0.5f, y = y1 + 0.5f;   // 四舍五入。
            if (x_or_y) {
                while ((deta_x > 0 && x <= x2) || (deta_x < 0 && x >= x2)) {
                    data[(static_cast<int>(x) + static_cast<int>(y) * width) * channel_num] = value;
                    x += deta_x;
                    y += deta_y;
                }
            } else {
                while ((deta_y > 0 && y <= y2) || (deta_y < 0 && y >= y2)) {
                    data[(static_cast<int>(x) + static_cast<int>(y) * width) * channel_num] = value;
                    x += deta_x;
                    y += deta_y;
                }
            }
        }
    }
#endif
}

bool CCommonAlg::LineHit(int& hit_x, int& hit_y, unsigned char lower_equal_value, int x1, int y1, int x2, int y2,
                         unsigned char* data, int width, int height, int channel_num) {
    if (data) {
        {
            double theta  = calcVectAngel(x2 - x1, y2 - y1);
            double deta_x = cos(theta);
            double deta_y = sin(theta);
            bool x_or_y   = true;
            if (fabs(deta_x) > fabs(deta_y)) {
                x_or_y     = true;
                double tmp = deta_x;
                deta_x     = deta_x > 0 ? 1.0 : -1.0;
                deta_y     = deta_y / tmp * deta_x;
            } else {
                x_or_y     = false;
                double tmp = deta_y;
                deta_y     = deta_y > 0 ? 1.0 : -1.0;
                deta_x     = deta_x / tmp * deta_y;
            }

            double x = x1 + 0.5f, y = y1 + 0.5f;   // 四舍五入。
            if (x_or_y) {
                while ((deta_x > 0 && x <= x2) || (deta_x < 0 && x >= x2)) {
                    if (data[(static_cast<int>(x) + static_cast<int>(y) * width) * channel_num] <= lower_equal_value) {
                        hit_x = static_cast<int>(x);
                        hit_y = static_cast<int>(y);
                        return true;
                    }
                    x += deta_x;
                    y += deta_y;
                }
            } else {
                while ((deta_y > 0 && y <= y2) || (deta_y < 0 && y >= y2)) {
                    if (data[(static_cast<int>(x) + static_cast<int>(y) * width) * channel_num] <= lower_equal_value) {
                        hit_x = static_cast<int>(x);
                        hit_y = static_cast<int>(y);
                        return true;
                    }
                    x += deta_x;
                    y += deta_y;
                }
            }
        }
    }

    return false;
}

int CCommonAlg::DrawCleanedAndSum(unsigned char value, int centre_x, int centre_y, int draw_half_width,
                                  unsigned char* data, int width, int height, int minx, int maxx, int miny, int maxy,
                                  int channel_num) {
    int min_x = centre_x - draw_half_width;
    int max_x = centre_x + draw_half_width;
    int min_y = centre_y - draw_half_width;
    int max_y = centre_y + draw_half_width;

    min_x = min_x < minx ? minx : min_x;
    max_x = max_x > maxx ? maxx : max_x;
    min_y = min_y < miny ? miny : min_y;
    max_y = max_y > maxy ? maxy : max_y;

    min_x = min_x < 0 ? 0 : min_x;
    max_x = max_x > width - 1 ? width - 1 : max_x;
    min_y = min_y < 0 ? 0 : min_y;
    max_y = max_y > height - 1 ? height - 1 : max_y;

    int tol_add_cnt = 0;

    for (int xx = min_x; xx <= max_x; xx++) {
        for (int yy = min_y; yy <= max_y; yy++) {
            int index = yy * width + xx;
            index *= channel_num;
            if (data[index] != 255) {
                data[index] = value;
                if (value > 0) {
                    tol_add_cnt++;
                }
            }
        }
    }

    return tol_add_cnt;
}

void CCommonAlg::circleMat(unsigned char value, int centre_x, int centre_y, float radius, unsigned char* data,
                           int width, int height, int minx, int maxx, int miny, int maxy, int channel_num) {
    int min_x = centre_x - static_cast<int>(radius);
    int max_x = centre_x + static_cast<int>(radius);
    int min_y = centre_y - static_cast<int>(radius);
    int max_y = centre_y + static_cast<int>(radius);

    // LA_PRINT("-----%f\n", radius);

    if (minx >= 0 && maxx >= 0 && miny >= 0 && maxy >= 0) {
        min_x = min_x < minx ? minx : min_x;
        max_x = max_x > maxx ? maxx : max_x;
        min_y = min_y < miny ? miny : min_y;
        max_y = max_y > maxy ? maxy : max_y;
    }

    {
        min_x = min_x < 0 ? 0 : min_x;
        max_x = max_x > width - 1 ? width - 1 : max_x;
        min_y = min_y < 0 ? 0 : min_y;
        max_y = max_y > height - 1 ? height - 1 : max_y;
    }

    for (int xx = min_x; xx <= max_x; xx++) {
        int pow_x = static_cast<int>(pow(xx - centre_x, 2));
        for (int yy = min_y; yy <= max_y; yy++) {
            double dist = sqrt(pow_x + pow(yy - centre_y, 2));
            if (dist <= radius) {
                data[yy * width * channel_num + xx * channel_num] = value;
            }
        }
    }
}

void CCommonAlg::DrawRectangle(unsigned char value, int x, int y, int w, int h, unsigned char* data, int width,
                               int height, int minx, int maxx, int miny, int maxy, int channel_num) {
    int min_x = x;
    int max_x = x + w;
    int min_y = y;
    int max_y = y + h;

    if (minx >= 0 && maxx >= 0 && miny >= 0 && maxy >= 0) {
        min_x = min_x < minx ? minx : min_x;
        max_x = max_x > maxx ? maxx : max_x;
        min_y = min_y < miny ? miny : min_y;
        max_y = max_y > maxy ? maxy : max_y;
    }

    {
        min_x = min_x < 0 ? 0 : min_x;
        max_x = max_x > width - 1 ? width - 1 : max_x;
        min_y = min_y < 0 ? 0 : min_y;
        max_y = max_y > height - 1 ? height - 1 : max_y;
    }

    for (int xx = min_x; xx <= max_x; xx++) {
        for (int yy = min_y; yy <= max_y; yy++) {
            data[yy * width * channel_num + xx * channel_num] = value;
        }
    }
}

void CCommonAlg::edge_set(unsigned char* data, bool is_vertical_line, bool is_horison_line, int width, int height,
                          int value, int xmin, int xmax, int ymin, int ymax, int channel_num) {
    // check edge
    xmin = xmin < 0 ? 0 : xmin;
    xmax = xmax > width - 1 ? width - 1 : xmax;
    ymin = ymin < 0 ? 0 : ymin;
    ymax = ymax > height - 1 ? height - 1 : ymax;

    if (is_vertical_line) {
        for (int i = ymin; i <= ymax; i++) {
            int temp0                          = i * width;
            data[(temp0 + xmin) * channel_num] = value;
            data[(temp0 + xmax) * channel_num] = value;
        }
    }

    if (is_horison_line) {
        int temp1 = ymin * width;
        int temp2 = ymax * width;

        for (int j = xmin; j <= xmax; j++) {
            data[(temp1 + j) * channel_num] = value;
            data[(temp2 + j) * channel_num] = value;
        }
    }
}

const bool CCommonAlg::kernal3_3[9]  = {1, 1, 1, 1, 1, 1, 1, 1, 1};
const bool CCommonAlg::kernal5_5[25] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
const bool CCommonAlg::kernal7_7[49] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

void CCommonAlg::dilate_erode(unsigned char* data, int width, int height, bool dilate_or_erode,
                              int kernal_size_3_or_5_or_7, int xmin, int xmax, int ymin, int ymax, int channel_num) {
    int size                 = width * height;
    unsigned char* data_copy = new unsigned char[size];
    // memcpy(data_copy, data, width*height);
    for (int i = 0; i < size; i++) data_copy[i] = *(data + i * channel_num);

    kernal_size_3_or_5_or_7 = kernal_size_3_or_5_or_7 < 3 ? 3 : kernal_size_3_or_5_or_7;
    kernal_size_3_or_5_or_7 = kernal_size_3_or_5_or_7 > 7 ? 7 : kernal_size_3_or_5_or_7;

    const bool* kernal;
    if (kernal_size_3_or_5_or_7 == 3)
        kernal = kernal3_3;
    else if (kernal_size_3_or_5_or_7 == 5)
        kernal = kernal5_5;
    else
        kernal = kernal7_7;

    int mid_size = kernal_size_3_or_5_or_7 / 2;
    if (xmin >= 0) {
        xmin = xmin > mid_size ? xmin : mid_size;
        ymin = ymin > mid_size ? ymin : mid_size;
        xmax = xmax < width - mid_size - 1 ? xmax : width - mid_size - 1;
        ymax = ymax < height - mid_size - 1 ? ymax : height - mid_size - 1;
    } else {
        xmin = ymin = mid_size;
        xmax        = width - mid_size - 1;
        ymax        = height - mid_size - 1;
    }

    for (int i = ymin; i <= ymax; i++) {
        int temp0 = i * width * channel_num;
        for (int j = xmin; j <= xmax; j++) {
            int idx        = temp0 + j * channel_num;
            bool not_break = true;
            for (int m = -mid_size; m <= mid_size && not_break; m++) {
                int temp1 = (m + mid_size) * kernal_size_3_or_5_or_7 + mid_size;
                int temp2 = (i + m) * width + j;
                for (int n = -mid_size; n <= mid_size && not_break; n++) {
                    if (kernal[temp1 + n] && ((data_copy[temp2 + n] == 0) ^ dilate_or_erode)) {
                        *(data + idx) = dilate_or_erode ? 255 : 0;
                        not_break     = false;
                        break;
                    }
                }
            }
        }
    }
    delete[] data_copy;
}

void CCommonAlg::RotateArbitraryAxis3D(float* vect, float* axis, float theta) {
    // 构造向量
    float vect_4e[4];
    for (int i = 0; i < 3; i++) vect_4e[i] = vect[i];
    vect_4e[3] = 1;

    // 归一化旋转轴向量
    float norm = sqrt(pow(axis[0], 2) + pow(axis[1], 2) + pow(axis[2], 2));
    if (norm <= 0)
        return;

    float u = axis[0] / norm;
    float v = axis[1] / norm;
    float w = axis[2] / norm;

    // 计算旋转矩阵
    float p_out[4][4];
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    p_out[0][0]      = cos_theta + (u * u) * (1 - cos_theta);
    p_out[0][1]      = u * v * (1 - cos_theta) + w * sin_theta;
    p_out[0][2]      = u * w * (1 - cos_theta) - v * sin_theta;
    p_out[0][3]      = 0;

    p_out[1][0] = u * v * (1 - cos_theta) - w * sin_theta;
    p_out[1][1] = cos_theta + v * v * (1 - cos_theta);
    p_out[1][2] = w * v * (1 - cos_theta) + u * sin_theta;
    p_out[1][3] = 0;

    p_out[2][0] = u * w * (1 - cos_theta) + v * sin_theta;
    p_out[2][1] = v * w * (1 - cos_theta) - u * sin_theta;
    p_out[2][2] = cos_theta + w * w * (1 - cos_theta);
    p_out[2][3] = 0;

    p_out[3][0] = 0;
    p_out[3][1] = 0;
    p_out[3][2] = 0;
    p_out[3][3] = 1;

    // 执行矩阵乘法
    float tmp[4];
    for (int r = 0; r < 4; r++) {
        tmp[r] = 0;
        for (int c = 0; c < 4; c++) {
            tmp[r] += p_out[c][r] * vect_4e[c];
        }
    }
    for (int i = 0; i < 3; i++) vect[i] = tmp[i] / tmp[3];
}

void CCommonAlg::drawArc(unsigned char* data, int width, int height, unsigned char value, int centre_x, int centre_y,
                         float radius, float start_yaw, float deta_yaw, int channel_num) {
    int x = 0, y = 0;
    float yaw_step = 0.5f / radius;
    // 暂时设为5.0参数，待调试
    for (float yaw = 0; yaw < deta_yaw; yaw += yaw_step) {
        x                                   = static_cast<int>(radius * cos(start_yaw + yaw));
        y                                   = static_cast<int>(radius * sin(start_yaw + yaw));
        x                                   = x + centre_x;
        y                                   = y + centre_y;
        x                                   = x > width - 1 ? width - 1 : x;
        x                                   = x < 0 ? 0 : x;
        y                                   = y > height - 1 ? height - 1 : y;
        y                                   = y < 0 ? 0 : y;
        data[(y * width + x) * channel_num] = value;
    }
}

bool CCommonAlg::allWhite(const unsigned char* data, int width, int height, int centerX, int centerY, int half_width,
                          int channel_num) {
    for (int x = centerX - half_width; x <= centerX + half_width; x++) {
        for (int y = centerY - half_width; y <= centerY + half_width; y++) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                if (data[(y * width + x) * channel_num] < 255) {
                    return false;
                }
            } else {
                return false;
            }
        }
    }

    return true;
}
bool CCommonAlg::FindNearestWhitePt(const unsigned char* data, int width, int height, int start_x, int start_y,
                                    int offset, int& target_x, int& target_y, int channel_num) {
    int ext_r = 1;
    int ext_x = 0, ext_y = 0;

    int loop_times = width > height ? width : height;
    loop_times     = loop_times;
    for (; loop_times > 0; loop_times--) {
        ext_x = start_x + ext_r;
        for (ext_y = start_y - ext_r; ext_y <= start_y + ext_r; ext_y++) {
            if (allWhite(data, width, height, ext_x, ext_y, offset, channel_num)) {
                target_x = ext_x;
                target_y = ext_y;
                return true;
            }
        }

        ext_x = start_x - ext_r;
        for (ext_y = start_y - ext_r; ext_y <= start_y + ext_r; ext_y++) {
            if (allWhite(data, width, height, ext_x, ext_y, offset, channel_num)) {
                target_x = ext_x;
                target_y = ext_y;
                return true;
            }
        }

        ext_y = start_y + ext_r;
        for (ext_x = start_x - ext_r; ext_x <= start_x + ext_r; ext_x++) {
            if (allWhite(data, width, height, ext_x, ext_y, offset, channel_num)) {
                target_x = ext_x;
                target_y = ext_y;
                return true;
            }
        }

        ext_y = start_y - ext_r;
        for (ext_x = start_x - ext_r; ext_x <= start_x + ext_r; ext_x++) {
            if (allWhite(data, width, height, ext_x, ext_y, offset, channel_num)) {
                target_x = ext_x;
                target_y = ext_y;
                return true;
            }
        }

        ext_r++;
    }

    return false;
}

void CCommonAlg::drawLineByDeltaVal(unsigned char* data, unsigned char* ignore_mask, int width, int height, int x1,
                                    int y1, int x2, int y2, int delta_val, int line_radius, int channel_num) {
    if (data) {
        {
            double theta  = calcVectAngel(x2 - x1, y2 - y1);
            double deta_x = cos(theta);
            double deta_y = sin(theta);
            bool x_or_y   = true;
            if (fabs(deta_x) > fabs(deta_y)) {
                x_or_y     = true;
                double tmp = deta_x;
                deta_x     = deta_x > 0 ? 1.0 : -1.0;
                deta_y     = deta_y / tmp * deta_x;
            } else {
                x_or_y     = false;
                double tmp = deta_y;
                deta_y     = deta_y > 0 ? 1.0 : -1.0;
                deta_x     = deta_x / tmp * deta_y;
            }

            double x = x1 + 0.5f, y = y1 + 0.5f;   // 四舍五入。
            if (x_or_y) {
                while ((deta_x > 0 && x <= x2) || (deta_x < 0 && x >= x2)) {
                    drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x), static_cast<int>(y),
                                        delta_val, channel_num);
                    if (line_radius > 0) {
                        for (int i = 1; i <= line_radius; i++) {
                            drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x),
                                                static_cast<int>(y) + i, delta_val, channel_num);
                            drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x),
                                                static_cast<int>(y) - i, delta_val, channel_num);
                        }
                    }
                    x += deta_x;
                    y += deta_y;
                }
            } else {
                while ((deta_y > 0 && y <= y2) || (deta_y < 0 && y >= y2)) {
                    drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x), static_cast<int>(y),
                                        delta_val, channel_num);
                    if (line_radius > 0) {
                        for (int i = 1; i <= line_radius; i++) {
                            drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x) + i,
                                                static_cast<int>(y), delta_val, channel_num);
                            drawPointByDeltaVal(data, ignore_mask, width, height, static_cast<int>(x) - i,
                                                static_cast<int>(y), delta_val, channel_num);
                        }
                    }
                    x += deta_x;
                    y += deta_y;
                }
            }
        }
    }
}

void CCommonAlg::getLine(std::set<std::pair<int, int>>& line_out, int width, int height, int x1, int y1, int x2, int y2,
                         int line_radius) {
    line_out.clear();

    {
        double theta  = calcVectAngel(x2 - x1, y2 - y1);
        double deta_x = cos(theta);
        double deta_y = sin(theta);
        bool x_or_y   = true;
        if (fabs(deta_x) > fabs(deta_y)) {
            x_or_y     = true;
            double tmp = deta_x;
            deta_x     = deta_x > 0 ? 1.0 : -1.0;
            deta_y     = deta_y / tmp * deta_x;
        } else {
            x_or_y     = false;
            double tmp = deta_y;
            deta_y     = deta_y > 0 ? 1.0 : -1.0;
            deta_x     = deta_x / tmp * deta_y;
        }

        double x = x1, y = y1;
        if (x_or_y) {
            while ((deta_x > 0 && x <= x2) || (deta_x < 0 && x >= x2)) {
                line_out.insert(std::pair<int, int>(static_cast<int>(x), static_cast<int>(y)));
                if (line_radius > 0) {
                    for (int i = 1; i <= line_radius; i++) {
                        line_out.insert(std::pair<int, int>(static_cast<int>(x), static_cast<int>(y) + i));
                        line_out.insert(std::pair<int, int>(static_cast<int>(x), static_cast<int>(y) - i));
                    }
                }
                x += deta_x;
                y += deta_y;
            }
        } else {
            while ((deta_y > 0 && y <= y2) || (deta_y < 0 && y >= y2)) {
                line_out.insert(std::pair<int, int>(static_cast<int>(x), static_cast<int>(y)));
                if (line_radius > 0) {
                    for (int i = 1; i <= line_radius; i++) {
                        line_out.insert(std::pair<int, int>(static_cast<int>(x) + i, static_cast<int>(y)));
                        line_out.insert(std::pair<int, int>(static_cast<int>(x) - i, static_cast<int>(y)));
                    }
                }
                x += deta_x;
                y += deta_y;
            }
        }
    }
}

void CCommonAlg::drawArcByDeltaVal(unsigned char* data, int width, int height, int centre_x, int centre_y, float radius,
                                   float start_yaw, float deta_yaw, int delta_val, int channel_num) {
    int x = 0, y = 0;
    float yaw_step = 0.5f / radius;
    // 暂时设为5.0参数，待调试
    for (float yaw = 0; yaw < deta_yaw; yaw += yaw_step) {
        x = static_cast<int>(radius * cos(start_yaw + yaw));
        y = static_cast<int>(radius * sin(start_yaw + yaw));
        x = x + centre_x;
        y = y + centre_y;

        drawPointByDeltaVal(data, nullptr, width, height, x, y, delta_val, channel_num);
    }
}

void CCommonAlg::drawPointByDeltaVal(unsigned char* data, unsigned char* ignore_mask, int width, int height, int x,
                                     int y, int delta_val, int channel_num) {
    if (data == nullptr) {
        return;
    }
    x                = x > width - 1 ? width - 1 : x;
    x                = x < 0 ? 0 : x;
    y                = y > height - 1 ? height - 1 : y;
    y                = y < 0 ? 0 : y;
    int normal_index = y * width + x;
    int index        = normal_index * channel_num;
    if (ignore_mask) {
        if (ignore_mask[normal_index] != 0) {
            int tmp     = data[index] + delta_val;
            tmp         = tmp > 255 ? 255 : tmp;
            tmp         = tmp < 0 ? 0 : tmp;
            data[index] = tmp;
        }
    } else {
        int tmp     = data[index] + delta_val;
        tmp         = tmp > 255 ? 255 : tmp;
        tmp         = tmp < 0 ? 0 : tmp;
        data[index] = tmp;
    }
}

void CCommonAlg::drawPointByDeltaVal(unsigned char* data, int index, int delta_val, int channel_num) {
    if (data == nullptr) {
        return;
    }
    index *= channel_num;
    int tmp     = data[index] + delta_val;
    tmp         = tmp > 255 ? 255 : tmp;
    tmp         = tmp < 0 ? 0 : tmp;
    data[index] = tmp;
}

void CCommonAlg::drawPointByLineSet(std::set<std::pair<int, int>>& line_in, unsigned char* data,
                                    unsigned char* ignore_mask, int width, int height, int delta_val, int channel_num) {
    if (data == nullptr) {
        return;
    }
    for (std::set<std::pair<int, int>>::iterator it = line_in.begin(); it != line_in.end(); it++) {
        int x = it->first;
        int y = it->second;

        x                = x > width - 1 ? width - 1 : x;
        x                = x < 0 ? 0 : x;
        y                = y > height - 1 ? height - 1 : y;
        y                = y < 0 ? 0 : y;
        int normal_index = y * width + x;
        int index        = normal_index * channel_num;
        if (ignore_mask) {
            if (ignore_mask[normal_index] != 0) {
                int tmp     = data[index] + delta_val;
                tmp         = tmp > 255 ? 255 : tmp;
                tmp         = tmp < 0 ? 0 : tmp;
                data[index] = tmp;
            }
        } else {
            int tmp     = data[index] + delta_val;
            tmp         = tmp > 255 ? 255 : tmp;
            tmp         = tmp < 0 ? 0 : tmp;
            data[index] = tmp;
        }
    }
}

bool CCommonAlg::IsPolygonContainsPoint(vector<pair<int32_t, int32_t>> polygon, pair<int32_t, int32_t> point) {
    int cross_point_cnt = 0;
    for (size_t i = 0; i < polygon.size(); i++) {
        pair<int32_t, int32_t> p1(polygon[i].first, polygon[i].second);
        int idx = (i + 1) % polygon.size();
        pair<int32_t, int32_t> p2(polygon[idx].first, polygon[idx].second);
        if (p1.second == p2.second)
            continue;
        if (point.second < min(p1.second, p2.second))
            continue;
        if (point.second >= max(p1.second, p2.second))
            continue;
        float x = (point.second - p1.second) * (p2.first - p1.first) / (p2.second - p1.second) + p1.first;
        if (x > point.first)
            cross_point_cnt++;
    }
    // 单边交点为偶数，点在多边形之外 ---
    return (cross_point_cnt % 2 == 1);
}

}   // namespace LDCV

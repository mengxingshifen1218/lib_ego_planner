/**
 * @file common_alg.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-09-26
 *
 * @copyright Copyright (c) 2021 深圳乐动机器人版权所有
 *
 */
#ifndef LDCV_COMMON_ALG_H_
#define LDCV_COMMON_ALG_H_

#include <inttypes.h>

#include <set>
#include <vector>

namespace LDCV {

class CCommonAlg {
 public:
    CCommonAlg();
    ~CCommonAlg();

    /**
     * @brief 输入向量终点坐标，输出[0,2pi)
     *
     * @param x x坐标
     * @param y y坐标
     * @return double 角度
     */
    static double calcVectAngel(double x, double y);

    /**
     * @brief 将向量（x,y）旋转deta_angel
     *
     * @param x x坐标
     * @param y y坐标
     * @param deta_angel 旋转角度
     */
    static void rotateVect(double& x, double& y, double deta_angel);

    /**
     * @brief 将向量（x,y）旋转deta_angel
     *
     * @param x x坐标
     * @param y y坐标
     * @param deta_angel 旋转角度
     */
    static void rotateVect(float& x, float& y, float deta_angel);

    /**
     * @brief 输入两个取值范围[0,2pi)的角度值，输出两个矢量的夹角的绝对值
     *
     * @param angel1 角度1
     * @param angel2 角度2
     * @return double 夹角绝对值
     */
    static double calcAngelDiffAbs(double angel1, double angel2);

    /**
     * @brief 输入两个取值范围[0,2pi)的角度值，输出两个矢量的夹角
     *
     * @param angel1 角度1
     * @param angel2 角度2
     * @return double 夹角
     */
    static double calcAngelDiff(double angel1, double angel2);

    /**
     * @brief 画直线函数
     *
     * @param value 直线代表的值
     * @param x1 端点1的x坐标
     * @param y1 端点1的y坐标
     * @param x2 端点2的x坐标
     * @param y2 端点2的y坐标
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param domain_4 是否画虚线，暂未启用
     * @param channel_num 图像通道数
     */
    static void lineMat(unsigned char value, int x1, int y1, int x2, int y2, unsigned char* data, int width, int height,
                        bool domain_4 = true, int channel_num = 1);

    /**
     * @brief 检查线段是否与图片中的对应像素值碰撞
     *
     * @param hit_x 从起点开始第一个碰撞点的x坐标
     * @param hit_y 从起点开始第一个碰撞点的y坐标
     * @param lower_equal_value 障碍物阈值，小于等于阈值为障碍物
     * @param x1 线段起点x坐标
     * @param y1 线段起点y坐标
     * @param x2 线段终点x坐标
     * @param y2 线段终点y坐标
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param channel_num 图像通道数
     * @return true 检查到碰撞
     * @return false 没有碰撞
     */
    static bool LineHit(int& hit_x, int& hit_y, unsigned char lower_equal_value, int x1, int y1, int x2, int y2,
                        unsigned char* data, int width, int height, int channel_num = 1);

    /**
     * @brief 以center为中心 正负draw_half_width 的范围中 像素点的值 如果不等于value,则改为value 并返回操作的个数
     *
     * @param value 涂抹值
     * @param centre_x 中心点x坐标
     * @param centre_y 中心点y坐标
     * @param draw_half_width 涂抹宽度的一半
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param minx x坐标最小值限制
     * @param maxx x坐标最大值限制
     * @param miny y坐标最小值限制
     * @param maxy y坐标最大值限制
     * @param channel_num 图像通道数
     * @return int 涂抹像素统计值
     */
    static int DrawCleanedAndSum(unsigned char value, int centre_x, int centre_y, int draw_half_width,
                                 unsigned char* data, int width, int height, int minx, int maxx, int miny, int maxy,
                                 int channel_num = 1);

    /**
     * @brief 画实心圆
     *
     * @param value 涂抹值
     * @param centre_x 圆心点x坐标
     * @param centre_y 圆心点y坐标
     * @param radius 涂抹半径
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param minx x坐标最小值限制
     * @param maxx x坐标最大值限制
     * @param miny y坐标最小值限制
     * @param maxy y坐标最大值限制
     * @param channel_num 图像通道数
     */
    static void circleMat(unsigned char value, int centre_x, int centre_y, float radius, unsigned char* data, int width,
                          int height, int minx, int maxx, int miny, int maxy, int channel_num = 1);

    /**
     * @brief 画矩形
     *
     * @param value 涂抹值
     * @param x 矩形角点x坐标
     * @param y 矩形角点y坐标
     * @param w 矩形宽度
     * @param h 矩形高度
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param minx x坐标最小值限制
     * @param maxx x坐标最大值限制
     * @param miny y坐标最小值限制
     * @param maxy y坐标最大值限制
     * @param channel_num 图像通道数
     */
    static void DrawRectangle(unsigned char value, int x, int y, int w, int h, unsigned char* data, int width,
                              int height, int minx, int maxx, int miny, int maxy, int channel_num = 1);

    /**
     * @brief 画边框，仅图片边缘一个像素
     *
     * @param data 图片数据首地址
     * @param is_vertical_line 是否画垂直边界
     * @param is_horison_line 是都话水平边界
     * @param width 图片宽度
     * @param height 图片高度
     * @param value 涂抹值
     * @param minx x坐标最小值限制
     * @param maxx x坐标最大值限制
     * @param miny y坐标最小值限制
     * @param maxy y坐标最大值限制
     * @param channel_num 图像通道数
     */
    static void edge_set(unsigned char* data, bool is_vertical_line, bool is_horison_line, int width, int height,
                         int value, int xmin, int xmax, int ymin, int ymax, int channel_num = 1);


    static const bool kernal3_3[9];
    static const bool kernal5_5[25];
    static const bool kernal7_7[49];
    /****
     * @brief 膨胀、腐蚀，膨胀：白色往黑色扩展，腐蚀，黑色往白色扩展
     * 
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param dilate_or_erode 膨胀：true 腐蚀：false
     * @param kernal_size_3_or_5_or_7 掩膜大小
     * @param minx x坐标最小值限制
     * @param maxx x坐标最大值限制
     * @param miny y坐标最小值限制
     * @param maxy y坐标最大值限制
     * @param channel_num 图像通道数
     */
    static void dilate_erode(unsigned char* data, int width, int height, bool dilate_or_erode,
                             int kernal_size_3_or_5_or_7, int xmin, int xmax, int ymin, int ymax, int channel_num = 1);

    /**
     * @brief 计算绕任意过原点转轴旋转向量的旋转矩阵
     *
     * @param vect 向量
     * @param axis 旋转轴
     * @param theta 旋转角度
     */
    static void RotateArbitraryAxis3D(float* vect, float* axis, float theta);

    /**
     * @brief 画圆弧
     * 
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param value 涂抹值
     * @param centre_x 圆心x坐标
     * @param centre_y 圆心y坐标
     * @param radius 圆弧半径
     * @param startYaw 起始角度
     * @param detaYaw 结束角度
     * @param channel_num 图像通道数
     */
    static void drawArc(unsigned char* data, int width, int height, unsigned char value, int centre_x, int centre_y,
                        float radius, float startYaw, float detaYaw, int channel_num = 1);

    /**
     * @brief 检查一个点为中心固定距离的正方形范围内是否所有像素均为白色
     * 
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param centerX 中心点x坐标
     * @param centerY 中心点y坐标
     * @param half_width 正方形边长的一般
     * @param channel_num 图片通道数
     * @return true 全白
     * @return false 非全白
     */
    static bool allWhite(const unsigned char* data, int width, int height, int centerX, int centerY, int half_width,
                         int channel_num = 1);

    /**
     * @brief 查找点周围最近满足要求的白色点
     * 
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param start_x 起点x坐标
     * @param start_y 起点y坐标
     * @param offset 满足要求的白色点大小
     * @param target_x 目标点x坐标
     * @param target_y 目标点y坐标
     * @param channel_num 图片通道数
     * @return true 
     * @return false 
     */
    static bool FindNearestWhitePt(const unsigned char* data, int width, int height, int start_x, int start_y,
                                   int offset, int& target_x, int& target_y, int channel_num = 1);

    /**
     * @brief 画直线，在原图片像素值上加上固定值
     * 
     * @param data 图片数据首地址
     * @param ignore_mask 忽略区域掩膜
     * @param width 图片宽度
     * @param height 图片高度
     * @param x1 端点1的x坐标
     * @param y1 端点1的y坐标
     * @param x2 端点2的x坐标
     * @param y2 端点2的y坐标
     * @param delta_val 像素增加值
     * @param line_radius 线宽半径
     * @param channel_num 图片通道数
     */
    static void drawLineByDeltaVal(unsigned char* data, unsigned char* ignore_mask, int width, int height, int x1,
                                   int y1, int x2, int y2, int delta_val, int line_radius, int channel_num);

    /**
     * @brief 获取线段上所有像素点坐标集合
     * 
     * @param line_out 输出直线像素点坐标集合
     * @param width 图片宽度
     * @param height 图片高度
     * @param x1 端点1的x坐标
     * @param y1 端点1的y坐标
     * @param x2 端点2的x坐标
     * @param y2 端点2的y坐标
     * @param line_radius 线宽半径
     */
    static void getLine(std::set<std::pair<int, int>>& line_out, int width, int height, int x1, int y1, int x2, int y2,
                        int line_radius);

    /**
     * @brief 画圆弧，在原图片像素值上加上固定值
     * 
     * @param data 图片数据首地址
     * @param width 图片宽度
     * @param height 图片高度
     * @param centerX 中心点x坐标
     * @param centerY 中心点y坐标
     * @param radius 圆弧半径
     * @param startYaw 起始角度
     * @param detaYaw 结束角度
     * @param delta_val 像素增加值
     * @param channel_num 图片通道数
     */
    static void drawArcByDeltaVal(unsigned char* data, int width, int height, int centre_x, int centre_y, float radius,
                                  float startYaw, float detaYaw, int delta_val, int channel_num = 1);

    /**
     * @brief 画点，在原图片像素值上加上固定值
     * 
     * @param data 图片数据首地址
     * @param ignore_mask 忽略区域掩膜
     * @param width 图片宽度
     * @param height 图片高度
     * @param x 点坐标x
     * @param y 点坐标y
     * @param delta_val 像素增加值
     * @param channel_num 图片通道数
     */
    static void drawPointByDeltaVal(unsigned char* data, unsigned char* ignore_mask, int width, int height, int x,
                                    int y, int delta_val, int channel_num = 1);

    /**
     * @brief 画点，在原图片像素值上加上固定值
     * 
     * @param data 图片数据首地址
     * @param index 点的索引
     * @param delta_val 像素增加值
     * @param channel_num 图片通道数
     */
    static void drawPointByDeltaVal(unsigned char* data, int index, int delta_val, int channel_num = 1);

    /**
     * @brief 画输入线段中的所有点
     * 
     * @param line_in 输入线段点集
     * @param data 图片数据首地址
     * @param ignore_mask 忽略区域掩膜
     * @param width 图片宽度
     * @param height 图片高度
     * @param delta_val 像素增加值
     * @param channel_num 图片通道数
     */
    static void drawPointByLineSet(std::set<std::pair<int, int>>& line_in, unsigned char* data,
                                   unsigned char* ignore_mask, int width, int height, int delta_val,
                                   int channel_num = 1);

    /**
     * @brief 判断点是否在多边形区域内
     * 
     * @param polygon 多边形顶点集合
     * @param point 目标点
     * @return true 在多边形内部
     * @return false 在多边形外部
     */
    static bool IsPolygonContainsPoint(std::vector<std::pair<int32_t, int32_t>> polygon,
                                       std::pair<int32_t, int32_t> point);
};
}   // namespace LDCV

#endif   // LDCV_COMMON_ALG_H_

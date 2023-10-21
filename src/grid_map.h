#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <chrono>
#include <iostream>
#include <queue>
#include <random>
#include <tuple>

#include <opencv2/opencv.hpp>

#include "maps.h"

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false

constexpr int DIME_SIZE = 2;
using namespace std;

namespace ego_planner
{
    using Time = std::chrono::time_point<std::chrono::steady_clock>;
    // 返回的是ns
    static Time Now()
    {
        return std::chrono::steady_clock::now();
    }
    // 返回的是以s为单位的当前时刻
    static double toSec(Time now)
    {
        return now.time_since_epoch().count() / 1e9;
    }
}
// constant parameters
class GridMap
{
public:
    GridMap() {}
    ~GridMap() {}

    void initMap(TMapData &map);
    inline int getOccupancy(Eigen::Vector2d pos);
    inline int getInflateOccupancy(Eigen::Vector2d pos);
    inline double getResolution();

    typedef std::shared_ptr<GridMap> Ptr;

    TMapData map_;
    float resolution_ = 0.05;
};

inline int GridMap::getOccupancy(Eigen::Vector2d pos)
{
    int x     = map_.mapParam.x2idx(pos[0]);
    int y     = map_.mapParam.y2idx(pos[1]);
    int index = y * map_.mapParam.width + x;

    return map_.map[index] <= 127;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector2d pos)
{
    int x     = map_.mapParam.x2idx(pos[0]);
    int y     = map_.mapParam.y2idx(pos[1]);
    int index = y * map_.mapParam.width + x;

    return map_.map[index] <= 127 ? GRID_MAP_OBS_FLAG : 0;
}

inline double GridMap::getResolution() { return resolution_; }

#endif
#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <iostream>
#include <random>
#include <queue>
#include <tuple>
#include <chrono>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>

#define logit(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false

constexpr int DIME_SIZE = 2;
using namespace std;

using Time = std::chrono::time_point<std::chrono::steady_clock>;
// 返回的是ns
static Time Now() 
{
  return std::chrono::steady_clock::now();
}

// 返回的是以s为单位的当前时刻
static double toSec(Time now){
  return now.time_since_epoch().count() / 1e9;
}
// constant parameters

struct MappingParameters
{
  bool have_initialized_ = false;

  /* map properties */
  Eigen::Vector2d local_update_range3d_;
  Eigen::Vector2i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;
  string frame_id_;
  int pose_type_;
  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  bool use_depth_filter_;
  double depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                           // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit of occupancy probability
  double min_ray_length_;                                                                   // range of doing raycasting
  double fading_time_;

  /* visualization and computation time display */
  bool show_occ_time_;
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector2i center_last3i_;
  Eigen::Vector2i ringbuffer_origin3i_;
  Eigen::Vector2d ringbuffer_lowbound3d_;
  Eigen::Vector2i ringbuffer_lowbound3i_;
  Eigen::Vector2d ringbuffer_upbound3d_;
  Eigen::Vector2i ringbuffer_upbound3i_;
  // Eigen::Vector2d ringbuffer_size3d_;
  Eigen::Vector2i ringbuffer_size3i_;
  Eigen::Vector2i ringbuffer_inf_origin3i_;
  Eigen::Vector2d ringbuffer_inf_lowbound3d_;
  Eigen::Vector2i ringbuffer_inf_lowbound3i_;
  Eigen::Vector2d ringbuffer_inf_upbound3d_;
  Eigen::Vector2i ringbuffer_inf_upbound3i_;
  Eigen::Vector2i ringbuffer_inf_size3i_;

  // main map data, occupancy of each voxel

  std::vector<double> occupancy_buffer_;
  std::vector<uint16_t> occupancy_buffer_inflate_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_;

  // odom_depth_timeout_
  Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_have_ever_received_depth_;

  // depth image projected point cloud

  vector<Eigen::Vector2d> proj_points_;
  int proj_points_cnt_;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;

  vector<Eigen::Vector2i> cache_voxel_;
  int cache_voxel_cnt_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  void initMap();
  inline int getOccupancy(Eigen::Vector2d pos);
  inline int getInflateOccupancy(Eigen::Vector2d pos);
  inline double getResolution();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  inline Eigen::Vector2d globalIdx2Pos(const Eigen::Vector2i &id);  // 1.69ns
  inline Eigen::Vector2i pos2GlobalIdx(const Eigen::Vector2d &pos); // 0.13ns
  inline int globalIdx2BufIdx(const Eigen::Vector2i &id);           // 2.2ns
  inline int globalIdx2InfBufIdx(const Eigen::Vector2i &id);        // 2.2ns
  inline Eigen::Vector2i BufIdx2GlobalIdx(size_t address);          // 10.18ns
  inline Eigen::Vector2i infBufIdx2GlobalIdx(size_t address);       // 10.18ns
  inline bool isInBuf(const Eigen::Vector2d &pos);
  inline bool isInBuf(const Eigen::Vector2i &idx);
  inline bool isInInfBuf(const Eigen::Vector2d &pos);
  inline bool isInInfBuf(const Eigen::Vector2i &idx);


  void clearBuffer(char casein, int bound);

  // main update process
  void moveRingBuffer();
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();

  inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector2i global_idx);
  inline int setCacheOccupancy(Eigen::Vector2d pos, int occ);
  Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &camera_pt);
  void testIndexingCost();
  bool checkDepthOdomNeedupdate();
  void initMapBoundary();


  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::setCacheOccupancy(Eigen::Vector2d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector2i id = pos2GlobalIdx(pos);
  int idx_ctns = globalIdx2BufIdx(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector2i global_idx)
{
  int inf_grid = mp_.inf_grid_;
  if (dir)
    md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  else
    md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      {
        Eigen::Vector2i id_inf(global_idx + Eigen::Vector2i(x_inf, y_inf));
        int id_inf_buf = globalIdx2InfBufIdx(id_inf);
        if (dir)
          ++md_.occupancy_buffer_inflate_[id_inf_buf];
        else
        {
          --md_.occupancy_buffer_inflate_[id_inf_buf];
          if (md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // An error case
          {
            printf("A negtive value of nearby obstacle number! reset the map.\n");
            fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_);
            fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0L);
          }
        }
      }
}

inline int GridMap::globalIdx2BufIdx(const Eigen::Vector2i &id)
{
  int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_origin3i_(1)) % md_.ringbuffer_size3i_(1);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_size3i_(1);


  return md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

inline int GridMap::globalIdx2InfBufIdx(const Eigen::Vector2i &id)
{
  int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
  int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(1)) % md_.ringbuffer_inf_size3i_(1);
  if (x_buffer < 0)
    x_buffer += md_.ringbuffer_inf_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.ringbuffer_inf_size3i_(1);


  return md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector2i GridMap::BufIdx2GlobalIdx(size_t address)
{

  const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
  if (xid_global > md_.ringbuffer_upbound3i_(0))
    xid_global -= md_.ringbuffer_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
  if (yid_global > md_.ringbuffer_upbound3i_(1))
    yid_global -= md_.ringbuffer_size3i_(1);

  return Eigen::Vector2i(xid_global, yid_global);
}

inline Eigen::Vector2i GridMap::infBufIdx2GlobalIdx(size_t address)
{

  const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
  int zid_in_buffer = address / ringbuffer_xysize;
  address %= ringbuffer_xysize;
  int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
  int xid_in_buffer = address % md_.ringbuffer_inf_size3i_(0);

  int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
  if (xid_global > md_.ringbuffer_inf_upbound3i_(0))
    xid_global -= md_.ringbuffer_inf_size3i_(0);
  int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
  if (yid_global > md_.ringbuffer_inf_upbound3i_(1))
    yid_global -= md_.ringbuffer_inf_size3i_(1);

  return Eigen::Vector2i(xid_global, yid_global);
}

inline int GridMap::getOccupancy(Eigen::Vector2d pos)
{
  if (!isInBuf(pos))
    return 0;


  return md_.occupancy_buffer_[globalIdx2BufIdx(pos2GlobalIdx(pos))] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector2d pos)
{
  if (!isInInfBuf(pos))
    return 0;

  return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline bool GridMap::isInBuf(const Eigen::Vector2d &pos)
{
  if (pos(0) < md_.ringbuffer_lowbound3d_(0) || pos(1) < md_.ringbuffer_lowbound3d_(1) )
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_upbound3d_(0) || pos(1) > md_.ringbuffer_upbound3d_(1) )
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInBuf(const Eigen::Vector2i &idx)
{
  if (idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) )
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) )
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector2d &pos)
{
  if (pos(0) < md_.ringbuffer_inf_lowbound3d_(0) || pos(1) < md_.ringbuffer_inf_lowbound3d_(1) )
  {
    return false;
  }
  if (pos(0) > md_.ringbuffer_inf_upbound3d_(0) || pos(1) > md_.ringbuffer_inf_upbound3d_(1))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector2i &idx)
{
  if (idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) )
  {
    return false;
  }
  if (idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1))
  {
    return false;
  }
  return true;
}

inline Eigen::Vector2d GridMap::globalIdx2Pos(const Eigen::Vector2i &id) // t ~ 0us
{
  return Eigen::Vector2d((id(0) + 0.5) * mp_.resolution_, (id(1) + 0.5) * mp_.resolution_);
}

inline Eigen::Vector2i GridMap::pos2GlobalIdx(const Eigen::Vector2d &pos)
{
  return (pos * mp_.resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
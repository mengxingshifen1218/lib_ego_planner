#include "grid_map.h"

void GridMap::initMap(cv::Mat& map)
{

  // map_ = map.clone();
  cv::cvtColor(map, map_, cv::COLOR_BGR2GRAY); 
  /* get parameter */
  imwrite("map.pgm", map_);
  cout<<"write map"<<endl;
  mp_.resolution_ = 0.1;
  mp_.local_update_range2d_(0) = 5.5;
  mp_.local_update_range2d_(1) = 5.5;
  mp_.obstacles_inflation_ = 0.1;
  mp_.enable_virtual_walll_ = true;
  mp_.virtual_ceil_ = 3.0;
  mp_.virtual_ground_ = 0.1;

  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)
  {
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    printf("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range2i_ = (mp_.local_update_range2d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range2d_ = mp_.local_update_range2i_.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_size2i_ = 2 * mp_.local_update_range2i_;
  md_.ringbuffer_inf_size2i_ = md_.ringbuffer_size2i_ + Eigen::Vector2i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_);

  // initialize data buffers
  Eigen::Vector2i map_voxel_num2i = 2 * mp_.local_update_range2i_;
  int buffer_size = map_voxel_num2i(0) * map_voxel_num2i(1);
  int buffer_inf_size = (map_voxel_num2i(0) + 2 * mp_.inf_grid_) * (map_voxel_num2i(1) + 2 * mp_.inf_grid_);
  md_.ringbuffer_origin2i_ = Eigen::Vector2i(0, 0);
  md_.ringbuffer_inf_origin2i_ = Eigen::Vector2i(0, 0);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_);
  md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ = vector<Eigen::Vector2i>(buffer_size, Eigen::Vector2i(0, 0));

  md_.raycast_num_ = 0;
  md_.proj_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;

  md_.occ_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  // md_.last_occ_update_time_.fromSec(0);

  md_.flag_have_ever_received_depth_ = false;
  md_.flag_depth_odom_timeout_ = false;

  initMapBoundary();
}

void GridMap::clearAndInflateLocalMap()
{
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {
    Eigen::Vector2i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    if (md_.occupancy_buffer_inflate_[inf_buf_id] < GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] >= mp_.min_occupancy_log_)
    {
      changeInfBuf(true, inf_buf_id, idx);
    }

    if (md_.occupancy_buffer_inflate_[inf_buf_id] >= GRID_MAP_OBS_FLAG && md_.occupancy_buffer_[buf_id] < mp_.min_occupancy_log_)
    {
      changeInfBuf(false, inf_buf_id, idx);
    }
  }
}

void GridMap::initMapBoundary()
{
  mp_.have_initialized_ = true;

  mp_.have_initialized_ = true;

  md_.center_last2i_ = Eigen::Vector2i(0, 0);

  md_.ringbuffer_lowbound2i_ = md_.center_last2i_ - mp_.local_update_range2i_;
  md_.ringbuffer_lowbound2d_ = md_.ringbuffer_lowbound2i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound2i_ = md_.center_last2i_ + mp_.local_update_range2i_;
  md_.ringbuffer_upbound2d_ = md_.ringbuffer_upbound2i_.cast<double>() * mp_.resolution_;
  md_.ringbuffer_upbound2i_ -= Eigen::Vector2i(1, 1);

  // cout<<"md_.center_last2i_  :"<<md_.center_last2i_.transpose() <<" mp_.local_update_range2i_: "<< mp_.local_update_range2i_.transpose()<<endl;
  const Eigen::Vector2i inf_grid2i(mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector2d inf_grid2d = inf_grid2i.array().cast<double>() * mp_.resolution_;
  md_.ringbuffer_inf_lowbound2i_ = md_.ringbuffer_lowbound2i_ - inf_grid2i;
  md_.ringbuffer_inf_lowbound2d_ = md_.ringbuffer_lowbound2d_ - inf_grid2d;
  md_.ringbuffer_inf_upbound2i_ = md_.ringbuffer_upbound2i_ + inf_grid2i;
  md_.ringbuffer_inf_upbound2d_ = md_.ringbuffer_upbound2d_ + inf_grid2d;

  // cout << "md_.ringbuffer_lowbound2i_=" << md_.ringbuffer_lowbound2i_.transpose() << " md_.ringbuffer_lowbound2d_=" << md_.ringbuffer_lowbound2d_.transpose() << endl
  //      << "md_.ringbuffer_upbound2i_ =" << md_.ringbuffer_upbound2i_.transpose()  << " md_.ringbuffer_upbound2d_ =" << md_.ringbuffer_upbound2d_.transpose() << endl;

  // cout << "md_.ringbuffer_inf_lowbound2i_=" << md_.ringbuffer_inf_lowbound2i_.transpose() << " md_.ringbuffer_inf_lowbound2d_=" << md_.ringbuffer_inf_lowbound2d_.transpose() << endl
  //      << "md_.ringbuffer_inf_upbound2i_ =" << md_.ringbuffer_inf_upbound2i_.transpose()  << " md_.ringbuffer_inf_upbound2d_ =" << md_.ringbuffer_inf_upbound2d_.transpose() << endl;

  for (int i = 0; i < DIME_SIZE; ++i)
  {
    while (md_.ringbuffer_origin2i_(i) < md_.ringbuffer_lowbound2i_(i))
    {
      md_.ringbuffer_origin2i_(i) += md_.ringbuffer_size2i_(i);
    }
    while (md_.ringbuffer_origin2i_(i) > md_.ringbuffer_upbound2i_(i))
    {
      md_.ringbuffer_origin2i_(i) -= md_.ringbuffer_size2i_(i);
    }

    while (md_.ringbuffer_inf_origin2i_(i) < md_.ringbuffer_inf_lowbound2i_(i))
    {
      md_.ringbuffer_inf_origin2i_(i) += md_.ringbuffer_inf_size2i_(i);
    }
    while (md_.ringbuffer_inf_origin2i_(i) > md_.ringbuffer_inf_upbound2i_(i))
    {
      md_.ringbuffer_inf_origin2i_(i) -= md_.ringbuffer_inf_size2i_(i);
    }
  }

#if GRID_MAP_NEW_PLATFORM_TEST
  testIndexingCost();
#endif
}

void GridMap::clearBuffer(char casein, int bound)
{
  for (int x = (casein == 0 ? bound : md_.ringbuffer_lowbound2i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound2i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.ringbuffer_lowbound2i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound2i_(1)); ++y)
    {
      Eigen::Vector2i id_global(x, y);
      int id_buf = globalIdx2BufIdx(id_global);
      int id_buf_inf = globalIdx2InfBufIdx(id_global);
      Eigen::Vector2i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                        (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)));
      // int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);

      // md_.occupancy_buffer_inflate_[id_buf_inf_clr] = 0;
      md_.count_hit_[id_buf] = 0;
      md_.count_hit_and_miss_[id_buf] = 0;
      md_.flag_traverse_[id_buf] = md_.raycast_num_;
      md_.flag_rayend_[id_buf] = md_.raycast_num_;
      md_.occupancy_buffer_[id_buf] = mp_.clamp_min_log_;

      if (md_.occupancy_buffer_inflate_[id_buf_inf] > GRID_MAP_OBS_FLAG)
      {
        changeInfBuf(false, id_buf_inf, id_global);
      }
    }
}

Eigen::Vector2d GridMap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &camera_pt)
{
  Eigen::Vector2d diff = pt - camera_pt;
  Eigen::Vector2d max_tc = md_.ringbuffer_upbound2d_ - camera_pt;
  Eigen::Vector2d min_tc = md_.ringbuffer_lowbound2d_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < DIME_SIZE; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  Time t0 = Now();
  double a = 0;
  int b = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound2i_(0); x <= md_.ringbuffer_upbound2i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound2i_(1); y <= md_.ringbuffer_upbound2i_(1); ++y)
      {
        b += x + y;
      }
  Time t1 = Now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound2i_(0); x <= md_.ringbuffer_upbound2i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound2i_(1); y <= md_.ringbuffer_upbound2i_(1); ++y)
      {
        int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector2i(x, y)); // 8396us = 7970
        b += id_buf_inf_clr;
        b += x + y;
      }
  Time t2 = Now();

  for (int i = 0; i < 10; ++i)
    for (int x = md_.ringbuffer_lowbound2i_(0); x <= md_.ringbuffer_upbound2i_(0); ++x)
      for (int y = md_.ringbuffer_lowbound2i_(1); y <= md_.ringbuffer_upbound2i_(1); ++y)
      {
        Eigen::Vector2d pos = globalIdx2Pos(Eigen::Vector2i(x, y)); // 6553us = 6127
        a += pos.sum();
        b += x + y;
      }
  Time t3 = Now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound2d_(0); xd <= md_.ringbuffer_upbound2d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound2d_(1); yd <= md_.ringbuffer_upbound2d_(1); yd += mp_.resolution_)
      {
        a += xd + yd;
      }
  Time t4 = Now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound2d_(0); xd <= md_.ringbuffer_upbound2d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound2d_(1); yd <= md_.ringbuffer_upbound2d_(1); yd += mp_.resolution_)
      {
        Eigen::Vector2i idx = pos2GlobalIdx(Eigen::Vector2d(xd, yd)); // 7088us = 478us
        a += xd + yd;
        b += idx.sum();
      }
  Time t5 = Now();

  for (int i = 0; i < 10; ++i)
    for (double xd = md_.ringbuffer_lowbound2d_(0); xd <= md_.ringbuffer_upbound2d_(0); xd += mp_.resolution_)
      for (double yd = md_.ringbuffer_lowbound2d_(1); yd <= md_.ringbuffer_upbound2d_(1); yd += mp_.resolution_)
      {
        int id_buf_inf_clr = globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector2d(xd, yd)));
        a += xd + yd;
        b += id_buf_inf_clr;
      }
  Time t6 = Now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      b += i;
    }
  Time t7 = Now();

  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occupancy_buffer_.size(); ++i)
    {
      Eigen::Vector2i idx = BufIdx2GlobalIdx(i); // 36939
      b += i;
      b += idx.sum();
    }
  Time t8 = Now();

  double n = md_.occupancy_buffer_.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%f, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f ms\n", n, (t1 - t0).count() / 1e6, (t2 - t1).count() / 1e6,
         (t3 - t2).count() / 1e6, (t4 - t3).count() / 1e6, (t5 - t4).count() / 1e6, (t6 - t5).count() / 1e6, (t7 - t6).count() / 1e6, (t8 - t7).count() / 1e6);
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).count() / n, ((t3 - t2) - (t1 - t0)).count() / n,
         ((t5 - t4) - (t4 - t3)).count() / n, ((t6 - t5) - (t4 - t3)).count() / n, ((t8 - t7) - (t7 - t6)).count() / n);
}
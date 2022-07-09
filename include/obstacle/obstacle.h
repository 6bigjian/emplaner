#pragma once

#include "emplaner/head.h"

class Obstacle
{
public:
  Obstacle(double x, double y)
  {
    x_set = x;
    y_set = y;
  }
  double x_set;
  double y_set;
  double s_set;
  double l_set;
  double length = 2.0;
  double width = 2.0;
  bool obs_flag = false;   //这个障碍物是否靠近参考线
};

class ObsProj
{
public:
  ObsProj();

  void Get_globline();
  void Set_Obstacle();
  void obstacle_trans_frenet();
  double calc_obstacle_cost(const double road_s, const double road_l, const double cost_obs = 1e6, const double cost_neib = 1e4);

  static std::vector<Obstacle> All_obstacle;

private:
  bool simulation_flag;

  

  Dynamic_planning::Road_mags global_reference; //全局路劲  

  const std::string Frame_id = "planning_odom";

  ros::Publisher marker_pub_;
};







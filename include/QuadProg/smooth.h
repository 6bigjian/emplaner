#pragma once

#include"emplaner/head.h"

#define arrayCapacity 40u


#define    QuadProgslover     0   //QuadProg++求解器
#define    qpoasessolver      1     //qpOASES求解器


#define qpsolver  qpoasessolver

class SmoLine
{ 
public:
  SmoLine();
  ~SmoLine();

  void odom_call_back(const nav_msgs::Odometry& odom);
  void GPSToMGRS(Eigen::Vector3d& gps, Eigen::Vector3d& mgrs);
  void CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg);

  void sml_thread_worker();
  void generate_convex_space();
  int16_t FindNearIndex(double s_set);
  void Frenet_trans_Cartesian();
  void Cartesian2Frenet();
  
  bool Calc_Plan_Start_Point();
  void calc_traj_theta();

  void QuadProg_solver();

  /*qpOASES求解器函数*/
  void qpOASES_init();
  void qpOASES_burn();
  void qpOASES_solver();
  void qpOASES_solver(const uint16_t pointsize);

  void qpOASES_SLinit();
  void qpOASES_SLburn();
  void qpOASES_SLsolver();
  void qpOASES_SLsolver(const uint16_t pointsize);

  void qpOASES_XYinit();
  void qpOASES_XYburn();
  void qpOASES_XYsolver();
  void qpOASES_XYsolver(const uint16_t pointsize);
  /******************/
private:
  uint16_t arraysize;
  int16_t plan_startIndex;

  const double borderLimit = 2.0;
  const double dlLimit = 10.0;
  const double ddlLimit = 100;

  const double ds = 0.5;

  const double safety_distance = 0.2;

  const int16_t retaintraj_num = 5;  //轨迹保留点数
  int16_t vehTotraj_projIndex_;       //车辆到二次规划轨迹的投影点

  double vehicleWidth;
  double vehicleLength;

  double w_cost_dl = 10;
  double w_cost_ddl = 60;
  double w_cost_centre = 0.2;

  double w_cost_smooth = 10.0;             //平滑代价
  double w_cost_length = 0.1;             //长度代价
  double w_cost_ref = 1.0;                //相似代价

  double L_limit[arrayCapacity][4];   //s,l_max,l_min,traj_line;

  double save_line[arrayCapacity][4];   //保持路径，在规划犯病的时候用。

  bool simulation_flag;
  bool partLine_build_ = false;
  bool first_sml = true;
  bool retraj = true;     //是否重新规划

  const std::string Frame_id = "planning_odom";

  std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet;
  Dynamic_planning::Road_mags reference_path;   //从全局路劲提取出来的局部参考路劲
  nav_msgs::Path  trajline_SLpoint;
  nav_msgs::Path  trajline_point;                //最终平滑曲线
  std::vector<double> traj_theta;               //轨迹的theta

  geometry_msgs::Pose New_car_pose;
  Dynamic_planning::Frenet_mags plan_start_mags;//frenet坐标系下，规划起点信息

  boost::thread* generate_sml_thread_;

  ros::Subscriber odom_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher trajline_pub_;
};

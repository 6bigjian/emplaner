#pragma once

#include"emplaner/head.h"

#define arrayCapacity 40u


#define    qpoasessolver      0     //qpOASES求解器
#define    qpsolver   qpoasessolver

enum{
  LINEBACK = -2,  //在规划后面
  LINEFRONT = -1  //超出规划范围
};

enum objpoint{
  MINPOINT = 0,
  MIDPOINT,
  MAXPOINT
};

class SmoLine
{ 
public:
  SmoLine();
  ~SmoLine();

  void odom_call_back(const nav_msgs::Odometry& odom);
  void GPSToMGRS(Eigen::Vector3d& gps, Eigen::Vector3d& mgrs);
  void CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg);
  void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void sml_thread_worker();
  void generate_convex_space();
  int16_t FindNearIndex(const double s_set);
  int16_t FindObjIndex(const double s_set, objpoint OBJPOINT);
  void Frenet_trans_Cartesian();
  Dynamic_planning::Frenet_mags Cartesian2Frenet(geometry_msgs::Pose& match_pose);
  
  bool Calc_Plan_Start_Point();
  void calc_traj_theta();

  void prediMovObsTraj();
  void genST();

  /*qpOASES求解器函数*/
  void qpOASES_init();
  void qpOASES_burn();
  void qpOASES_solver();
  void qpOASES_solver(uint16_t& pointsize);

  void qpOASES_SLinit();
  void qpOASES_SLburn();
  void qpOASES_SLsolver();
  void qpOASES_SLsolver(const uint16_t pointsize);

  void qpOASES_XYinit();
  void qpOASES_XYburn();
  void qpOASES_XYsolver();
  void qpOASES_XYsolver(const uint16_t pointsize);

  void qpOASES_STinit();
  void qpOASES_STburn();
  void qpOASES_STsolver();
  void qpOASES_STsolver(const uint16_t pointsize);
  /******************/
private:
  uint16_t arraysize;
  int16_t plan_startIndex;

  double lon_speed = 1.0;
  double max_speed = 2.0;

  const double borderLimit = 2.0;
  const double dlLimit = 10.0;
  const double ddlLimit = 100;

  const double ds = 0.5;//sl以0.5m为间隔取点
  const double dt = 0.5;//st以0.5s为间隔取点

  const double safety_distance = 0.2;

  const int16_t retaintraj_num = 5;  //轨迹保留点数
  int16_t vehTotraj_projIndex_;       //车辆到二次规划轨迹的投影点

  double vehicleWidth;
  double vehicleLength;

  double w_cost_dl = 10;
  double w_cost_ddl = 60;
  double w_cost_centre = 0.2;

  double w_cost_smooth = 50.0;             //平滑代价
  double w_cost_length = 0.1;             //长度代价
  double w_cost_ref = 1.0;                //相似代价

  double w_st_smooth = 10.0;
  double w_st_ref = 1.0;

  int16_t divisionTime_num = (((double)arrayCapacity*ds)/lon_speed + 5);

  double L_limit[arrayCapacity][4];   //s,l_max,l_min,traj_line;

  double save_line[arrayCapacity][4];   //保持路径，在平滑犯病的时候用。

  double move_region[arrayCapacity][2]; //

  double STub[50];        //ST图以T为坐标轴的s上边界，及在这个时间s不能超越这个极限值
  double STlb[50];        //ST图以T为坐标轴的s下边界，及在这个时间s不能小于这个极限值

  double TSub[arrayCapacity];//ST图以s为坐标轴的T上边界，及在这个到达此处的时间T不能超越这个极限值
  double TSlb[arrayCapacity];//ST图以s为坐标轴的T下边界，及在这个到达此处的时间T不能小于这个极限值

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
  double trajline_s[arrayCapacity];       //用来存放最终平滑轨迹的s

  geometry_msgs::Pose New_car_pose; //当前车辆位置
  geometry_msgs::Pose goal_pose;   //终点位置坐标
  Dynamic_planning::Frenet_mags plan_start_mags;//frenet坐标系下，规划起点信息
  Dynamic_planning::Frenet_mags plan_goal_mags;//frenet坐标系下，规划起点信息

  boost::thread* generate_sml_thread_;

  ros::Subscriber odom_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher trajline_pub_;

};

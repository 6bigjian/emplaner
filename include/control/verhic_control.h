#pragma once

#include "emplaner/head.h"


class Ver_Ctrl
{
public:
  Ver_Ctrl();
  ~Ver_Ctrl();

  void start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void odom_call_back(const nav_msgs::Odometry& odom);
  void CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg);
  void trajLineCallBack(const nav_msgs::Path& Path);

  void ctrl_thread_worker();

  void verhicle_control(const nav_msgs::Path& referenceline, const geometry_msgs::Pose& car_pose);


private:
  bool simulation_flag;
  bool ctrl_flag = false;

  int16_t carindex;
  int16_t Ld = 3;//前瞻距离
  double vx = 1;

  nav_msgs::Path traj_line;
    
  geometry_msgs::Pose goal_pose_;   //终点位置坐标
  geometry_msgs::Pose New_car_pose;

  boost::thread* generate_ctrl_thread_;

  ros::Subscriber start_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber traj_line_sub_;

  ros::Publisher control_data_pub_;
  ros::Publisher vehicle_pose_pub_;
};

class PID
{
  public:
    double kp = 0.001;
    double ki = 0.0003;
    double kd = 0;
    double integral=0;
    double output_limit = M_PI/4;
    double count_e(double error);
    void clear_integral();
};


double PIDcontrol(int16_t carindex, const nav_msgs::Path& referenceline, const geometry_msgs::Pose& car_pose);
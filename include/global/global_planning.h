#pragma once

#include "emplaner/head.h"

#include "global/hdmap_building.h"

class Global_Plan
{
public:
  Global_Plan();
  ~Global_Plan();
  void start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);//起点回调函数
  void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg);//终点回调函数
  static void GPSToMGRS(Eigen::Vector3d& gps, Eigen::Vector3d& mgrs);
  void CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg);
  static int16_t Search_Match_Point(const double& search_pose_x, const double& search_pose_y,//寻找匹配点
                             const nav_msgs::Path referenceline,
                             const int16_t start_index, const int16_t end_index);//
  void global_thread_worker();


private:
  Hdmap_Build hdmap_build_;
  bool simulation_flag;

  geometry_msgs::Pose start_pose_;  //起点位置坐标
  geometry_msgs::Pose goal_pose_;   //终点位置坐标

  boost::thread* generate_glo_thread_;


  //消息订阅发布
  ros::Subscriber start_pose_sub_;
  ros::Subscriber goal_pose_sub_;

  ros::Publisher referenceline_pub_;
};










// void Smooth_Path_array(const int& start_index, const int& end_index,
//                        const geometry_msgs::PoseArray& stores_path_array, Dynamic_planning::Road_mags& Original_Path);

// void Calc_Three_Coeffient(const double start_x, const double start_dx, const double start_ddx,
//                           const double end_x, const double end_dx,
//                           const double dT, Eigen::MatrixXd& coeff);
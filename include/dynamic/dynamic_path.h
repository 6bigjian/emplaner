#pragma once

#include "emplaner/head.h"
#include "obstacle/obstacle.h"

class Dynamic_Plan
{
public:
  Dynamic_Plan();
  ~Dynamic_Plan();

  void dynamic_thread_worker();
  void start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void odom_call_back(const nav_msgs::Odometry& odom);
  void CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg);

  void Update_Path_mags(Dynamic_planning::Road_mags& reference_path, const Dynamic_planning::Road_mags& row_path, const geometry_msgs::Pose& car_pose);
  void First_dynamic_planning(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose, const double velocity);
  void Generate_Dynamic_Path(const geometry_msgs::Pose& car_pose ,const double velocity);
  bool Calc_Plan_Start_Point(const Dynamic_planning::Road_mags& pre_path , const double velocity,
                             const geometry_msgs::Pose& car_pose, routing_msgs::vehicle_pose& plan_start_pose);
  void Calc_Plan_Goal_Point(const Dynamic_planning::Road_mags& Original_Path, const double velocity,
                            const geometry_msgs::Pose& goal_pose, Dynamic_planning::Frenet_mags& plan_goal_mags);

  Dynamic_planning::Frenet_mags Cartesian2Frenet(const routing_msgs::vehicle_pose& plan_start_pose,
                              const Dynamic_planning::Road_mags& reference_path, const double velocity);

  void dynamic_planning(const Dynamic_planning::Frenet_mags& plan_start_mags, const uint16_t col = 8, const uint16_t row = 7,
                        const double sample_s = 5, const double sample_l = 0.5);

  void goal_dynamic_planning(const Dynamic_planning::Frenet_mags& plan_start_mags, const Dynamic_planning::Frenet_mags& plan_goal_mags);

  Eigen::VectorXi Generate_coeff(const Dynamic_planning::Frenet_mags& plan_start_mags, std::vector<Eigen::MatrixXd>& node_coeff,
                                const uint16_t col, const uint16_t row, const double sample_s, const double sample_l);
  void Generate_Dynamic_Frenet(const std::vector<Eigen::MatrixXd>& node_coeff, const Eigen::VectorXi& node_list_row,
                              const Dynamic_planning::Frenet_mags& plan_start_mags,std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet, 
                              const uint16_t col, const uint16_t row, const double sample_s);

  void Frenet_trans_Cartesian(const std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet, const Dynamic_planning::Road_mags& referenceline_mags);

  double Calc_Neighbour_Cost(const Dynamic_planning::Frenet_mags& plan_start_mags, const Dynamic_planning::Frenet_mags& plan_end_mags, 
                            Eigen::MatrixXd& coeff, const uint16_t cost_ref = 2, const uint16_t cost_dl = 10, const uint16_t cost_ddl = 100);

  void Calc_Quintc_Coeffient(const double start_l, const double start_dl, const double start_ddl,
                            const double end_l, const double end_dl, const double end_ddl,
                            const double start_s, const double end_s, Eigen::MatrixXd& coeff);

  

private:
  geometry_msgs::Pose start_pose_;  //起点位置坐标
  geometry_msgs::Pose goal_pose_;   //终点位置坐标
  geometry_msgs::Pose New_car_pose;

  Dynamic_planning::Road_mags global_reference; //全局路劲
  Dynamic_planning::Road_mags dynamic_path;     //动态规划得出的路劲
  Dynamic_planning::Road_mags reference_path;   //从全局路劲提取出来的局部参考路劲

  std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet_;//动态规划的frenet路径

  Dynamic_planning::Frenet_mags plan_start_mags;//动态规划起点信息
  Dynamic_planning::Frenet_mags plan_goal_mags;//动态规划终点信息，单指全局终点
  
  bool gloLine_build_;
  bool first_dyn = true;
  bool simulation_flag;
  bool closeGoal = false; //动态规划靠近终点

  bool retraj = true;
  bool savetrajflag_ = false;//是否保存一部分动态规划规矩，来缩短规划时间

  u_int16_t dyCol = 8;
  u_int16_t dyRow = 7;    //动态规划的行数,必须位奇数
  double dySimple_s = 5.0;
  double dySimple_l = 0.5;

  int16_t start_search_index_;
  int16_t end_search_index_;
  
  boost::thread* generate_dyn_thread_;

  ObsProj obs_;//障碍物

  const std::string Frame_id = "planning_odom";

  //消息订阅发布
  ros::Subscriber odom_sub_;
  ros::Subscriber start_pose_sub_;
  ros::Subscriber goal_pose_sub_;

  ros::Publisher referenceline_pub_;
};




  // void smooth_coeff(const Eigen::VectorXi& node_list_row, const Dynamic_planning::Frenet_mags& plan_start_mags,
  //                   std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet,
  //                   const uint16_t col, const uint16_t row, const double sample_s, const double sample_l);









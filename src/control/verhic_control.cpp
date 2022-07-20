#include "control/verhic_control.h"
#include "global/global_planning.h"


#define sign(x) ( ((x) <0 )? -1 : 1 )

//获取起点
void Ver_Ctrl::start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Pose start_pose;

    start_pose.position.x = msg->pose.pose.position.x;
    start_pose.position.y = msg->pose.pose.position.y;
    start_pose.orientation = msg->pose.pose.orientation;

    //终点标志位初始化
    global_date::my_mutex.lock();
    global_date::reach_goal_ = false;
    global_date::my_mutex.unlock();

    geometry_msgs::Vector3 ctrl; 
    ctrl.x = 0;
    ctrl.y = 0;
    this->control_data_pub_.publish(ctrl);

    routing_msgs::vehicle_pose vehicle_start_pose;
    vehicle_start_pose.x = msg->pose.pose.position.x;
    vehicle_start_pose.y = msg->pose.pose.position.y;
    vehicle_start_pose.yaw = tf2::getYaw(start_pose.orientation);

    this->vehicle_pose_pub_.publish(vehicle_start_pose);
}

/*获取终点*/
void Ver_Ctrl::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    goal_pose_.orientation = msg->pose.orientation;

    //终点标志位初始化
    global_date::my_mutex.lock();
    global_date::reach_goal_ = false;
    global_date::my_mutex.unlock();

    if(this->simulation_flag == true)
    {
      geometry_msgs::Vector3 c; 
      c.x = 0;
      c.y = 0;
      this->control_data_pub_.publish(c);
    }
    else
    {
      geometry_msgs::Twist T;
      T.angular.z = 0;
      T.linear.x = 0;
      this->control_data_pub_.publish(T);
    }

}

/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void Ver_Ctrl::odom_call_back(const nav_msgs::Odometry& odom)
{
  New_car_pose.position.x = odom.pose.pose.position.x;
  New_car_pose.position.y = odom.pose.pose.position.y;
  New_car_pose.orientation = odom.pose.pose.orientation;
}

void Ver_Ctrl::CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg)
{
  //定位
  Eigen::Vector3d gp_start_pt_;
  Eigen::Vector3d gp_start_gps_;

  //将定位信息转为MGRS坐标
  gp_start_gps_(0) = msg->lat;
  gp_start_gps_(1) = msg->lon;
  Global_Plan::GPSToMGRS(gp_start_gps_, gp_start_pt_);

  //将角度转为弧度
  double heading = msg->heading;
  heading = -heading + 90;
  double pi_to_degree = M_PI / 180.0;
  heading *= pi_to_degree;

  //实时定位作为实时位置获取
  New_car_pose.position.x = gp_start_pt_(0) - Hdmap_Build::origin_point_(0);
  New_car_pose.position.y = gp_start_pt_(1) - Hdmap_Build::origin_point_(1);
  New_car_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, heading);
}

void Ver_Ctrl::trajLineCallBack(const nav_msgs::Path& Path)
{
  this->traj_line.poses.clear();
  this->traj_line.poses = Path.poses;
}




Ver_Ctrl::Ver_Ctrl()
{
  ros::NodeHandle n_ctrl;
  n_ctrl.param("simulation_model", this->simulation_flag, true);

  if(this->simulation_flag == true)
  {
    this->start_pose_sub_ = n_ctrl.subscribe("/initialpose", 10, &Ver_Ctrl::start_pose_call_back, this);
    this->odom_sub_ = n_ctrl.subscribe("odom", 10, &Ver_Ctrl::odom_call_back, this);
    this->control_data_pub_ = n_ctrl.advertise<geometry_msgs::Vector3>("control_data", 2); 
    this->vehicle_pose_pub_ = n_ctrl.advertise<routing_msgs::vehicle_pose>("vehicle_start_pose", 10);//发送坐标到rivz显示模型的位置
  }
  else
  {
    this->start_pose_sub_ = 
        n_ctrl.subscribe("/localization/fusion_position", 10, &Ver_Ctrl::CurGPSPtCallBack, this);
    this->control_data_pub_ = n_ctrl.advertise<geometry_msgs::Twist>("/control/control_parameter", 2);
  }
  this->goal_pose_sub_ = 
    n_ctrl.subscribe("/move_base_simple/goal", 10, &Ver_Ctrl::goal_pose_call_back, this);
  this->traj_line_sub_ = 
    n_ctrl.subscribe("trajLine_point",10, &Ver_Ctrl::trajLineCallBack, this);

  generate_ctrl_thread_ = new boost::thread(boost::bind(&Ver_Ctrl::ctrl_thread_worker, this));
}



Ver_Ctrl::~Ver_Ctrl()
{
  if(generate_ctrl_thread_ != NULL)
  {
    delete generate_ctrl_thread_;
    generate_ctrl_thread_ = NULL;
  }
}

#define control_frequency   100u

void Ver_Ctrl::ctrl_thread_worker()
{
  ros::Rate loop_rate(control_frequency);

  while(ros::ok())
  {
    ros::spinOnce();

    global_date::my_mutex.lock();
    this->ctrl_flag = global_date::smoLine_build_;
    global_date::my_mutex.unlock();

    if(this->ctrl_flag == true)
    {
      verhicle_control(this->traj_line, this->New_car_pose);
    }
    
    loop_rate.sleep();
  }
}


/*计算车辆打角
 * 输入:    referenceline   参考线路径
 *         car_pose         车辆当前位置
 * 输出：    msg             车辆的前轮打角*/
void Ver_Ctrl::verhicle_control(const nav_msgs::Path& referenceline, const geometry_msgs::Pose& car_pose)
{
  //根据上一时刻的匹配点坐标，来计算这一时刻的匹配点坐标，缩短遍历时间

  //计算当前车辆的匹配点
  carindex = Global_Plan::Search_Match_Point(car_pose.position.x, car_pose.position.y, referenceline, 0, referenceline.poses.size()-1);
  double point_distance;
  //计算车辆后轮中心点的位置
  double x = car_pose.position.x - 0.5*global_date::vehicleLength*cos(tf2::getYaw(car_pose.orientation));
  double y = car_pose.position.y - 0.5*global_date::vehicleLength*sin(tf2::getYaw(car_pose.orientation));
  int16_t i;
  //遍历寻找前视距离点
  for(i = carindex; i < referenceline.poses.size()-1; i++)
  {
    point_distance = sqrt(pow(referenceline.poses[i].pose.position.x - x, 2) +
                          pow(referenceline.poses[i].pose.position.y - y, 2));
    if(point_distance > Ld) break;
  }

  
  //计算车辆航向与道路切线方向的夹角
  double altha = tf2::getYaw(car_pose.orientation) - 
         atan2((referenceline.poses[i].pose.position.y - y), (referenceline.poses[i].pose.position.x - x));
  //纯跟踪法计算偏转角
  geometry_msgs::Vector3 msg; 
  msg.y = -altha;
  global_date::my_mutex.lock();
  if(sqrt(pow(car_pose.position.x - goal_pose_.position.x, 2) +
          pow(car_pose.position.y - goal_pose_.position.y, 2)) < 0.5)//抵达终点
  {
    global_date::reach_goal_ = true;
    global_date::gloLine_build_ = false;
    global_date::partLine_build_ = false;
    msg.x = 0;
  }
  else if(global_date::gloLine_build_ == false) //仿真重新选取了起点
  {
    msg.x = 0;
  }
  else
  {
    msg.x = referenceline.poses[carindex].pose.orientation.w;
    // ROS_WARN("VX is :%f",msg.x);
  }
  global_date::my_mutex.unlock();
  //实际跑车打角
  geometry_msgs::Twist T3;
  T3.angular.z = -atan2(2*global_date::vehicleLength*sin(altha), point_distance);
  T3.linear.x = msg.x;

  if(this->simulation_flag = true) this->control_data_pub_.publish(msg);
  else this->control_data_pub_.publish(T3);
}



double PIDcontrol(int16_t carindex, const nav_msgs::Path& referenceline, const geometry_msgs::Pose& car_pose)
{
  carindex += 5;
  if(carindex >= referenceline.poses.size())
  {
    carindex = referenceline.poses.size() - 1;
  }
  double car_ver[2];
  car_ver[0] = car_pose.position.x - referenceline.poses[carindex].pose.position.x;
  car_ver[1] = car_pose.position.y - referenceline.poses[carindex].pose.position.y;

  double t_r[2];
  if(carindex < referenceline.poses.size()-1)
  {
    t_r[0] = referenceline.poses[carindex + 1].pose.position.x -
             referenceline.poses[carindex].pose.position.x;
    t_r[1] = referenceline.poses[carindex + 1].pose.position.y -
             referenceline.poses[carindex].pose.position.y;
  }
  else
  {
    t_r[0] = referenceline.poses[carindex].pose.position.x -
             referenceline.poses[carindex - 1].pose.position.x;
    t_r[1] = referenceline.poses[carindex].pose.position.y -
             referenceline.poses[carindex - 1].pose.position.y;
  }

  double n_r[2] ={-sin(atan2(t_r[1], t_r[0])), cos(atan2(t_r[1], t_r[0]))};

  double distance = (car_ver[0] * n_r[0] + car_ver[1] * n_r[1]);

  // return cont.count_e(-distance);
  return 0;

}



double PID::count_e(double error)
{
  integral += ki * error;
  if(abs(integral) > output_limit)
  {
    integral = output_limit*sign(integral);
  }

  double out = kp * error + integral;

  if(abs(out) > output_limit)
  {
    out = output_limit*sign(out);
  }

  return out;
}

void PID::clear_integral()
{
  integral = 0;
}
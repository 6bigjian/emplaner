#include "QuadProg/smooth.h"
#include "global/global_planning.h"
#include "obstacle/obstacle.h"


/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void SmoLine::odom_call_back(const nav_msgs::Odometry& odom)
{
  New_car_pose.position.x = odom.pose.pose.position.x;
  New_car_pose.position.y = odom.pose.pose.position.y;
  New_car_pose.orientation = odom.pose.pose.orientation;
}

/*将GPS下的定位转换到直角坐标系
 *这里获取的定位信息是偏移后的
 *如果要用于规划，记得减去偏移量*/
void SmoLine::GPSToMGRS(Eigen::Vector3d& gps, Eigen::Vector3d& mgrs)
{
  lanelet::GPSPoint lanelet2_gps;
  lanelet2_gps.lat = gps(0);
  lanelet2_gps.lon = gps(1);
  mgrs(0) = Hdmap_Build::projector.forward(lanelet2_gps).x();
  mgrs(1) = Hdmap_Build::projector.forward(lanelet2_gps).y();
  mgrs(2) = 0;
}

void SmoLine::CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg)
{
  //定位
  Eigen::Vector3d gp_start_pt_;
  Eigen::Vector3d gp_start_gps_;

  //将定位信息转为MGRS坐标
  gp_start_gps_(0) = msg->lat;
  gp_start_gps_(1) = msg->lon;
  SmoLine::GPSToMGRS(gp_start_gps_, gp_start_pt_);

  //将角度转为弧度
  double heading = msg->heading;
  heading = -heading + 90;
  double pi_to_degree = M_PI / 180.0;
  heading *= pi_to_degree;

  //车辆定位信息
  New_car_pose.position.x = gp_start_pt_(0) - Hdmap_Build::origin_point_(0);
  New_car_pose.position.y = gp_start_pt_(1) - Hdmap_Build::origin_point_(1);
  New_car_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, heading);
}

/*获取终点*/
void SmoLine::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goal_pose.position.x = msg->pose.position.x;
  goal_pose.position.y = msg->pose.position.y;
  goal_pose.orientation = msg->pose.orientation;
}


SmoLine::SmoLine()
{
  ros::NodeHandle n_sml;
  n_sml.param("simulation_model", this->simulation_flag, true);

  if(this->simulation_flag == true)
  {
    this->odom_sub_ = n_sml.subscribe("odom", 10, &SmoLine::odom_call_back, this);
  }
  else
  {
    this->odom_sub_ = 
        n_sml.subscribe("/localization/fusion_position", 10, &SmoLine::CurGPSPtCallBack, this);
  }

  this->trajline_pub_ = 
      n_sml.advertise<nav_msgs::Path>("trajLine_point", 10);

  this->marker_pub_ = n_sml.advertise<visualization_msgs::Marker>("smool_marker", 10);

  this->goal_pose_sub_ = 
    n_sml.subscribe("/move_base_simple/goal", 10, &SmoLine::goal_pose_call_back, this);

  this->vehicleWidth = global_date::vehicleWidth;//获取车辆长宽，只读数据无需上锁
  this->vehicleLength = global_date::vehicleLength;

  this->generate_sml_thread_ = new boost::thread(boost::bind(&SmoLine::sml_thread_worker, this));
}

SmoLine::~SmoLine()
{
#if qpsolver == qpoasessolver
  qpOASES_burn();
#endif

  if(generate_sml_thread_ != NULL)
  {
    delete generate_sml_thread_;
    generate_sml_thread_ = NULL;
  }
}


void SmoLine::sml_thread_worker()
{
  ros::Rate loop_rate(5);

#if qpsolver == qpoasessolver
    qpOASES_init();
#endif

  while(ros::ok())
  {
    ros::spinOnce();

    global_date::my_mutex.lock();
    this->partLine_build_ = global_date::partLine_build_;
    if(this->partLine_build_ == true)
    {
      this->reference_path.cope_data(Sline::reference_path);
      this->dynamic_frenet = Sline::dynamic_frenet;
      global_date::retraj_flag_ = this->retraj;
    }
    else global_date::smoLine_build_ = false;

    global_date::my_mutex.unlock();

    if(this->partLine_build_ == true)
    {
      global_date::obj_mutex.lock();
      generate_convex_space();
      global_date::obj_mutex.unlock();
      this->first_sml = false;

#if qpsolver == qpoasessolver

      if(arraysize == arrayCapacity) qpOASES_solver();
      else qpOASES_solver(arraysize);
        
#endif

      global_date::my_mutex.lock();
      global_date::smoLine_build_ = true;
      global_date::my_mutex.unlock();
    }
    else this->first_sml = true;

    loop_rate.sleep();
  }
}


//计算凸空间的上界和下界
void SmoLine::generate_convex_space()
{
  this->retraj = Calc_Plan_Start_Point();

  double start_s;
  if(this->retraj == true) //从车辆起点开始规划
  {
    plan_start_mags = Cartesian2Frenet(New_car_pose);
    vehTotraj_projIndex_ = retaintraj_num - 3;//要保留二次规划起点的约束
    //计算当前车辆在全局参考线上的投影坐标s
    int16_t start_index = Global_Plan::Search_Match_Point(New_car_pose.position.x, New_car_pose.position.y, 
                        reference_path.referenceline, 0, reference_path.referenceline.poses.size()-1);

    start_s = reference_path.s[start_index];

    for(uint16_t i = 0; i < arrayCapacity; i++)
    {
      if(start_s + i*this->ds < dynamic_frenet.back().s) //平滑的路径要在动态规划的范围内
      {
        L_limit[i][0] = start_s + i*this->ds;
        L_limit[i][1] = borderLimit;
        L_limit[i][2] = -borderLimit;

        int16_t match_index = FindNearIndex(L_limit[i][0]);
        if(match_index == -2) match_index = 0;

        L_limit[i][3] = dynamic_frenet[match_index].l;

        arraysize = i+1;
      }
      else break;
    }
  }
  else//保留上一时刻部分轨迹
  {
    start_s = L_limit[vehTotraj_projIndex_][0];

    for(uint16_t i = 0; i < arrayCapacity; i++)
    {
      if(start_s + i*this->ds < dynamic_frenet.back().s) //平滑的路径要在动态规划的范围内
      {
        L_limit[i][0] = start_s + i*this->ds;
        L_limit[i][1] = borderLimit;
        L_limit[i][2] = -borderLimit;

        int16_t match_index = FindNearIndex(L_limit[i][0]);
        if(match_index == -2) match_index = 0;

        //递归保留上一时刻二次规划出来的道路
        if(i < retaintraj_num - vehTotraj_projIndex_) L_limit[i][3] = L_limit[i + vehTotraj_projIndex_][3];
        else L_limit[i][3] = dynamic_frenet[match_index].l;

        arraysize = i+1;
      }
      else break;
    }
  }

  for(uint16_t i = 0; i < ObsProj::All_obstacle.size(); i++)
  {
    if(ObsProj::All_obstacle[i].obs_flag == false) continue;

    double obs_s_min = ObsProj::All_obstacle[i].s_set - ObsProj::All_obstacle[i].length/2;
    double obs_s_max = ObsProj::All_obstacle[i].s_set + ObsProj::All_obstacle[i].length/2;

    int16_t start_index = FindObjIndex(obs_s_min, MINPOINT);
    int16_t end_index = FindObjIndex(obs_s_max, MAXPOINT);
    int16_t centre_index = FindObjIndex(ObsProj::All_obstacle[i].s_set, MIDPOINT);
    if(centre_index == LINEBACK) centre_index = 0;
    else if(centre_index == LINEFRONT) centre_index = arraysize-1;

    if(start_index == LINEFRONT) continue;//障碍物超出界限
    else if(end_index == LINEBACK) continue;//障碍物在车后面
    else
    {
      if(start_index == LINEBACK) //表示障碍物一部分在车后面，一部分在前面
      {
        start_index = 0;
        if(end_index == LINEFRONT) end_index = arraysize;//障碍物超长，超过了整个规划的范围
      }
      else if(end_index == LINEFRONT)  //表示障碍物一部分超出二次规划范围，一部分在范围内
      {
        end_index = arraysize;
      }

      if(L_limit[centre_index][3] > ObsProj::All_obstacle[i].l_set)  //动态规划向左绕
      {
        for(int16_t j = start_index; j < end_index; j++)
        {
            if(L_limit[j][2] < ObsProj::All_obstacle[i].l_set + ObsProj::All_obstacle[i].width/2)
              L_limit[j][2] = ObsProj::All_obstacle[i].l_set + ObsProj::All_obstacle[i].width/2;
        }
      }
      else    //动态规划向右绕
      {
        for(int16_t j = start_index; j < end_index; j++)
        {
            if(L_limit[j][1] > ObsProj::All_obstacle[i].l_set - ObsProj::All_obstacle[i].width/2)
              L_limit[j][1] = ObsProj::All_obstacle[i].l_set - ObsProj::All_obstacle[i].width/2;
        }
      }
    }
  }
}



/*计算障碍物在动态规划路径上最近的点坐标
  输入：  s_set           投影到FL坐标系的s
  输出：  index           投影点
  错误代码：LINEFRONT超出规划范围。LINEBACK规划后面
*/
int16_t SmoLine::FindObjIndex(const double s_set, objpoint OBJPOINT)
{
  int16_t index = 0;

  if(L_limit[0][0] > s_set) return LINEBACK;    //表示在后面
  else if(L_limit[arraysize-1][0] <= s_set) return LINEFRONT; //表示超出界限
  else
  {
    for(u_int16_t i = 0; ; i++)
    {
      if(L_limit[i][0] <= s_set) index++;
      else break;
    }

    if(OBJPOINT == MINPOINT) return index-1;
    else if(OBJPOINT == MAXPOINT) return index;
    else return index;
  }
}

/*计算在动态规划路径上最近的点坐标
  输入：  s_set           投影到FL坐标系的s
  输出：  index           投影点
*/
int16_t SmoLine::FindNearIndex(const double s_set)
{
  int16_t index = 0;

  if(dynamic_frenet[0].s > s_set) return -2;    //-2表示在后面
  else if(dynamic_frenet.back().s < s_set) return -1; //-1表示超出界限
  else
  {
    for(u_int16_t i = 0; ; i++)
    {
      if(dynamic_frenet[i].s < s_set) index++;
      else break;
    }

    if(dynamic_frenet[index].s - s_set > s_set - dynamic_frenet[index-1].s)
      return index-1;
    else return index;
  }
}



/*计算规划起点的Frenet坐标信息
 *输入：      match_pose                笛卡尔坐标下需要转换位置点信息
 *           reference_path            参考线
 *           velocity                  线速度
 *输出：      pose_mags           Frenet坐标下规划起点信息
 */
Dynamic_planning::Frenet_mags SmoLine::Cartesian2Frenet(geometry_msgs::Pose& match_pose)
{
  Dynamic_planning::Frenet_mags pose_mags;
  //计算规划起点在参考线上的投影点
  int16_t match_point_index = Global_Plan::Search_Match_Point(match_pose.position.x, match_pose.position.y, 
                              reference_path.referenceline, 0, reference_path.referenceline.poses.size() - 1);
  //车到匹配点的误差向量
  double d_err[2] = {(match_pose.position.x - reference_path.referenceline.poses[match_point_index].pose.position.x),
                     (match_pose.position.y - reference_path.referenceline.poses[match_point_index].pose.position.y)};
  //匹配点的法向量
  double nor[2] = {-sin(reference_path.theta[match_point_index]), cos(reference_path.theta[match_point_index])};
  //匹配点的切向量
  double tor[2] = {cos(reference_path.theta[match_point_index]), sin(reference_path.theta[match_point_index])};
  //纵向误差
  double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
  //横向误差,l
  pose_mags.l = d_err[0] * nor[0] + d_err[1] * nor[1];
  //纵向距离s
  pose_mags.s = reference_path.s[match_point_index] + lon_err;
  //车辆航向角
  double yaw = tf2::getYaw(match_pose.orientation);
  //计算l对时间t的导数 l_dot = v * n_r
  pose_mags.l_dot = (cos(yaw) * nor[0] + sin(yaw) * nor[1]);
  //计算s对t的导数 s_dot = (v * t_r)/(1 - k_r * l)
  pose_mags.s_dot = (cos(yaw) * tor[0] + sin(yaw) * tor[1]);
  pose_mags.s_dot = pose_mags.s_dot / (1 - reference_path.k[match_point_index]*pose_mags.l);
  //计算l对s的导数  dl = l_dot/s_dot;
  pose_mags.dl = 0;
  // if(abs(pose_mags.s_dot) > 0.001)
  pose_mags.dl = pose_mags.l_dot/pose_mags.s_dot;
  // else
  // {
  //   pose_mags.dl = tan(yaw - reference_path.theta[match_point_index]);
  // }
  //认为车匀速运动，在s上没有加速度
  pose_mags.s_dot2 = 0;
  pose_mags.l_dot2 = 0;
  pose_mags.ddl = 0;

  if(abs(pose_mags.dl - yaw + reference_path.theta[match_point_index])> 20*3.14/180)
  {
    ROS_WARN("ERROR");
    std::cout<< "yaw: "<<yaw<<"\n"<<
                "ref theta: "<<reference_path.theta[match_point_index]<<"\n"<<
                "pose_mags.dl: "<< pose_mags.dl<<"\n"<<
                "pose_mags.l: "<<  pose_mags.l <<"\n"<<
                "pose_mags.ldot: "<<pose_mags.l_dot<<"\n"<<
                "pose_mags.s_dot: "<<pose_mags.s_dot<<"\n"<<std::endl;
  }

  return pose_mags;
}



/*frenet转Cartesian
 *输入： dynamic_frenet           动态规划生成的frenet曲线
 *      referenceline_mags       参考线信息
 *输出： dynamic_path             转化为Cartesian坐标系的最终路径点信息
 */
void SmoLine::Frenet_trans_Cartesian()
{
  geometry_msgs::PoseStamped pose_stamp;
  trajline_SLpoint.poses.clear();

  visualization_msgs::Marker L_max,L_min,traj_line;
  L_max.header.frame_id =  "planning_odom";
  L_max.header.stamp = ros::Time::now();
  L_max.ns = "points_and_line";
  L_max.action = visualization_msgs::Marker::ADD;
  L_max.pose.orientation.w = 1.0;
  L_max.id = 0;
  L_max.type = visualization_msgs::Marker::LINE_STRIP;
  L_max.scale.x = 0.1;
  L_max.scale.y = 0.1;
  L_max.color.r = 1.0;
  L_max.color.a = 1.0;

  L_min.header.frame_id =  "planning_odom";
  L_min.header.stamp = ros::Time::now();
  L_min.ns = "points_and_line";
  L_min.action = visualization_msgs::Marker::ADD;
  L_min.pose.orientation.w = 1.0;
  L_min.id = 1;
  L_min.type = visualization_msgs::Marker::LINE_STRIP;
  L_min.scale.x = 0.1;
  L_min.scale.y = 0.1;
  L_min.color.r = 1.0;
  L_min.color.a = 1.0;

  traj_line.header.frame_id =  "planning_odom";
  traj_line.header.stamp = ros::Time::now();
  traj_line.ns = "points_and_line";
  traj_line.action = visualization_msgs::Marker::ADD;
  traj_line.pose.orientation.w = 1.0;
  traj_line.id = 2;
  traj_line.type = visualization_msgs::Marker::LINE_STRIP;
  traj_line.scale.x = 0.01;
  traj_line.scale.y = 0.01;
  traj_line.color.r = 1.0;
  traj_line.color.a = 1.0;

  visualization_msgs::Marker peak;
  peak.header.frame_id =  "planning_odom";
  peak.header.stamp = ros::Time::now();
  peak.ns = "points_and_line";
  peak.action = visualization_msgs::Marker::ADD;
  peak.pose.orientation.w = 1.0;
  peak.id = 3;
  peak.type = visualization_msgs::Marker::POINTS;
  peak.scale.x = 0.1;
  peak.scale.y = 0.1;
  peak.color.r = 1.0;
  peak.color.a = 1.0;
  /********************/

  uint16_t match_index = 0;
  double tor[2],nor[2];
  double ds;
  double project_point[2];//匹配点与投影点
  geometry_msgs::Point p;
  for(uint16_t i = 0; i < arraysize; i++)
  {
    //遍历寻找s对应的匹配点,一直到referenceline_mags.s大于dynamic_frenet.s就退出
    while(L_limit[i][0] > reference_path.s[match_index])
    {
      match_index++;
      if(match_index >= reference_path.s.size()) goto fail;
    }
    //求匹配点与投影点之间的ds
    ds = L_limit[i][0] - reference_path.s[match_index];
    //切向量
    tor[0] = cos(reference_path.theta[match_index]); 
    tor[1] = sin(reference_path.theta[match_index]); 
    //计算投影点信息
    project_point[0] = reference_path.referenceline.poses[match_index].pose.position.x + ds * tor[0];
    project_point[1] = reference_path.referenceline.poses[match_index].pose.position.y + ds * tor[1];
    //计算法；向量
    nor[0] = -sin(reference_path.theta[match_index]); 
    nor[1] = cos(reference_path.theta[match_index]); 

    p.x = project_point[0] + L_limit[i][1] * nor[0];
    p.y = project_point[1] + L_limit[i][1] * nor[1];
    L_max.points.push_back(p);

    p.x = project_point[0] + L_limit[i][2] * nor[0];
    p.y = project_point[1] + L_limit[i][2] * nor[1];
    L_min.points.push_back(p);

    p.x = project_point[0] + L_limit[i][3] * nor[0];
    p.y = project_point[1] + L_limit[i][3] * nor[1];
    traj_line.points.push_back(p);
    // peak.points.push_back(p);
    pose_stamp.pose.position = p;
    trajline_SLpoint.poses.push_back(pose_stamp);
  }

fail:

  this->marker_pub_.publish(L_max);
  this->marker_pub_.publish(L_min);
  // this->marker_pub_.publish(traj_line);
  // this->marker_pub_.publish(peak);
}


/*计算车辆的规划起点，并判断是否需要重新以车辆位置为起点规划，需要true,不需要false*/
bool SmoLine::Calc_Plan_Start_Point()
{
  if(this->first_sml == true)//第一次规划从起点开始
    return true;

  //计算当前车辆与上一时刻平滑轨迹的匹配点
  this->vehTotraj_projIndex_ = Global_Plan::Search_Match_Point(New_car_pose.position.x, New_car_pose.position.y, 
                                            trajline_point, 0, trajline_point.poses.size() - 1);
  if(this->vehTotraj_projIndex_ >= retaintraj_num) return true; //跑得太远，需要从新规划
  //车到匹配点的误差向量
  double d_err[2] = {(New_car_pose.position.x - trajline_point.poses[this->vehTotraj_projIndex_].pose.position.x),
                     (New_car_pose.position.y - trajline_point.poses[this->vehTotraj_projIndex_].pose.position.y)};

  //匹配点的法向量
  double nor[2] = {-sin(traj_theta[this->vehTotraj_projIndex_]), cos(traj_theta[this->vehTotraj_projIndex_])};
  //匹配点的切向量
  double tor[2] = {cos(traj_theta[this->vehTotraj_projIndex_]), sin(traj_theta[this->vehTotraj_projIndex_])};
  //纵向误差
  double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
  //横向误差
  double lat_err = d_err[0] * nor[0] + d_err[1] * nor[1];

  if(abs(lat_err) > 0.5) return true;//横向误差太大
  else return false;
}



/*计算平滑轨迹的theta*/
void SmoLine::calc_traj_theta()
{
  this->traj_theta.clear();

  this->traj_theta.push_back(atan2((trajline_point.poses[1].pose.position.y - trajline_point.poses[0].pose.position.y),
                                   (trajline_point.poses[1].pose.position.x - trajline_point.poses[0].pose.position.x)));

  for(uint16_t i = 1; i < trajline_point.poses.size(); i++)
  {
    this->traj_theta.push_back(atan2((trajline_point.poses[i].pose.position.y - trajline_point.poses[i-1].pose.position.y),
                                     (trajline_point.poses[i].pose.position.x - trajline_point.poses[i-1].pose.position.x)));
  }
}

#include "dynamic/dynamic_path.h"
#include "global/global_planning.h"
#include "global/hdmap_building.h"
#include "obstacle/obstacle.h"

//获取起点
void Dynamic_Plan::start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    start_pose_.position.x = msg->pose.pose.position.x;
    start_pose_.position.y = msg->pose.pose.position.y;
    start_pose_.orientation = msg->pose.pose.orientation;

    global_date::my_mutex.lock();
    global_date::partLine_build_ = false;
    global_date::my_mutex.unlock();
}

/*获取终点*/
void Dynamic_Plan::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    goal_pose_.orientation = msg->pose.orientation;
}

/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void Dynamic_Plan::odom_call_back(const nav_msgs::Odometry& odom)
{
  New_car_pose.position.x = odom.pose.pose.position.x;
  New_car_pose.position.y = odom.pose.pose.position.y;
  New_car_pose.orientation = odom.pose.pose.orientation;
}

void Dynamic_Plan::CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg)
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

  //规划起点定位信息
  start_pose_.position.x = gp_start_pt_(0) - Hdmap_Build::origin_point_(0);
  start_pose_.position.y = gp_start_pt_(1) - Hdmap_Build::origin_point_(1);
  start_pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, heading);

  //实时定位作为实时位置获取
  New_car_pose.position.x = start_pose_.position.x;
  New_car_pose.position.y = start_pose_.position.y;
  New_car_pose.position.z = 0;
  New_car_pose.orientation = start_pose_.orientation;
}



Dynamic_Plan::Dynamic_Plan()
{
  ros::NodeHandle n_dy;
  n_dy.param("simulation_model", this->simulation_flag, true);

  if(this->simulation_flag == true)
  {
    this->start_pose_sub_ = n_dy.subscribe("/initialpose", 10, &Dynamic_Plan::start_pose_call_back, this);
    this->odom_sub_ = n_dy.subscribe("odom", 10, &Dynamic_Plan::odom_call_back, this);
  }
  else
  {
    this->start_pose_sub_ = 
        n_dy.subscribe("/localization/fusion_position", 10, &Dynamic_Plan::CurGPSPtCallBack, this);
  }
  this->goal_pose_sub_ = 
    n_dy.subscribe("/move_base_simple/goal", 10, &Dynamic_Plan::goal_pose_call_back, this);
  this->referenceline_pub_ = 
      n_dy.advertise<nav_msgs::Path>("ReferenceLine_CenterPoint", 10);

  this->generate_dyn_thread_ = new boost::thread(boost::bind(&Dynamic_Plan::dynamic_thread_worker, this));
}


Dynamic_Plan::~Dynamic_Plan()
{
  if(generate_dyn_thread_ != NULL)
  {
    delete generate_dyn_thread_;
    generate_dyn_thread_ = NULL;
  }
}


void Dynamic_Plan::dynamic_thread_worker()
{
  ros::Rate loop_rate(4);//变为250ms动态规划一次

  while(ros::ok())
  {
    ros::spinOnce();

    global_date::my_mutex.lock();
    this->gloLine_build_ = global_date::gloLine_build_;

    if(this->savetrajflag_ == true) this->retraj = global_date::retraj_flag_;
    else this->retraj = true;

    global_date::my_mutex.unlock();

    if(this->gloLine_build_ == true)
    {
      if(this->first_dyn == true)
      {
        //第一次规划的时候先提取
        global_date::my_mutex.lock();
        this->global_reference.cope_data(Hdmap_Build::Global_Path_);
        global_date::my_mutex.unlock();

        this->obs_.Get_globline();
        this->obs_.Set_Obstacle();

        this->dynamic_frenet_.clear();//动态规划先把上一次的frenet路径清楚

        this->start_search_index_ = 0;
        this->end_search_index_ = this->global_reference.referenceline.poses.size()-1;

        First_dynamic_planning(start_pose_, goal_pose_, 2);
        this->first_dyn = false;

        global_date::my_mutex.lock();
        global_date::partLine_build_ = true;
        global_date::my_mutex.unlock();
      }
      else
      {
        this->obs_.Set_Obstacle();
        Generate_Dynamic_Path(New_car_pose, 2);
        clock_t endTime = clock();
      }

      global_date::my_mutex.lock();
      Sline::dynamic_frenet = this->dynamic_frenet_;
      Sline::reference_path.cope_data(this->reference_path);
      global_date::my_mutex.unlock();

      // ROS_WARN("dynamic line have build");
    }
    else
    {
      this->first_dyn = true;

      global_date::my_mutex.lock();
      global_date::partLine_build_ = false;
      global_date::my_mutex.unlock();
    }
    
    loop_rate.sleep();
  }
}


/*reference_path尽可能保证后面20个点，前面80个点，一共101个点。除非在起点与终点附近，否则车辆位于第21号点即car_index = 20
 *输入：    row_path            全局路径信息
           car_pose            车辆位置信息
 *输出：    reference_path      参考路径信息
 */
void Dynamic_Plan::Update_Path_mags(Dynamic_planning::Road_mags& reference_path, 
                   const Dynamic_planning::Road_mags& row_path, const geometry_msgs::Pose& car_pose)
{
  int16_t row_path_index = Global_Plan::Search_Match_Point(car_pose.position.x, car_pose.position.y, 
                                        row_path.referenceline, start_search_index_, end_search_index_);

  if(row_path.referenceline.poses.size() > 101) //先判断原始参考线是否多与101个点
  {
      if(row_path_index >= 21 && row_path.referenceline.poses.size() - row_path_index > 80) //车辆前方的点>80个,后方的点>20个
      {
          reference_path.clear_data();
          for(int16_t i = row_path_index - 20; i <= row_path_index+80; i++)
          {
                  reference_path.referenceline.poses.push_back(row_path.referenceline.poses[i]);
                  reference_path.s.push_back(row_path.s[i]);
                  reference_path.theta.push_back(row_path.theta[i]);
                  reference_path.k.push_back(row_path.k[i]);
          }
      }
      else if(row_path_index < 21)
      {
          reference_path.clear_data();
          for(int16_t i = 0; i < 101; i++)
          {
                  reference_path.referenceline.poses.push_back(row_path.referenceline.poses[i]);
                  reference_path.s.push_back(row_path.s[i]);
                  reference_path.theta.push_back(row_path.theta[i]);
                  reference_path.k.push_back(row_path.k[i]);
          }
      }
      else if(row_path.referenceline.poses.size() - row_path_index < 80)
      {
          reference_path.clear_data();
          for(int16_t i = row_path.referenceline.poses.size() - 101 ; i < row_path.referenceline.poses.size(); i++)
          {
                  reference_path.referenceline.poses.push_back(row_path.referenceline.poses[i]);
                  reference_path.s.push_back(row_path.s[i]);
                  reference_path.theta.push_back(row_path.theta[i]);
                  reference_path.k.push_back(row_path.k[i]);
          }
      }
  }
  else
  {
      reference_path.clear_data();
      reference_path.referenceline.poses = row_path.referenceline.poses;
      reference_path.k = row_path.k;
      reference_path.s = row_path.s;
      reference_path.theta = row_path.theta;
  }

  //设置下一个周期遍历匹配点的范围，节省时间
  start_search_index_ = row_path_index - 10;
  if(start_search_index_ < 0) 
  start_search_index_ = 0;

  end_search_index_ = row_path_index + 30;
  if(end_search_index_ >= row_path.referenceline.poses.size())
  end_search_index_ = row_path.referenceline.poses.size() - 1;

}


// Dynamic_planning::Dynamic_State dynamic_state;
/*第一次动态规划
 *输入：  start_pose        起点无偏坐标
         goal_pose         终点无偏坐标
         velocity          车辆速度
 *输出：  dynamic_state     动态规划路径
 */
void Dynamic_Plan::First_dynamic_planning(const geometry_msgs::Pose& start_pose ,const geometry_msgs::Pose& goal_pose ,const double velocity)
{
  reference_path.clear_data();
  dynamic_path.clear_data();
  if(global_reference.referenceline.poses.size() > 100)
  {
    reference_path.referenceline.poses.assign(global_reference.referenceline.poses.begin(),global_reference.referenceline.poses.begin()+100);
    reference_path.k.assign(global_reference.k.begin(),global_reference.k.begin()+100);
    reference_path.s.assign(global_reference.s.begin(),global_reference.s.begin()+100);
    reference_path.theta.assign(global_reference.theta.begin(),global_reference.theta.begin()+100);
  }
  else
  {
    reference_path.referenceline.poses = global_reference.referenceline.poses;
    reference_path.k = global_reference.k;
    reference_path.s = global_reference.s;
    reference_path.theta = global_reference.theta;
  }
  reference_path.referenceline.header.frame_id = global_reference.referenceline.header.frame_id;
  reference_path.referenceline.header.stamp = global_reference.referenceline.header.stamp;

  //这里简单的计算一下车辆终点，主要是提取s
  Calc_Plan_Goal_Point(global_reference, velocity, goal_pose, plan_goal_mags);

  routing_msgs::vehicle_pose plan_start_pose;
  //输入上一时刻的轨迹计算规划起点，因为这是第一次规划，所以认为reference_path就算上一时刻的规划
  Calc_Plan_Start_Point(reference_path,0,start_pose,plan_start_pose);
  //将plan_start_pose转化成弗兰纳坐标系plan_start_mags
  plan_start_mags = Cartesian2Frenet(plan_start_pose,reference_path,velocity);

  if(plan_goal_mags.s < reference_path.s.back())
  {
    this->closeGoal = true;
    goal_dynamic_planning(plan_start_mags, plan_goal_mags);

    if(this->dynamic_frenet_.size()>20)
    {
      this->plan_start_mags = this->dynamic_frenet_[10-1];
    }
  }
  else
  {
    dynamic_planning(plan_start_mags,this->dyCol,this->dyRow,this->dySimple_s,this->dySimple_l);

    this->plan_start_mags = this->dynamic_frenet_[20-1];
  }
}

void Dynamic_Plan::Generate_Dynamic_Path(const geometry_msgs::Pose& car_pose ,const double velocity)
{
  Update_Path_mags(reference_path, global_reference, car_pose);

  routing_msgs::vehicle_pose plan_start_pose;

  Dynamic_planning::Road_mags pre_path;

  pre_path.cope_data(dynamic_path);

  this->closeGoal = false;

  if(this->retraj == true)
  { //重新规划轨迹
    Calc_Plan_Start_Point(pre_path,velocity,car_pose,plan_start_pose);
    //将plan_start_pose转化成弗兰纳坐标系plan_start_mags
    plan_start_mags = Cartesian2Frenet(plan_start_pose,reference_path,velocity);

    if(plan_goal_mags.s < reference_path.s.back())
    {
      this->closeGoal = true;

      if(this->dynamic_frenet_.size()>20)
        this->plan_start_mags = this->dynamic_frenet_[10-1];
          
      dynamic_path.clear_data();
      this->dynamic_frenet_.clear();

      goal_dynamic_planning(plan_start_mags, plan_goal_mags);
    }
    else
    {
      dynamic_path.clear_data();
      this->dynamic_frenet_.clear();
      
      dynamic_planning(plan_start_mags,this->dyCol,this->dyRow,this->dySimple_s,this->dySimple_l);

      if(this->savetrajflag_ == true)
        this->plan_start_mags = this->dynamic_frenet_[20-1];
    }
  }
  else
  {
     //寻找car_pose 在动态规划的轨迹的匹配点
    int16_t match_point_index = Global_Plan::Search_Match_Point(car_pose.position.x, car_pose.position.y, 
                              pre_path.referenceline, 0, pre_path.referenceline.poses.size() - 1);

    if(match_point_index >= 10) //车前面的第一个五次多项式走完
    {
      std::vector<Dynamic_planning::Frenet_mags> predy_frenet = this->dynamic_frenet_;
      this->dynamic_frenet_.clear();

      dynamic_path.clear_data();
       if(plan_goal_mags.s < reference_path.s.back())
       {
         this->closeGoal = true;
        //  cout << "true" <<endl;
         for(int16_t i = 10; plan_start_mags.s > pre_path.s[i]; i++)
         {
           this->dynamic_frenet_.push_back(predy_frenet[i]);
            // dynamic_path.referenceline.poses.push_back(pre_path.referenceline.poses[i]);
            // dynamic_path.s.push_back(pre_path.s[i]);
         }
         goal_dynamic_planning(plan_start_mags, plan_goal_mags);
         if(this->dynamic_frenet_.size()>30)
         {
           this->plan_start_mags = this->dynamic_frenet_[20-1];
         }
       }
       else
       {
          for(int16_t i = 10; i < 20; i++)
          {
            this->dynamic_frenet_.push_back(predy_frenet[i]);
            // dynamic_path.referenceline.poses.push_back(pre_path.referenceline.poses[i]);
            // dynamic_path.s.push_back(pre_path.s[i]);
          }

          dynamic_planning(this->plan_start_mags, this->dyCol-1,this->dyRow,this->dySimple_s,this->dySimple_l);
          this->plan_start_mags = this->dynamic_frenet_[20-1];
       }
    }
  }

}


/*计算车辆的规划起点，并判断是否需要重新规划，需要true,不需要false
 *输入： pre_path         上一次的规划轨迹
        stitch_path      待拼接轨迹
        velocity         车辆速度大小
        car_pose         车辆坐标
  输出： plan_start_pose  规划起点信息
*/
bool Dynamic_Plan::Calc_Plan_Start_Point(const Dynamic_planning::Road_mags& pre_path , const double velocity,
                           const geometry_msgs::Pose& car_pose, routing_msgs::vehicle_pose& plan_start_pose)
{
  int16_t match_point_index = Global_Plan::Search_Match_Point(car_pose.position.x, car_pose.position.y, 
                                            pre_path.referenceline, 0, pre_path.referenceline.poses.size() - 1);
  //车到匹配点的误差向量
  double d_err[2] = {(car_pose.position.x - pre_path.referenceline.poses[match_point_index].pose.position.x),
                     (car_pose.position.y - pre_path.referenceline.poses[match_point_index].pose.position.y)};
  //匹配点的法向量
  double nor[2] = {-sin(pre_path.theta[match_point_index]), cos(pre_path.theta[match_point_index])};
  //匹配点的切向量
  double tor[2] = {cos(pre_path.theta[match_point_index]), sin(pre_path.theta[match_point_index])};
  //纵向误差
  double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
  //横向误差
  double lat_err = d_err[0] * nor[0] + d_err[1] * nor[1];

  if(abs(lat_err) > 0.5) //横向误差太大
  {
    plan_start_pose.x = car_pose.position.x + velocity*cos(pre_path.theta[match_point_index])*0.1; //每100ms规划一次
    plan_start_pose.y = car_pose.position.y + velocity*sin(pre_path.theta[match_point_index])*0.1; //每100ms规划一次
    plan_start_pose.yaw = tf2::getYaw(car_pose.orientation); //认为车的方向变化小，可忽略。后期在改吧
    return true;
  }
  else
  {
    plan_start_pose.x = car_pose.position.x;
    plan_start_pose.y = car_pose.position.y;
    plan_start_pose.yaw = tf2::getYaw(car_pose.orientation);
    return false;
  }
}

/*计算全局终点的信息
 *输入：  Original_Path         全局路径
 *       velocity              车辆速度
 *       goal_pose             终点笛卡尔坐标
 *输出：  plan_goal_mags        终点Frenet坐标
 */
void Dynamic_Plan::Calc_Plan_Goal_Point(const Dynamic_planning::Road_mags& Original_Path, const double velocity,
                          const geometry_msgs::Pose& goal_pose, Dynamic_planning::Frenet_mags& plan_goal_mags)
{
  int16_t match_point_index = Global_Plan::Search_Match_Point(goal_pose.position.x, goal_pose.position.y, 
                              Original_Path.referenceline, 0, Original_Path.referenceline.poses.size() - 1);

  //车到匹配点的误差向量
  double d_err[2] = {(goal_pose.position.x - Original_Path.referenceline.poses[match_point_index].pose.position.x),
                     (goal_pose.position.y - Original_Path.referenceline.poses[match_point_index].pose.position.y)};
  //匹配点的法向量
  double nor[2] = {-sin(Original_Path.theta[match_point_index]), cos(Original_Path.theta[match_point_index])};
  //匹配点的切向量
  double tor[2] = {cos(Original_Path.theta[match_point_index]), sin(Original_Path.theta[match_point_index])};
  //纵向误差
  double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
  //横向误差
  double lat_err = d_err[0] * nor[0] + d_err[1] * nor[1];

  plan_goal_mags.s = Original_Path.s[match_point_index] + lon_err;
  plan_goal_mags.l = lat_err;

  double goal_yaw = tf2::getYaw(goal_pose.orientation);
  //计算l对时间t的导数 l_dot = v * n_r
  plan_goal_mags.l_dot = (cos(goal_yaw) * nor[0] + sin(goal_yaw) * nor[1]) * velocity;
  //计算s对t的导数 s_dot = (v * t_r)/(1 - k_r * l)
  plan_goal_mags.s_dot = (cos(goal_yaw) * tor[0] + sin(goal_yaw) * tor[1]) * velocity;
  plan_goal_mags.s_dot = plan_goal_mags.s_dot / (1 - reference_path.k[match_point_index]*plan_goal_mags.l);
  //计算l对s的导数  dl = l_dot/s_dot;
  plan_goal_mags.dl = 0;
  // if(abs(plan_goal_mags.s_dot) > 0.001)
  // {
  //   plan_goal_mags.dl = plan_goal_mags.l_dot/plan_goal_mags.s_dot;
  // }
  // else
  {
    plan_goal_mags.dl = tan(goal_yaw - Original_Path.theta[match_point_index]);
  }
  

  plan_goal_mags.ddl = 0;

  // cout << "goal_mags yaw: "<< goal_yaw*180/3.1415 <<endl;
  // cout << "goal_mags dl: "<< atan2(plan_goal_mags.dl, 1)*180/3.1415 <<endl;
}

/*计算规划起点的Frenet坐标信息
 *输入：      plan_start_pose           笛卡尔坐标下规划起点信息
 *           reference_path            参考线
 *           velocity                  线速度
 *输出：      plan_start_mags           Frenet坐标下规划起点信息
 */
Dynamic_planning::Frenet_mags Dynamic_Plan::Cartesian2Frenet(const routing_msgs::vehicle_pose& plan_start_pose,
                                              const Dynamic_planning::Road_mags& reference_path, const double velocity)
{
  Dynamic_planning::Frenet_mags plan_start_mags;
  //计算规划起点在参考线上的投影点
  int16_t match_point_index = Global_Plan::Search_Match_Point(plan_start_pose.x, plan_start_pose.y, 
                              reference_path.referenceline, 0, reference_path.referenceline.poses.size() - 1);
  //车到匹配点的误差向量
  double d_err[2] = {(plan_start_pose.x - reference_path.referenceline.poses[match_point_index].pose.position.x),
                     (plan_start_pose.y - reference_path.referenceline.poses[match_point_index].pose.position.y)};
  //匹配点的法向量
  double nor[2] = {-sin(reference_path.theta[match_point_index]), cos(reference_path.theta[match_point_index])};
  //匹配点的切向量
  double tor[2] = {cos(reference_path.theta[match_point_index]), sin(reference_path.theta[match_point_index])};
  //纵向误差
  double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
  //横向误差,l
  plan_start_mags.l = d_err[0] * nor[0] + d_err[1] * nor[1];
  //纵向距离s
  plan_start_mags.s = reference_path.s[match_point_index] + lon_err;
  //计算l对时间t的导数 l_dot = v * n_r
  plan_start_mags.l_dot = (cos(plan_start_pose.yaw) * nor[0] + sin(plan_start_pose.yaw) * nor[1]) * velocity;
  //计算s对t的导数 s_dot = (v * t_r)/(1 - k_r * l)
  plan_start_mags.s_dot = (cos(plan_start_pose.yaw) * tor[0] + sin(plan_start_pose.yaw) * tor[1]) * velocity;
  plan_start_mags.s_dot = plan_start_mags.s_dot / (1 - reference_path.k[match_point_index]*plan_start_mags.l);
  //计算l对s的导数  dl = l_dot/s_dot;
  plan_start_mags.dl = 0;
  if(abs(plan_start_mags.s_dot) > 0.001)
  plan_start_mags.dl = plan_start_mags.l_dot/plan_start_mags.s_dot;
  //认为车匀速运动，在s上没有加速度
  plan_start_mags.s_dot2 = 0;
  plan_start_mags.l_dot2 = 0;
  plan_start_mags.ddl = 0;

  return plan_start_mags;
}


/*动态规划
 *输入：  plan_start_mags         规划起点信息
 *       col  row                规划撒点数，默认4列7行
 *       sample_s sample_l       点间距，默认列间距10，行间距1
 *输出：  node_list_row           动态规划最优路径
 */
void Dynamic_Plan::dynamic_planning(const Dynamic_planning::Frenet_mags& plan_start_mags, const uint16_t col, const uint16_t row,
                      const double sample_s, const double sample_l)
{
  std::vector<Eigen::MatrixXd> node_coeff;
  Eigen::VectorXi node_list_row(col); 
  node_list_row = Generate_coeff(plan_start_mags, node_coeff, col, row, sample_s, sample_l);

  // std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet;
  Generate_Dynamic_Frenet(node_coeff, node_list_row, plan_start_mags, dynamic_frenet_, col, row, sample_s);
  // smooth_coeff(node_list_row, plan_start_mags, dynamic_frenet, col, row, sample_s, sample_l);

  Frenet_trans_Cartesian(dynamic_frenet_, reference_path);
}

/*靠近终点时，近距离动态规划
 *输入：      plan_start_mags     规划起点信息
 *           plan_goal_mags      规划重点信息
 */
void Dynamic_Plan::goal_dynamic_planning(const Dynamic_planning::Frenet_mags& plan_start_mags, const Dynamic_planning::Frenet_mags& plan_goal_mags)
{
  //计算起始点之间能有多少列
  int16_t col = (int16_t)((plan_goal_mags.s - plan_start_mags.s)/this->dySimple_s);
  //计算距离余数
  double remain_distance = (plan_goal_mags.s - plan_start_mags.s) - col * this->dySimple_s;

  if(remain_distance > 0)
  {
      col--;
      remain_distance += this->dySimple_s;
  }

  Eigen::MatrixXd coeff;
  // std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet;
  if(col > 0)
  {
    std::vector<Eigen::MatrixXd> node_coeff;
    Eigen::VectorXi node_list_row(col); 
    node_list_row = Generate_coeff(plan_start_mags, node_coeff, col, this->dyRow, this->dySimple_s, this->dySimple_l);

    Generate_Dynamic_Frenet(node_coeff, node_list_row, plan_start_mags, dynamic_frenet_, col, this->dyRow, this->dySimple_s);

    //在记录时先扫描的列在扫描行，所以 i*row 会转移到第i列(如果认为最前的列为第0列的话)，
    //因为 node_list_row 在记录行号的时候加一了，所以要减一
    coeff = node_coeff[(col)* this->dyRow + node_list_row(col - 1) - 1]; 
  }
  else
  {
    Calc_Neighbour_Cost(plan_start_mags,plan_goal_mags,coeff);
  }
  Dynamic_planning::Frenet_mags path_point_mags;
  double ds;
  Eigen::MatrixXd B(3, 1);
  Eigen::MatrixXd A(3, 6);
  int16_t num = (int16_t)(remain_distance/0.5) + 1;

  for(int16_t j = 1; j <= num; j++)
  {
    ds = j * (remain_distance) / num;

    A <<  1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),      pow(ds,5),
          0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),    5*pow(ds,4),
          0,   0,      2,              6*ds,           12*pow(ds,2),   20*pow(ds,3);

    B = A*coeff;

    path_point_mags.s = plan_goal_mags.s - remain_distance + ds;
    path_point_mags.l = B(0, 0);
    path_point_mags.dl = B(1, 0);
    path_point_mags.ddl = B(2, 0);
    // path_point_mags.index = i*20 + j;     //没有坐标0
    dynamic_frenet_.push_back(path_point_mags);

  }

  Frenet_trans_Cartesian(dynamic_frenet_, reference_path);

}

/*计算动态规划离散点最优轨迹
 *输入：  plan_start_mags         规划起点信息
 *       col  row                规划撒点数，默认4列7行
 *       sample_s sample_l       点间距，默认列间距10，行间距1
 *输出：  node_list_row           动态规划最优路径
 *       node_coeff              动态规划各点到起点的代价矩阵
 */
Eigen::VectorXi Dynamic_Plan::Generate_coeff(const Dynamic_planning::Frenet_mags& plan_start_mags, std::vector<Eigen::MatrixXd>& node_coeff, 
                               const uint16_t col, const uint16_t row, const double sample_s, const double sample_l)
{
  //保存各点最小代价路径的五次多项式
  // vector<Eigen::MatrixXd> node_coeff;
  Eigen::MatrixXd coeff(6, 1), col_coeff_list(6, row);

  Eigen::MatrixXd node_cost(row, col);//保存各点到起点的最小代价
  // uint16_t min_cost_index[row][col]; //保存与当前点相连的最优路径的上一列坐标,前是行后是列
  Eigen::MatrixXd min_cost_index(row ,col);

  Dynamic_planning::Frenet_mags end_mags, start_mags(plan_start_mags);
  //计算第一列的位置信息
  end_mags.s = start_mags.s + sample_s;
  end_mags.dl = 0;
  end_mags.ddl = 0;
  //先计算起点到第一列的cost,因为C++的特性，在矩阵对应位置要注意减一
  for(int16_t i = 1; i <= row; i++)
  {
    //第一列各点的l，以最中间那一个点
    end_mags.l = ((row + 1)/2 - i) * sample_l;
    node_cost(i-1, 0) = Calc_Neighbour_Cost(start_mags,end_mags,coeff);
    // min_cost_index[i-1][0] = 0;
    min_cost_index(i-1, 0) = 0;
    node_coeff.push_back(coeff);
  }
  //从第二列开始遍历前面各点
  for(int16_t j = 2; j <= col; j++)
  {
    //遍历当前列的点
    for(int16_t i = 1; i <= row; i++)
    {
      end_mags.s = plan_start_mags.s + j*sample_s;
      end_mags.l = ((row + 1)/2 - i) * sample_l;

      double pre_cost[row];
      uint16_t index;
      double min_cost;
      //遍历上一列的点
      for(int16_t k = 1; k <= row; k++)
      {
        start_mags.s = end_mags.s - sample_s;
        start_mags.l = ((row + 1)/2 - k) * sample_l;
        start_mags.dl = 0;
        start_mags.ddl = 0;
        //计算当前节点通过上一列的第k个节点，到达起点的代价
        pre_cost[k-1] = Calc_Neighbour_Cost(start_mags,end_mags,coeff) + node_cost(k-1, j-2);
        col_coeff_list.col(k-1) = coeff;
      }

      //寻找当前的点到起点的点的最优路径
      min_cost = pre_cost[0]; index = 0;
      coeff = col_coeff_list.col(0);
      for(int16_t k = 1; k < row; k++)
      {
        if(pre_cost[k] < min_cost)
        {
          min_cost = pre_cost[k];
          index = k;
          coeff = col_coeff_list.col(k);
        }
      }
      node_cost(i-1, j-1) = min_cost;
      min_cost_index(i-1, j-1) = index;
      node_coeff.push_back(coeff);
    }
  }

  //如果到了终点规划阶段，需要在最后一列的代价加上到终点的代价
  //node_cost最后一列中的 最小代价行 将是规划起点到终点的最小代价
  if(this->closeGoal == true)
  {
    // cout<<"end="<<"("<<dynamic_state.plan_goal_mags.s<<","<<dynamic_state.plan_goal_mags.l<<")"<<endl;
    
    for(int16_t i = 1; i <= row; i++)
    {
      start_mags.s = plan_start_mags.s + col * sample_s;
      start_mags.l = ((row + 1)/2 - i) * sample_l;
      start_mags.dl = 0;
      start_mags.ddl = 0;

      // cout<<"start="<<"("<<start_mags.s<<","<<start_mags.l<<")"<<endl;
      //计算最后一列的各点到终点的代价
      double cost = Calc_Neighbour_Cost(start_mags, plan_goal_mags, coeff);
      //将这些代价累积到最后一列上，这样得出的最后一列代价就是最优代价
      node_cost(i-1, col-1) = node_cost(i-1, col-1) + cost;
      //保存这些五次多项式系数
      node_coeff.push_back(coeff);
      // cout << "coeff="<<coeff<<endl;
    }
  }

  // cout << node_cost <<endl;

  //遍历最后一列找代价最小的路径
  double min_cost = node_cost(0, col-1);
  int16_t index = 0;
  for(int16_t i = 1; i < row; i++)
  {
    if(node_cost(i, col-1) < min_cost)
    {
      min_cost = node_cost(i, col-1);
      index = i;
    }
  }
  //开始递归向前找最小代价路径
  Eigen::VectorXi node_list_row(col);
  node_list_row(col - 1) = index + 1; //保存行号
  for(int16_t i = 1; i < col; i++)
  {
    // node_list_row(col - 1 - i) = min_cost_index[node_list_row(col - i) - 1][col - i] + 1;
    node_list_row(col - 1 - i) = min_cost_index(node_list_row(col - i) - 1, col - i) + 1;
  }

  // Generate_Dynamic_Frenet(node_coeff, node_list_row, plan_start_mags, col, row, sample_s);

  // cout<<"node cost \n"<<node_cost<<endl;
  // cout <<"min index \n" <<min_cost_index <<endl;
  // cout <<"node list \n" << node_list_row <<endl;

  return node_list_row;
}


// void smooth_coeff(const Eigen::VectorXi& node_list_row, const Dynamic_planning::Frenet_mags& plan_start_mags,
//                   std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet,
//                   const uint16_t col, const uint16_t row, const double sample_s, const double sample_l)
// {
//   double start_l = plan_start_mags.l;
//   double start_dl = plan_start_mags.dl;
//   double start_ddl = plan_start_mags.ddl;

//   double end_l,end_dl,end_ddl;
//   double start_s, end_s;

//   double ds;

//   Dynamic_planning::Frenet_mags path_point_mags;

//   Eigen::MatrixXd coeff(6, 1);

//   for(uint16_t i = 0; i < col; i++)
//   {
//     end_l = ((row + 1)/2 - node_list_row(i)) * sample_l;
//     end_ddl = 0;
//     if(i != col -1)
//     {
//       end_dl = (end_l - start_l)/(2 * sample_s) + 
//                 (node_list_row(i + 1) - node_list_row(i))/(2 * sample_s);
//     }
//     else
//     {
//       end_dl = 0;
//     }

//     start_s = 0;
//     end_s = sample_s;

//     Calc_Quintc_Coeffient(start_l, start_dl, start_ddl,
//                           end_l, end_dl, end_ddl,
//                           start_s, end_s, coeff);

//     Eigen::MatrixXd B(3, 1);
//     Eigen::MatrixXd A(3, 6);
//     for(int16_t j = 1; j <= 10; j++)
//     {
//       ds = j * sample_s / 10;

//       A << 1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),      pow(ds,5),
//           0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),    5*pow(ds,4),
//           0,   0,      2,              6*ds,           12*pow(ds,2),   20*pow(ds,3);

//       B = A*coeff;

//       path_point_mags.s = plan_start_mags.s + i * sample_s + ds;
//       path_point_mags.l = B(0, 0);
//       path_point_mags.dl = B(1, 0);
//       path_point_mags.ddl = B(2, 0);
//       path_point_mags.index = i*10 + j;     //没有坐标0
//       dynamic_frenet.push_back(path_point_mags);
//     }

//     start_l = end_l;
//     start_dl = end_dl;
//     start_ddl = end_ddl;
//   }

// }


/*生成动态规划的frenet曲线
 *输入： node_coeff               动态规划产生的五次多项式矩阵
 *      node_list_row            动态规划最优路径信息
 *      plan_start_mags          动态规划的起点
 *输出： dynamic_frenet           最终生成的动态规划frenet轨迹
 */
void Dynamic_Plan::Generate_Dynamic_Frenet(const std::vector<Eigen::MatrixXd>& node_coeff, const Eigen::VectorXi& node_list_row,
                             const Dynamic_planning::Frenet_mags& plan_start_mags, std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet, 
                             const uint16_t col, const uint16_t row, const double sample_s)
{
  // vector<Frenet_mags> dynamic_frenet;
  Dynamic_planning::Frenet_mags path_point_mags;
  Eigen::MatrixXd coeff(6, 1);
  for(int16_t i = 0; i < col ;i++) //遍历列
  {
    //在记录时先扫描的列在扫描行，所以 i*row 会转移到第i列(如果认为最前的列为第0列的话)，
    //因为 node_list_row 在记录行号的时候加一了，所以要减一
    coeff = node_coeff[i*row + node_list_row(i) - 1]; 

    double ds;
    Eigen::MatrixXd B(3, 1);
    Eigen::MatrixXd A(3, 6);
    for(int16_t j = 1; j <= 10; j++)
    {
      ds = j * sample_s / 10;

      A << 1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),      pow(ds,5),
           0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),    5*pow(ds,4),
           0,   0,      2,              6*ds,           12*pow(ds,2),   20*pow(ds,3);

      B = A*coeff;

      path_point_mags.s = plan_start_mags.s + i * sample_s + ds;
      path_point_mags.l = B(0, 0);
      path_point_mags.dl = B(1, 0);
      path_point_mags.ddl = B(2, 0);
      path_point_mags.index = i*10 + j;     //没有坐标0
      dynamic_frenet.push_back(path_point_mags);
    }
  }
}

/*frenet转Cartesian
 *输入： dynamic_frenet           动态规划生成的frenet曲线
 *      referenceline_mags       参考线信息
 *输出： dynamic_path             转化为Cartesian坐标系的最终路径点信息
 */
void Dynamic_Plan::Frenet_trans_Cartesian(const std::vector<Dynamic_planning::Frenet_mags>& dynamic_frenet, const Dynamic_planning::Road_mags& referenceline_mags)
{
  dynamic_path.referenceline.header.frame_id = Frame_id;
  dynamic_path.referenceline.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamp;
  pose_stamp.header.frame_id = Frame_id;
  pose_stamp.header.stamp = ros::Time::now();

  visualization_msgs::Marker peak;
  peak.header.frame_id =  "planning_odom";
  peak.header.stamp = ros::Time::now();
  peak.ns = "points_and_line";
  peak.action = visualization_msgs::Marker::ADD;
  peak.pose.orientation.w = 1.0;
  peak.id = 0;
  peak.type = visualization_msgs::Marker::POINTS;
  peak.scale.x = 0.1;
  peak.scale.y = 0.1;
  peak.color.g = 1.0;
  peak.color.a = 1.0;
  /********************/


  uint16_t match_index = 0;
  double tor[2],nor[2];
  double ds;
  double project_point[2];//匹配点与投影点
  double project_theta;
  double project_kappa;

  dynamic_path.l.clear();
  for(uint16_t i = 0; i < dynamic_frenet.size(); i++)
  {
    //遍历寻找s对应的匹配点,一直到referenceline_mags.s大于dynamic_frenet.s就退出
    while(dynamic_frenet[i].s > referenceline_mags.s[match_index])
    {
      match_index++;
      if(match_index >= referenceline_mags.s.size()) break;
    }

    if(match_index >= referenceline_mags.s.size()) break;
    //求匹配点与投影点之间的ds
    ds = dynamic_frenet[i].s - referenceline_mags.s[match_index];
    //切向量
    tor[0] = cos(referenceline_mags.theta[match_index]); 
    tor[1] = sin(referenceline_mags.theta[match_index]); 
    //计算投影点信息
    project_point[0] = referenceline_mags.referenceline.poses[match_index].pose.position.x + ds * tor[0];
    project_point[1] = referenceline_mags.referenceline.poses[match_index].pose.position.y + ds * tor[1];
    project_theta = referenceline_mags.theta[match_index];//这里默认投影点的theta和kappa一样。以后在改
    project_kappa = referenceline_mags.k[match_index];
    //计算法；向量
    nor[0] = -sin(referenceline_mags.theta[match_index]); 
    nor[1] = cos(referenceline_mags.theta[match_index]); 

    pose_stamp.pose.position.x = project_point[0] + dynamic_frenet[i].l * nor[0];
    pose_stamp.pose.position.y = project_point[1] + dynamic_frenet[i].l * nor[1];

    // cout << "(" << pose_stamp.pose.position.x<<","
    // << pose_stamp.pose.position.y << ")"<<endl;

    dynamic_path.referenceline.poses.push_back(pose_stamp);
    dynamic_path.s.push_back(dynamic_frenet[i].s);
    dynamic_path.theta.push_back(referenceline_mags.theta[match_index]);
    dynamic_path.l.push_back(dynamic_frenet[i].l);

    if(i %10 == 0)
    {
      peak.points.push_back(pose_stamp.pose.position);
    }
  }
  
  // if(dynamic_state.goal_dynamic_flag == true && dynamic_state.pubilc_flag == false)
  // {
  //   dynamic_state.pubilc_flag = true;
  //   ROS_WARN("end_xy:");
  //   for(int16_t i = 0; i < dynamic_path.s.size(); i++)
  //   {
  //     cout<<"("<<dynamic_path.s[i]<<", "<<dynamic_path.l[i]<<", "<<dynamic_path.theta[i]<<", "<<dynamic_path.referenceline.poses[i].pose .position.x
  //     <<", "<<dynamic_path.referenceline.poses[i].pose .position.y<<")"<<endl;
  //   }
  // }

  dynamic_path.Calc_theta();
  referenceline_pub_.publish(dynamic_path.referenceline);
  // marker_pub.publish(peak);
}


/*计算两点间轨迹的代价
 *输入：  plan_start_mags           规划起点信息
 *       plan_end_mags             规划终点信息
 *       cost_*                    代价权重
 *输出：  cost                      代价
 */
double Dynamic_Plan::Calc_Neighbour_Cost(const Dynamic_planning::Frenet_mags& plan_start_mags, const Dynamic_planning::Frenet_mags& plan_end_mags, 
                           Eigen::MatrixXd& coeff, const uint16_t cost_ref, const uint16_t cost_dl, const uint16_t cost_ddl)
{
  double cost = 0;
  double start_l = plan_start_mags.l;
  double start_dl = plan_start_mags.dl;
  double start_ddl = plan_start_mags.ddl;

  double end_l = plan_end_mags.l;
  double end_dl = plan_end_mags.dl;
  double end_ddl = plan_end_mags.ddl;

  //为了简化运算,避免出现5次方数太大，将起始点的s偏移到0点，逆推时在将坐标偏移回去
  double start_s = 0;
  double end_s = plan_end_mags.s - plan_start_mags.s;

  Calc_Quintc_Coeffient(start_l, start_dl, start_ddl,
                        end_l, end_dl, end_ddl,
                        start_s, end_s, coeff);

  double ds;
  Eigen::MatrixXd B(3, 1);
  Eigen::MatrixXd A(3, 6);
  for(int16_t i = 0; i < 10; i++)
  {
    ds = i * (plan_end_mags.s - plan_start_mags.s) / 10;

    A << 1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),      pow(ds,5),
         0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),    5*pow(ds,4),
         0,   0,      2,              6*ds,           12*pow(ds,2),   20*pow(ds,3);

    B = A*coeff;

    cost += cost_ref*B(0,0)*B(0,0)  + cost_dl*B(1,0)*B(1,0) + cost_ddl*B(2,0)*B(2,0);
    cost = cost + obs_.calc_obstacle_cost(plan_start_mags.s + ds, B(0,0));
  }

  return cost;
}

/*计算五次多项式
 *输入：    start                     规划起点信息
 *         end                       规划终点信息
 * 输出：   coeff                     五次多项式系数矩阵
 */
void Dynamic_Plan::Calc_Quintc_Coeffient(const double start_l, const double start_dl, const double start_ddl,
                           const double end_l, const double end_dl, const double end_ddl,
                           const double start_s, const double end_s, Eigen::MatrixXd& coeff)
{
  double start_s2 = start_s * start_s;
  double start_s3 = start_s2 * start_s;
  double start_s4 = start_s3 * start_s;
  double start_s5 = start_s4 * start_s;

  double end_s2 = end_s * end_s;
  double end_s3 = end_s2 * end_s;
  double end_s4 = end_s3 * end_s;
  double end_s5 = end_s4 * end_s;

  Eigen::MatrixXd A(6,6);
  A << 1, start_s,  start_s2,   start_s3,   start_s4,     start_s5,
       0, 1,        2*start_s,  3*start_s2, 4*start_s3,   5*start_s4,
       0, 0,        2,          6*start_s,  12*start_s2,  20*start_s3,
       1, end_s,    end_s2,     end_s3,     end_s4,       end_s5,
       0, 1,        2*end_s,    3*end_s2,   4*end_s3,     5*end_s4,
       0, 0,        2,          6*end_s,    12*end_s2,    20*end_s3;

  Eigen::MatrixXd B(6,1);
  B << start_l,
       start_dl,
       start_ddl,
       end_l,
       end_dl,
       end_ddl;

 coeff =  A.inverse() * B;
}




#include "global/global_planning.h"
#include "dynamic/dynamic_path.h"

//获取起点
void Global_Plan::start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    global_date::my_mutex.lock();
    global_date::gloLine_build_ = false;
    global_date::my_mutex.unlock();

    ROS_WARN("start: ");
    std::cout<<"("<<msg->pose.pose.position.x<<
               ","<<msg->pose.pose.position.y<<","
               <<tf2::getYaw(msg->pose.pose.orientation)*180/3.1415<<")"<<std::endl;
    start_pose_.position.x = msg->pose.pose.position.x;
    start_pose_.position.y = msg->pose.pose.position.y;
    start_pose_.orientation = msg->pose.pose.orientation;
}


/*获取终点
 *获取起点到终点的的原始参考线*/
void Global_Plan::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    ROS_WARN("goal: ");
    std::cout<<"("<<msg->pose.position.x<<
               ","<<msg->pose.position.y<<","<<
               tf2::getYaw(msg->pose.orientation)*180/3.1415<<")"<<std::endl;
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    goal_pose_.orientation = msg->pose.orientation;

    global_date::my_mutex.lock();
    hdmap_build_.search_ShortesrPath(start_pose_, goal_pose_, hdmap_build_.origin_point_);
    global_date::gloLine_build_ = true;
    global_date::my_mutex.unlock();
    ROS_WARN("global Line have build");
    // this->referenceline_pub_.publish(hdmap_build_.Global_Path_.referenceline);
}



/*将GPS下的定位转换到直角坐标系
 *这里获取的定位信息是偏移后的
 *如果要用于规划，记得减去偏移量*/
void Global_Plan::GPSToMGRS(Eigen::Vector3d& gps, Eigen::Vector3d& mgrs)
{
  lanelet::GPSPoint lanelet2_gps;
  lanelet2_gps.lat = gps(0);
  lanelet2_gps.lon = gps(1);
  mgrs(0) = Hdmap_Build::projector.forward(lanelet2_gps).x();
  mgrs(1) = Hdmap_Build::projector.forward(lanelet2_gps).y();
  mgrs(2) = 0;
}


void Global_Plan::CurGPSPtCallBack(const sleipnir_msgs::sensorgps::ConstPtr& msg)
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
  start_pose_.position.x = gp_start_pt_(0) - hdmap_build_.origin_point_(0);
  start_pose_.position.y = gp_start_pt_(1) - hdmap_build_.origin_point_(1);
  start_pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, heading);
}


Global_Plan::~Global_Plan()
{
  if(generate_glo_thread_ != NULL)
  {
    delete generate_glo_thread_;
    generate_glo_thread_ = NULL;
  }
}

Global_Plan::Global_Plan()
{
  ros::NodeHandle n_gp;
  n_gp.param("simulation_model", this->simulation_flag, true);

  if(simulation_flag == true)
  {
    this->start_pose_sub_ = n_gp.subscribe("/initialpose", 10, &Global_Plan::start_pose_call_back, this);
  }
  else
  {
    this->start_pose_sub_ = 
      n_gp.subscribe("/localization/fusion_position", 10, &Global_Plan::CurGPSPtCallBack, this);
  }

  this->goal_pose_sub_ = 
    n_gp.subscribe("/move_base_simple/goal", 10, &Global_Plan::goal_pose_call_back, this);
  this->referenceline_pub_ = 
    n_gp.advertise<nav_msgs::Path>("ReferenceLine_CenterPoint", 10);

  generate_glo_thread_ = new boost::thread(boost::bind(&Global_Plan::global_thread_worker, this));
}


/***在参考线上寻找匹配点
 * 输入： search_pose               需要匹配的位置
 *       referenceline             参考线数组
 *       start_index,end_index     为减少计算，选择数组范围，可以从上一时刻的匹配点来预测这一时刻的匹配点
 * 输出： match_index               在参考线referenceline上找到的匹配点位置
 * ***/
int16_t Global_Plan::Search_Match_Point(const double& search_pose_x, const double& search_pose_y,
                           const nav_msgs::Path referenceline,
                           const int16_t start_index, const int16_t end_index)
{
  double point_distance,last_distance;
  double min_distance;
  int16_t count_num = 0;

  int16_t match_index = start_index;
  last_distance = min_distance = sqrt(pow(referenceline.poses[start_index].pose.position.x - search_pose_x, 2) +
                                      pow(referenceline.poses[start_index].pose.position.y - search_pose_y, 2));
  //开始遍历比较距离大小，找到最近点
  for(int16_t i = start_index + 1; i <= end_index; i++)
  {
    point_distance = sqrt(pow(referenceline.poses[i].pose.position.x - search_pose_x, 2) +
                          pow(referenceline.poses[i].pose.position.y - search_pose_y, 2));
    if(point_distance < min_distance)
    {
      min_distance = point_distance;
      match_index = i;
    }
    else if(last_distance < point_distance) //比较上一个点的距离和当前点的距离。是不是越来越远
    {
      count_num++; 
      if(count_num >= 10) break; //越来越远，连续10次递增。跳出循环
    }
    else if(last_distance > point_distance) //越来越近，计数清零
    {
      count_num = 0;
    }
    last_distance = point_distance;
  }

  return match_index;
}


void Global_Plan::global_thread_worker()
{
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}



// /*计算三次多项式
//  *输入：    start                     规划起点信息
//  *         end                       规划终点信息
//  * 输出：   coeff                     三次多项式系数矩阵
//  */
// void Calc_Three_Coeffient(const double start_x, const double start_dx, const double start_ddx,
//                           const double end_x, const double end_dx,
//                           const double dT, Eigen::MatrixXd& coeff)
// {
//   double dT2 = dT * dT;
//   double dT3 = dT2 * dT;
//   double dT4 = dT3 * dT;

//   Eigen::MatrixXd A(5,5);
//   A << 1, 0,        0,          0,        0,
//        0, 1,        0,          0,        0,
//        0, 0,        1,          0,        0,
//        1, dT,       dT2,        dT3,      dT4,
//        0, 1,        2*dT,       3*dT2,    4*dT3;

//   Eigen::MatrixXd B(5,1);
//   B << start_x,
//        start_dx,
//        start_ddx,
//        end_x,
//        end_dx;

//   coeff =  A.inverse() * B;
// }


// void Smooth_Path_array(const int& start_index, const int& end_index,
//                        const geometry_msgs::PoseArray& stores_path_array, Dynamic_planning::Road_mags& Original_Path)
// {
//   double start_x,start_dx,start_ddx;
//   double start_y,start_dy,start_ddy;

//   double end_x,end_dx;
//   double end_y,end_dy;

//   double Vc = 2.0;
//   const double dT = 5.0/Vc;//默认速度2.0，默认距离5.0;

//   double theta;

//   /***参考线初始化***/
//   Original_Path.clear_data();
//   Original_Path.referenceline.header.frame_id = Frame_id;
//   Original_Path.referenceline.header.stamp = ros::Time::now();
//   /************/

//   /***************************/
//   geometry_msgs::PoseStamped pose_stamp;
//   pose_stamp.header.frame_id = Frame_id;
//   pose_stamp.header.stamp = ros::Time::now();

//   visualization_msgs::Marker peak;
//   peak.header.frame_id =  "planning_odom";
//   peak.header.stamp = ros::Time::now();
//   peak.ns = "points_and_line";
//   peak.action = visualization_msgs::Marker::ADD;
//   peak.pose.orientation.w = 1.0;
//   peak.id = 0;
//   peak.type = visualization_msgs::Marker::POINTS;
//   peak.scale.x = 0.1;
//   peak.scale.y = 0.1;
//   //红点
//   peak.color.g = 1.0;
//   peak.color.a = 1.0;
//   /********************/




//   // if(start_index < end_index)
//   // {
//   //   theta = atan2((stores_path_array.poses[start_index+3].position.y - stores_path_array.poses[start_index+2].position.y),
//   //                 (stores_path_array.poses[start_index+3].position.x - stores_path_array.poses[start_index+2].position.x));

//   //   start_x = stores_path_array.poses[start_index+2].position.x;
//   //   start_dx = Vc * cos(theta);
//   //   start_ddx = 0;

//   //   start_y = stores_path_array.poses[start_index+2].position.y;
//   //   start_dy = Vc * sin(theta);
//   //   start_ddy = 0;

//   //   theta = atan2((stores_path_array.poses[start_index+4].position.y - stores_path_array.poses[start_index+2].position.y),
//   //                 (stores_path_array.poses[start_index+4].position.x - stores_path_array.poses[start_index+2].position.x));

//   //   end_x = stores_path_array.poses[start_index + 3].position.x;
//   //   end_dx = Vc * cos(theta);

//   //   end_y = stores_path_array.poses[start_index + 3].position.y;
//   //   end_dy = Vc * sin(theta);


//   //     Calc_Three_Coeffient(start_x, start_dx, start_ddx, end_x, end_dx, dT, coeff_x);
//   //     Calc_Three_Coeffient(start_y, start_dy, start_ddy, end_y, end_dy, dT, coeff_y);


//   //     for(uint16_t j = 0; j <= 10; j++)
//   //     {
//   //       double ds = j * dT/10;
//   //       A << 1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),
//   //            0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),
//   //            0,   0,      2,              6*ds,           12*pow(ds,2);

//   //       B_x = A * coeff_x;
//   //       B_y = A * coeff_y;

//   //       theta = atan2(B_y(1, 0), B_x(1, 0));

//   //       pose_stamp.pose.position.x = B_x(0, 0);
//   //       pose_stamp.pose.position.y = B_y(0, 0);
//   //       pose_stamp.pose.position.z = 0;

//   //       if(j == 0 || j == 10)
//   //       {
//   //         peak.points.push_back(pose_stamp.pose.position);
//   //       }

//   //       Original_Path.referenceline.poses.push_back(pose_stamp);
//   //       Original_Path.theta.push_back(theta);
//   //     }
//   // }



//   if(start_index < end_index)
//   {
//     theta = atan2((stores_path_array.poses[start_index+1].position.y - stores_path_array.poses[start_index].position.y),
//                   (stores_path_array.poses[start_index+1].position.x - stores_path_array.poses[start_index].position.x));

//     start_x = stores_path_array.poses[start_index].position.x;
//     start_dx = Vc * cos(theta);
//     start_ddx = 0;

//     start_y = stores_path_array.poses[start_index].position.y;
//     start_dy = Vc * sin(theta);
//     start_ddy = 0;

//     for(uint16_t i = start_index; i < start_index + 5; i++)
//     {
//         Eigen::MatrixXd coeff_x(5, 1);
//         Eigen::MatrixXd coeff_y(5, 1);
//         Eigen::MatrixXd B_x(3, 1), B_y(3, 1);
//         Eigen::MatrixXd A(3, 5);
        
//       end_x = stores_path_array.poses[i+1].position.x;
//       end_y = stores_path_array.poses[i+1].position.y;
      
//       theta = atan2((stores_path_array.poses[i+2].position.y - stores_path_array.poses[i].position.y),
//                     (stores_path_array.poses[i+2].position.x - stores_path_array.poses[i].position.x));

//       end_dx = Vc * cos(theta);
//       end_dy = Vc * sin(theta);

//       Calc_Three_Coeffient(start_x, start_dx, start_ddx, end_x, end_dx, dT, coeff_x);
//       Calc_Three_Coeffient(start_y, start_dy, start_ddy, end_y, end_dy, dT, coeff_y);

//       for(uint16_t j = 0; j <= 10; j++)
//       {
//         double ds = j * dT/10;
//         A << 1,   ds,     pow(ds,2),      pow(ds,3),      pow(ds,4),
//              0,   1,      2*ds,           3*pow(ds,2),    4*pow(ds,3),
//              0,   0,      2,              6*ds,           12*pow(ds,2);

//         B_x = A * coeff_x;
//         B_y = A * coeff_y;

//         if(j == 10)
//         {
//           start_x = B_x(0, 0);
//           start_dx = B_x(1, 0);
//           start_ddx = B_x(2, 0);

//           start_y = B_y(0, 0);
//           start_dy = B_y(1, 0);
//           start_ddy = B_y(2, 0);
//           break;
//         }

//         theta = atan2(B_y(1, 0), B_x(1, 0));

//         pose_stamp.pose.position.x = B_x(0, 0);
//         pose_stamp.pose.position.y = B_y(0, 0);
//         pose_stamp.pose.position.z = 0;

//         if(j == 0)
//         {
//           peak.points.push_back(pose_stamp.pose.position);
//         }

//         Original_Path.referenceline.poses.push_back(pose_stamp);
//         Original_Path.theta.push_back(theta);
//       }
//     }

//   }

//   marker_pub.publish(peak);
// }

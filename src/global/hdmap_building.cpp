#include "global/hdmap_building.h"
#include "global/global_planning.h"
#include "dynamic/dynamic_path.h"
#include "obstacle/obstacle.h"
// #include "xmldoc/XmlDocument.h"

Dynamic_planning::Road_mags Hdmap_Build::Global_Path_;
Eigen::Vector3d Hdmap_Build::origin_point_;
lanelet::projection::MGRSProjector Hdmap_Build::projector; // MGRS,  Projection：提供全球地理坐标系到局部平面坐标系的准换


Hdmap_Build::Hdmap_Build()
{
    ros::NodeHandle n_hd;
    n_hd.param<std::string>("hdmap_file_map", this->hdmap_file_map, "x");
    
    this->Calc_origin_point();//计算坐标偏移

    this->routingGraph = this->InitHdMap(this->hd_map);//加载地图
}


/*
 *加载路由图
 */
lanelet::routing::RoutingGraphUPtr Hdmap_Build::InitHdMap(lanelet::LaneletMapPtr& map)
{
    //加载地图文件，并投影将经纬度改为xy坐标系
  map = load(this->hdmap_file_map, this->projector);

  //建立交通规则
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::traffic_rules::TrafficRulesPtr pedestrainRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Pedestrian);

  //导入交通规则
  lanelet::routing::RoutingGraphUPtr routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);

  return routingGraph;
}


/*计算坐标偏移量*/
void Hdmap_Build::Calc_origin_point()
{
  // shared_ptr<XmlDomDocument> doc = make_shared<XmlDomDocument>(dynamic_state.hdmap_file_map.c_str());
  //----------------------获得MGRS原点------------------------//
  // if (doc)
  // {
  //   for (int i = 0; i < doc->getChildCount("node", 0, "tag"); i++)
  //   {
  //     if (doc->getChildAttribute("node", 0, "tag", i, "k") == "local_x")
  //     {
  //       origin_point_(0) = atof(doc->getChildAttribute("node", 0, "tag", i, "v").c_str());
  //     }
  //     if (doc->getChildAttribute("node", 0, "tag", i, "k") == "local_y")
  //     {
  //       origin_point_(1) = atof(doc->getChildAttribute("node", 0, "tag", i, "v").c_str());
  //     }
  //   }
  //   origin_point_(2) = 0;
  // }
  this->origin_point_(0) = 45030.4;
  this->origin_point_(1) = 49558.2;
  this->origin_point_(2) = 0;
}



/*
 *计算起点到终点的最短路径
 *输入： start_pose         获取的起点坐标
 *      goal_pose          获取的终点坐标
 *      offset_point       坐标偏移量
 *输出： Original_Path      原始路劲信息
*/
void Hdmap_Build::search_ShortesrPath(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose, 
                         const Eigen::Vector3d& offset_point)
{
  //加上坐标偏移量
  geometry_msgs::Pose start_pose_offset;
  geometry_msgs::Pose goal_pose_offset;
  start_pose_offset.position.x = start_pose.position.x + offset_point(0);
  start_pose_offset.position.y = start_pose.position.y + offset_point(1);
  start_pose_offset.position.z = start_pose.position.z + offset_point(2);
  goal_pose_offset.position.x = goal_pose.position.x + offset_point(0);
  goal_pose_offset.position.y = goal_pose.position.y + offset_point(1);
  goal_pose_offset.position.z = goal_pose.position.z + offset_point(2);

  lanelet::Lanelet start_lanelet;
  lanelet::Lanelet goal_lanelet;
  if (Have_nearest_linelet(start_pose_offset, goal_pose_offset, start_lanelet, goal_lanelet, this->hd_map) == true)
  {
    //寻找最短路径
    lanelet::Optional<lanelet::routing::Route> route = this->routingGraph->getRoute(start_lanelet, goal_lanelet);
    lanelet::routing::LaneletPath shortestPath = route->shortestPath();
    
    //输出全局最优路径的位置数组
    Generate_Global_Routing(shortestPath, this->stores_path_array_, offset_point);
    //由起点终点生成原始的参考线
    Generate_Original_Referenceline(start_pose, goal_pose);
  }
}


/*
 *在地图上寻找离search_pose最近的 lanelet
 *输入：   search_pose        被寻找的点
 *        hpmap               地图
 *        distance_cost       距离阈值,默认距离阈值1
 * 输出：  closest_lanelet    最近的lanelet
 * */
bool Hdmap_Build::Get_ClosestLanelet(const geometry_msgs::Pose& search_pose, lanelet::Lanelet* closest_lanelet,
                        lanelet::LaneletMapPtr& hpmap, double distance_cost = 1.0)
{
  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet = 
      lanelet::geometry::findNearest(hpmap->laneletLayer, search_point, 1);

  //是否找到了lanelet
  if(nearest_lanelet.empty())
  {
    std::cout << "Failed to find the closest lane!" << std::endl;
    return false;
  }
  else if(nearest_lanelet.front().first > distance_cost)
  {
    std::cout << "Lanelet is too far away" << std::endl;
    std::cout <<"distance is "<<nearest_lanelet.front().first<<std::endl;
    std::cout << "Lanelet id: " << nearest_lanelet.front().second.id()<<std::endl;;
    return false;
  }
  else
  {
    *closest_lanelet = nearest_lanelet.front().second;
    return true;
  }

}


/*
 *寻找起点与终点的lanelet
 *输入： start_pose，goal_pose        起点与终点的位置
 *      hpmap                         地图
 *输出： start_lanelet，goal_lanelet  起点与终点的lanelet
 */
bool Hdmap_Build::Have_nearest_linelet(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
                          lanelet::Lanelet& start_lanelet, lanelet::Lanelet& goal_lanelet,
                          lanelet::LaneletMapPtr& hpmap)
{
  bool Have_start_lanelet = Get_ClosestLanelet(start_pose, &start_lanelet, hpmap);
  bool Have_goal_lanelet = Get_ClosestLanelet(goal_pose, &goal_lanelet, hpmap);

  if (Have_start_lanelet == false || Have_goal_lanelet == false)
  {
    std::cout << "No lanelet!" << std::endl;
    return false;
  }

  return true;
}









/*
 *输出全局规划路线的数组stores_path_array_
 *输入：  shortestPath        路由图最短路径
 *       offset_point        坐标偏移量
 *输出:   stores_path_array_  全局规划路线的数组
 */
void Hdmap_Build::Generate_Global_Routing(const lanelet::routing::LaneletPath& shortestPath,
                             geometry_msgs::PoseArray& stores_path_array_, const Eigen::Vector3d& offset_point)
{
  lanelet::ConstLanelet a_Lanelet1;
  lanelet::ConstLineString3d center_ls1;
  geometry_msgs::Pose point_posess;

  stores_path_array_.poses.clear();//清除最短路劲信息

  for(int i = 0; i < shortestPath.size(); i++)
  {
    a_Lanelet1 = shortestPath[i];
    center_ls1 = a_Lanelet1.centerline();

    for(int j = 0; j < center_ls1.size(); j++)
    {
      //在路由图上获取路径时，是在偏移后的坐标
      //在这里偏移回去
      point_posess.position.x = center_ls1[j].x() - offset_point(0);
      point_posess.position.y = center_ls1[j].y() - offset_point(1);
      point_posess.position.z = 0;
      stores_path_array_.poses.push_back(point_posess);
    }
  }
}



/*生成参考线
 *输入： start_pose，goal_pose        起始点坐标
        stores_path_array_          路由图的最优路径
 *输出   referenceline               插值后的参考线*/
void Hdmap_Build::Generate_Original_Referenceline(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose)
{
  double point_distance;
  double min_distance;
  int start_index = 0;
  int goal_index = 0;
  //遍历所有路径点，寻找当前车辆的位置匹配点，即与起点最近的路径点
  min_distance = sqrt((start_pose.position.x - stores_path_array_.poses[0].position.x)*(start_pose.position.x - stores_path_array_.poses[0].position.x)
                      + (start_pose.position.y - stores_path_array_.poses[0].position.y)*(start_pose.position.y - stores_path_array_.poses[0].position.y));
  for(int i=1; i < stores_path_array_.poses.size(); i++)
  {
    point_distance = sqrt((start_pose.position.x - stores_path_array_.poses[i].position.x)*(start_pose.position.x - stores_path_array_.poses[i].position.x)
                      + (start_pose.position.y - stores_path_array_.poses[i].position.y)*(start_pose.position.y - stores_path_array_.poses[i].position.y));
    
    if (point_distance < min_distance)
    {
      min_distance = point_distance;
      start_index = i;
    }
  }
  //寻找与终点最佳的路径点
  min_distance = sqrt((goal_pose.position.x - stores_path_array_.poses[0].position.x)*(goal_pose.position.x - stores_path_array_.poses[0].position.x)
                      + (goal_pose.position.y - stores_path_array_.poses[0].position.y)*(goal_pose.position.y - stores_path_array_.poses[0].position.y));
  for(int i=1; i < stores_path_array_.poses.size(); i++)
  {
    point_distance = sqrt((goal_pose.position.x - stores_path_array_.poses[i].position.x)*(goal_pose.position.x - stores_path_array_.poses[i].position.x)
                      + (goal_pose.position.y - stores_path_array_.poses[i].position.y)*(goal_pose.position.y - stores_path_array_.poses[i].position.y));
    
    if (point_distance < min_distance)
    {
      min_distance = point_distance;
      goal_index = i;
    }
  }

  if(start_index > goal_index)
  {
    if(start_index < stores_path_array_.poses.size()-1)
    {
      start_index++;
    }
    if(goal_index > 0)
    {
      goal_index--;
    }
  }
  else
  {
    if(goal_index < stores_path_array_.poses.size()-1)
    {
      goal_index++;
    }
    if(start_index > 0)
    {
      start_index--;
    }
  }

  // find_obj_road(start_index, goal_index);
  Insert_Point(start_index, goal_index);

  // Smooth_Path_array(start_index, goal_index, stores_path_array_, Original_Path);
}

/*参考线插值，让参考线更密集
 *输入：   start_index         起点的匹配点坐标
          end_index           终点的匹配点坐标
          stores_path_array_  路径点的坐标数组
          spacing_distance    路径点的最小距离，默认0.5m
 *输出：   referenceline       插值后的参考线
          输出的参考线由小到大并定是   从起点到终点
 */
void Hdmap_Build::Insert_Point(const int& start_index, const int& end_index,double spacing_distance, bool nearly)
{
  /***参考线初始化***/
  Global_Path_.clear_data();
  Global_Path_.referenceline.header.frame_id = Frame_id;
  Global_Path_.referenceline.header.stamp = ros::Time::now();
  /************/

  geometry_msgs::PoseStamped pose_stamp;
  pose_stamp.header.frame_id = Frame_id;
  pose_stamp.header.stamp = ros::Time::now();

  /********路径点**********/
  visualization_msgs::Marker points;
  points.header.frame_id =  "planning_odom";
  points.header.stamp = ros::Time::now();
  points.ns = "points_and_line";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  //红点
  points.color.r = 1.0;
  points.color.a = 1.0;

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

  double insert_distance;
  double remainder_distance;
  double sin_thela, cos_thela;
  int16_t insert_num;
  if(start_index < end_index)
  {
    double kappa;
    //开始遍历所有的原始路径点
    for(int16_t i = start_index; i < end_index; i++)
    {
      if(i <= end_index - 2)
      {
        kappa = Calc_kappa(stores_path_array_.poses[i], stores_path_array_.poses[i+1], stores_path_array_.poses[i+2]);
      }
      else
      {
        kappa = Calc_kappa(stores_path_array_.poses[end_index-2], stores_path_array_.poses[end_index-1], stores_path_array_.poses[end_index]);
      }
      //计算两点之间的距离、需要插值的个数、距离余数、方向正余弦
      insert_distance = sqrt( pow((stores_path_array_.poses[i].position.x - stores_path_array_.poses[i+1].position.x), 2) + 
                              pow((stores_path_array_.poses[i].position.y - stores_path_array_.poses[i+1].position.y), 2) );
      insert_num = (int16_t)(insert_distance / spacing_distance);
      remainder_distance = insert_distance - spacing_distance * insert_num;
      sin_thela = (stores_path_array_.poses[i+1].position.y - stores_path_array_.poses[i].position.y) / insert_distance;
      cos_thela = (stores_path_array_.poses[i+1].position.x - stores_path_array_.poses[i].position.x) / insert_distance;
      //开始插值
      for(int16_t j = 0; j < insert_num; j++)
      {
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x + j * cos_thela * spacing_distance;
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y + j * sin_thela * spacing_distance;
        pose_stamp.pose.position.z = 0;
        if(j == 0)
        {
          peak.points.push_back(pose_stamp.pose.position);
        }
        else
        {
          points.points.push_back(pose_stamp.pose.position);
        }
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        Global_Path_.k.push_back(kappa);
      }
      //判断距离余数的大小，如果距离余数较大就插值，否则不插值
      if(spacing_distance / remainder_distance < 2.5 && nearly == true)  //距离足够长，插值进去
      {
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x + insert_num * cos_thela * spacing_distance;
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y + insert_num * sin_thela * spacing_distance;
        pose_stamp.pose.position.z = 0;
        points.points.push_back(pose_stamp.pose.position);
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        Global_Path_.k.push_back(kappa);
      }
    }
    pose_stamp.pose.position.x = stores_path_array_.poses[end_index].position.x;
    pose_stamp.pose.position.y = stores_path_array_.poses[end_index].position.y;
    pose_stamp.pose.position.z = 0;
    peak.points.push_back(pose_stamp.pose.position);
    Global_Path_.referenceline.poses.push_back(pose_stamp);
    Global_Path_.k.push_back(kappa);
  }
  else
  {
    double kappa;
    //开始遍历所有的原始路径点，反向记录，保证起点在前，终点在后
    for(int16_t i = start_index; i > end_index; i--)
    {
      if(i >= end_index+2)
      {
        kappa = Calc_kappa(stores_path_array_.poses[i], stores_path_array_.poses[i-1], stores_path_array_.poses[i-2]);
      }
      else
      {
        kappa = Calc_kappa(stores_path_array_.poses[end_index+2], stores_path_array_.poses[end_index+1], stores_path_array_.poses[end_index]);
      }
      //计算两点之间的距离、需要插值的个数、距离余数、方向正余弦
      insert_distance = sqrt( pow((stores_path_array_.poses[i].position.x - stores_path_array_.poses[i-1].position.x), 2) + 
                              pow((stores_path_array_.poses[i].position.y - stores_path_array_.poses[i-1].position.y), 2) );
      insert_num = (int16_t)(insert_distance / spacing_distance);
      remainder_distance = insert_distance - spacing_distance * insert_num;
      sin_thela = (stores_path_array_.poses[i-1].position.y - stores_path_array_.poses[i].position.y) / insert_distance;
      cos_thela = (stores_path_array_.poses[i-1].position.x - stores_path_array_.poses[i].position.x) / insert_distance;
      //开始插值
      for(int16_t j = 0; j < insert_num; j++)
      {
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x + j * cos_thela * spacing_distance;
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y + j * sin_thela * spacing_distance;
        pose_stamp.pose.position.z = 0;
        if(j == 0) //初始未插值的点显示为绿色，方便观看插值效果
        {
          peak.points.push_back(pose_stamp.pose.position);
        }
        else
        {
          points.points.push_back(pose_stamp.pose.position);
        }
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        Global_Path_.k.push_back(kappa);
      }
      //判断距离余数的大小，如果距离余数较大就插值，否则不插值
      if(spacing_distance / remainder_distance < 2.5 && nearly == true)  //距离足够长，插值进去
      {
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x + insert_num * cos_thela * spacing_distance;
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y + insert_num * sin_thela * spacing_distance;
        pose_stamp.pose.position.z = 0;
        points.points.push_back(pose_stamp.pose.position);
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        Global_Path_.k.push_back(kappa);
      }

    }
    pose_stamp.pose.position.x = stores_path_array_.poses[end_index].position.x;
    pose_stamp.pose.position.y = stores_path_array_.poses[end_index].position.y;
    pose_stamp.pose.position.z = 0;
    peak.points.push_back(pose_stamp.pose.position);
    Global_Path_.referenceline.poses.push_back(pose_stamp);
    Global_Path_.k.push_back(kappa);
  }
  Global_Path_.Calc_Path_msgs();
  // ROS_WARN("Original_Path:");
  // for(int16_t i = 0; i < Original_Path.theta.size(); i++)
  // {

  //   cout<<"("<<Original_Path.s[i]<<", "<<Original_Path.theta[i]<<", "<<Original_Path.referenceline.poses[i].pose.position.x<<", "
  //   <<Original_Path.referenceline.poses[i].pose.position.y<<")"<<endl;
  // }
  // marker_pub.publish(peak);
}

/*计算k*/
double Hdmap_Build::Calc_kappa(const geometry_msgs::Pose& point1, const geometry_msgs::Pose& point2, const geometry_msgs::Pose& point3)
{
  double dx[2], dy[2];
  dx[0] = point2.position.x - point1.position.x;
  dx[1] = point3.position.x - point2.position.x;

  dy[0] = point2.position.x - point1.position.y;
  dy[1] = point3.position.y - point2.position.y;

  double dtheta = atan2(dy[1], dx[1]) - atan2(dy[0], dx[0]);

  double ds = sqrt(dy[0]*dy[0] + dx[0]*dx[0]);

  return dtheta/ds;
}






// void Generate_final_Referenceline(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
//                                   const nav_msgs::Path& referenceline)
// {
//     /********路径点**********/
//   visualization_msgs::Marker points;
//   points.header.frame_id =  "planning_odom";
//   points.header.stamp = ros::Time::now();
//   points.ns = "points_and_line";
//   points.action = visualization_msgs::Marker::ADD;
//   points.pose.orientation.w = 1.0;
//   points.id = 0;
//   points.type = visualization_msgs::Marker::POINTS;
//   points.scale.x = 0.1;
//   points.scale.y = 0.1;
//   //红点
//   points.color.r = 1.0;
//   points.color.a = 1.0;
//   /*********************/

//   nav_msgs::Path some_referenceline_;
//   referenceline_.poses.clear();

//   referenceline_.header.frame_id = Frame_id;
//   referenceline_.header.stamp = ros::Time::now();

//   int16_t carindex1 = Search_Match_Point(start_pose.position.x, start_pose.position.y, referenceline, 0, referenceline.poses.size()-1);


//   int16_t startindex = Min_Cost_Path(carindex1+10, carindex1+30, start_pose, carindex1, referenceline, some_referenceline_, start_direction);
//   referenceline_.poses =  some_referenceline_.poses;

//   int16_t carindex2 = Search_Match_Point(goal_pose.position.x, goal_pose.position.y, referenceline, 0, referenceline.poses.size()-1);

//   some_referenceline_.poses.clear();
//   int16_t goalindex = Min_Cost_Path(carindex2-30, carindex2-10, goal_pose, carindex2, referenceline, some_referenceline_, goal_direction);

//   for(int16_t i = carindex1+startindex+11; i < carindex2 - 30 + goalindex ; i++)
//   {
//     referenceline_.poses.push_back(referenceline.poses[i]);
//   }
//   for(int16_t i = some_referenceline_.poses.size()-1; i >=0 ; i--)
//   {
//     referenceline_.poses.push_back(some_referenceline_.poses[i]);
//   }

//   for(int16_t i = 0; i < referenceline_.poses.size();i++)
//   {
//     points.points.push_back(referenceline_.poses[i].pose.position);
//   }

//   referenceline_pub.publish(referenceline_);
//   // marker_pub.publish(points);
// }



void Hdmap_Build::find_obj_road(int start_index, int goal_index)
{
  /***参考线初始化***/
  Global_Path_.clear_data();
  Global_Path_.referenceline.header.frame_id = Frame_id;
  Global_Path_.referenceline.header.stamp = ros::Time::now();
  /************/

  geometry_msgs::PoseStamped pose_stamp;
  pose_stamp.header.frame_id = Frame_id;
  pose_stamp.header.stamp = ros::Time::now();
  double theta;
  double nor[2];

  if(start_index > goal_index)
  {
    for(int i = start_index; i >= goal_index; i--)
    {
        if(i == start_index)
          theta = atan2((stores_path_array_.poses[i-1].position.y - stores_path_array_.poses[i].position.y),
                        (stores_path_array_.poses[i-1].position.x - stores_path_array_.poses[i].position.x));
        else
          theta = atan2((stores_path_array_.poses[i].position.y - stores_path_array_.poses[i+1].position.y),
                        (stores_path_array_.poses[i].position.x - stores_path_array_.poses[i+1].position.x));

        nor[0] = -sin(theta);
        nor[1] = cos(theta);
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x - nor[0];
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y - nor[1];
        pose_stamp.pose.position.z = 0;
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        std::cout<<"("<<pose_stamp.pose.position.x<<","<<pose_stamp.pose.position.y<<","<<theta<<")"<<std::endl;
    }
  }
  else
  {
    for(int i = start_index; i <= start_index; i++)
    {
        if(i == start_index)
          theta = atan2((stores_path_array_.poses[i+1].position.y - stores_path_array_.poses[i].position.y),
                        (stores_path_array_.poses[i+1].position.x - stores_path_array_.poses[i].position.x));
        else
          theta = atan2((stores_path_array_.poses[i].position.y - stores_path_array_.poses[i-1].position.y),
                        (stores_path_array_.poses[i].position.x - stores_path_array_.poses[i-1].position.x));

        nor[0] = -sin(theta);
        nor[1] = cos(theta);
        pose_stamp.pose.position.x = stores_path_array_.poses[i].position.x - nor[0];
        pose_stamp.pose.position.y = stores_path_array_.poses[i].position.y - nor[1];
        pose_stamp.pose.position.z = 0;
        Global_Path_.referenceline.poses.push_back(pose_stamp);
        std::cout<<"("<<pose_stamp.pose.position.x<<","<<pose_stamp.pose.position.y<<","<<theta<<")"<<std::endl;
    }
  }
}

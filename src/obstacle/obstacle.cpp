#include "obstacle/obstacle.h"
#include "global/global_planning.h"


std::vector<Obstacle> ObsProj::All_obstacle;

ObsProj::ObsProj()
{
  ros::NodeHandle n_obs;
  n_obs.param("simulation_model", this->simulation_flag, true);

  if(this->simulation_flag == true)
    this->marker_pub_ = n_obs.advertise<visualization_msgs::Marker>("/marker",10);
}

void ObsProj::Get_globline()
{
    global_date::my_mutex.lock();
    this->global_reference.cope_data(Hdmap_Build::Global_Path_);
    global_date::my_mutex.unlock();
}


void ObsProj::Set_Obstacle()
{
  global_date::obj_mutex.lock();
  if(this->simulation_flag == true)
  {
    double array[][2] = 
    {
      {-234.2, -59.0},
      {-224.7, -55.7},
      {-255.2, -67.8},
      {-275.8, -81.5},
      { 0x00,   0x00}     //结束标识符
    };

    All_obstacle.clear();

    for(uint16_t i = 0; ;i++)
    {
      if(array[i][0] == 0x00 && array[i][1] == 0x00) break;

      All_obstacle.push_back(Obstacle(array[i][0], array[i][1]));

      visualization_msgs::Marker marker;
      marker.header.frame_id = "planning_odom";
      marker.header.stamp = ros::Time::now();

      marker.ns = "obstacle";

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 2;
      marker.scale.y = 2;
      marker.scale.z = 2;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.pose.position.x = array[i][0];
      marker.pose.position.y = array[i][1];
      marker.pose.position.z = 0;
      marker.id = i + 1;
      marker.lifetime = ros::Duration();

      this->marker_pub_.publish(marker);
    }

    obstacle_trans_frenet();
  }


  global_date::obj_mutex.unlock();
}


void ObsProj::obstacle_trans_frenet()
{
  for(uint16_t i = 0; i < All_obstacle.size(); i++)
  {
    uint16_t match_index = Global_Plan::Search_Match_Point(All_obstacle[i].x_set, All_obstacle[i].y_set, global_reference.referenceline,
                                              0, global_reference.referenceline.poses.size() - 1);

    //障碍物到匹配点的误差向量
    double d_err[2] = {(All_obstacle[i].x_set - global_reference.referenceline.poses[match_index].pose.position.x),
                       (All_obstacle[i].y_set - global_reference.referenceline.poses[match_index].pose.position.y)};
    //匹配点的法向量
    double nor[2] = {-sin(global_reference.theta[match_index]), cos(global_reference.theta[match_index])};
    //匹配点的切向量
    double tor[2] = {cos(global_reference.theta[match_index]), sin(global_reference.theta[match_index])};
    //纵向误差
    double lon_err = d_err[0] * tor[0] + d_err[1] * tor[1];
    //横向误差
    double lat_err = d_err[0] * nor[0] + d_err[1] * nor[1];


    if(abs(lon_err) > 5 || abs(lat_err) > 5)
    {
      continue;     //这个障碍物理参考线太远
    }

    All_obstacle[i].s_set = global_reference.s[match_index] + lon_err;
    All_obstacle[i].l_set = lat_err;
    All_obstacle[i].obs_flag = true;
  }
}

double ObsProj::calc_obstacle_cost(const double road_s, const double road_l, const double cost_obs, const double cost_neib)
{
  double cost = 0;

  for(uint16_t i = 0; i < All_obstacle.size(); i++)
  {
    if(All_obstacle[i].obs_flag == false)
    {
      continue;
    }
    
    if(abs(All_obstacle[i].s_set - road_s) > All_obstacle[i].length)
    {
      continue;      
    }
    
    double distance = sqrt(pow(All_obstacle[i].s_set - road_s, 2) + 
                           pow(All_obstacle[i].l_set - road_l, 2));

    if(distance <= All_obstacle[i].width/2)
    {
      cost += cost_obs;
    }
    else if(distance > All_obstacle[i].width/2 && distance < All_obstacle[i].width/2 + 0.5)
    {
      cost =  cost + (All_obstacle[i].width/2 + 0.5 - distance) * cost_neib / 0.5;
    }
  }

  return cost;
}

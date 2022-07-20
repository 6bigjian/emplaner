#include "obstacle/obstacle.h"
#include "global/global_planning.h"

std::vector<Obstacle> ObsProj::Static_Obstacle;
std::vector<MovObstacle> ObsProj::Move_Obstacle;

double moveobj1[21][3]=
{
  {-206.563,  -59.1676,   2.92846},
  {-210.583,  -58.2976,   2.92846},
  {-214.635,  -57.3417,   2.90995},
  {-218.378,  -56.7809,   2.99377},
  {-221.249,  -56.2657,   2.96417},
  {-224.999,  -56.1009,   3.10012},
  {-228.777,  -55.7835,   3.05802},
  {-232.713,  -56.44,    -2.96808},
  {-237.335,  -57.5319,  -2.90928},
  {-241.347,  -58.8664,  -2.81946},
  {-245.552,  -60.3199,  -2.80880},
  {-248.994,  -62.4875,  -2.57276},
  {-252.211,  -64.6672,  -2.54604},
  {-255.763,  -67.1168,  -2.53781},
  {-259.447,  -69.5562,  -2.55672},
  {-262.432,  -71.7206,  -2.51401},
  {-265.463,  -73.9551,  -2.50634},
  {-268.845,  -76.4616,  -2.50380},
  {-271.986,  -78.7349,  -2.51507},
  {-274.917,  -80.7282,  -2.54428},
  {-278.263,  -83.1613,  -2.51271}
};

double moveobj2[11][3]=
{
  {-281.282,  -87.4214,   0.561633},
  {-277.142,  -84.8164,   0.561633},
  {-273.767,  -82.3637,   0.628886},
  {-270.838,  -80.3718,   0.597315},
  {-267.664,  -78.0751,   0.626527},
  {-264.275,  -75.5634,   0.637796},
  {-261.252,  -73.3349,   0.635252},
  {-258.308,  -71.1994,   0.627586},
  {-254.643,  -68.7738,   0.584878},
  {-251.082,  -66.3182,   0.603782},
  {-247.894,  -64.1578,   0.595548}
};

ObsProj::ObsProj()
{
  ros::NodeHandle n_obs;
  n_obs.param("simulation_model", this->simulation_flag, true);

  if(this->simulation_flag == true)
  {
    this->StaticMarker_pub_ = n_obs.advertise<visualization_msgs::Marker>("/static_obc",10);
    this->MoveMarker_pub_ = n_obs.advertise<visualization_msgs::Marker>("/move_obc",10);
  }
    
}

void ObsProj::Get_globline()
{
    global_date::my_mutex.lock();
    this->global_reference.cope_data(Hdmap_Build::Global_Path_);
    global_date::my_mutex.unlock();
}


void ObsProj::Set_StaticObstacle()
{
  global_date::obj_mutex.lock();
  if(this->simulation_flag == true)
  {
    double array[][2] = 
    {
      // {-234.2, -59.0},
      // {-224.7, -55.7},
      // {-255.2, -67.8},
      // {-275.8, -81.5},
      { 0x00,   0x00}     //结束标识符
    };

    Static_Obstacle.clear();

    for(uint16_t i = 0; ;i++)
    {
      if(array[i][0] == 0x00 && array[i][1] == 0x00) break;

      Static_Obstacle.push_back(Obstacle(array[i][0], array[i][1]));

      visualization_msgs::Marker marker;
      marker.header.frame_id = "planning_odom";
      marker.header.stamp = ros::Time::now();

      marker.ns = "static obstacle";

      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = Static_Obstacle[i].width;
      marker.scale.y = Static_Obstacle[i].length;
      marker.scale.z = 2;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.pose.position.x = array[i][0];
      marker.pose.position.y = array[i][1];
      marker.pose.position.z = 0;
      marker.id = i;
      marker.lifetime = ros::Duration();

      this->StaticMarker_pub_.publish(marker);
    }
    obstacle_trans_frenet(STATICOBSTACLE);
  }


  global_date::obj_mutex.unlock();
}


void ObsProj::Set_simulateMovObst()
{
  if(this->simulation_flag == true)
  {
    global_date::obj_mutex.lock();

    Move_Obstacle.clear();
    // Move_Obstacle.push_back(MovObstacle(moveobj1, 2, 21));
    Move_Obstacle.push_back(MovObstacle(moveobj2, 0.5, 11));

    global_date::obj_mutex.unlock();
  }
}


void ObsProj::calc_movobjposition(double time)
{
if(this->simulation_flag == false) goto NotSimulate;

  global_date::obj_mutex.lock();
  for(int16_t i = 0; i < Move_Obstacle.size(); i++)
  {
    Move_Obstacle[i].calc_nowpoint(1.0/time);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "planning_odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "move obstacle";

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = Move_Obstacle[i].width;
    marker.scale.y = Move_Obstacle[i].length;
    marker.scale.z = 2;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.pose.position.x = Move_Obstacle[i].x_set;
    marker.pose.position.y = Move_Obstacle[i].y_set;
    marker.pose.position.z = 0;
    marker.id = i;
    marker.lifetime = ros::Duration();

    this->MoveMarker_pub_.publish(marker);
  }

  obstacle_trans_frenet(MOVEOBSTACLE);

  global_date::obj_mutex.unlock();

NotSimulate:;

}


void ObsProj::obstacle_trans_frenet(Obstacle_type OBSTYPE)
{
if(OBSTYPE == MOVEOBSTACLE) goto TransferMoveObs;
  for(uint16_t i = 0; i < Static_Obstacle.size(); i++)
  {
    uint16_t match_index = Global_Plan::Search_Match_Point(Static_Obstacle[i].x_set, Static_Obstacle[i].y_set, global_reference.referenceline,
                                              0, global_reference.referenceline.poses.size() - 1);

    //障碍物到匹配点的误差向量
    double d_err[2] = {(Static_Obstacle[i].x_set - global_reference.referenceline.poses[match_index].pose.position.x),
                       (Static_Obstacle[i].y_set - global_reference.referenceline.poses[match_index].pose.position.y)};
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
      Static_Obstacle[i].obs_flag = false;
      continue;     //这个障碍物理参考线太远
    }

    Static_Obstacle[i].s_set = global_reference.s[match_index] + lon_err;
    Static_Obstacle[i].l_set = lat_err;
    Static_Obstacle[i].obs_flag = true;
  }
  

TransferMoveObs:
  for(uint16_t i = 0; i < Move_Obstacle.size(); i++)
  {
    uint16_t match_index = Global_Plan::Search_Match_Point(Move_Obstacle[i].x_set, Move_Obstacle[i].y_set, global_reference.referenceline,
                                              0, global_reference.referenceline.poses.size() - 1);

    //障碍物到匹配点的误差向量
    double d_err[2] = {(Move_Obstacle[i].x_set - global_reference.referenceline.poses[match_index].pose.position.x),
                       (Move_Obstacle[i].y_set - global_reference.referenceline.poses[match_index].pose.position.y)};
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
      Move_Obstacle[i].obs_flag = false;
      continue;     //这个障碍物理参考线太远
    }

    Move_Obstacle[i].s_set = global_reference.s[match_index] + lon_err;
    Move_Obstacle[i].l_set = lat_err;
    Move_Obstacle[i].obs_flag = true;
  }
}

double ObsProj::calc_obstacle_cost(const double road_s, const double road_l, const double cost_obs, const double cost_neib)
{
  double cost = 0;

  for(uint16_t i = 0; i < Static_Obstacle.size(); i++)
  {
    if(Static_Obstacle[i].obs_flag == false)
    {
      continue;
    }
    
    if(abs(Static_Obstacle[i].s_set - road_s) > Static_Obstacle[i].length)
    {
      continue;      
    }
    
    double distance = sqrt(pow(Static_Obstacle[i].s_set - road_s, 2) + 
                           pow(Static_Obstacle[i].l_set - road_l, 2));

    if(distance <= Static_Obstacle[i].width/2)
    {
      cost += cost_obs;
    }
    else if(distance > Static_Obstacle[i].width/2 && distance < Static_Obstacle[i].width/2 + 0.5)
    {
      cost =  cost + (Static_Obstacle[i].width/2 + 0.5 - distance) * cost_neib / 0.5;
    }
  }

  return cost;
}

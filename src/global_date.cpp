#include "emplaner/global_date.h"

namespace global_date {

bool gloLine_build_ = false;  //全局路径构建标志位

bool partLine_build_ = false;       //动态规划完成标志位

bool smoLine_build_ = false;  //二次规划完成标志位

bool retraj_flag_ = true; //从新规划标志位

bool reach_goal_ = false;     //抵达终点

const double vehicleWidth = 0.5;
const double vehicleLength = 1.0;

const double identifyDist = 5.0;//后方障碍物的识别范围

const double fllowDist = 2 * vehicleLength; //跟车距离

boost::recursive_mutex my_mutex;  //上锁
boost::recursive_mutex obj_mutex;  //障碍物锁
}

namespace Sline{
std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet;

Dynamic_planning::Road_mags reference_path;   //从全局路劲提取出来的局部参考路劲
}

/*根据道路的referenceline信息计算道路的s、theta*/
void Dynamic_planning::Road_mags::Calc_Path_msgs()
{
  Calc_theta();
  Calc_s();
}

void Dynamic_planning::Road_mags::Calc_s()
{
  s.clear();

  double distance = 0;
  s.push_back(0);

  for(int16_t i = 1; i < referenceline.poses.size(); i++)
  {
    //计算参考线坐标点与道路长度s之间的关系
    distance += sqrt(pow(referenceline.poses[i].pose.position.x - referenceline.poses[i-1].pose.position.x, 2) +
                     pow(referenceline.poses[i].pose.position.y - referenceline.poses[i-1].pose.position.y, 2));
    s.push_back(distance);
  }
}

void Dynamic_planning::Road_mags::Calc_theta()
{
  theta.clear();

  theta.push_back(atan2((referenceline.poses[1].pose.position.y - referenceline.poses[0].pose.position.y),
                        (referenceline.poses[1].pose.position.x - referenceline.poses[0].pose.position.x)));

  for(int16_t i = 1; i < referenceline.poses.size(); i++)
  {
    theta.push_back(atan2((referenceline.poses[i].pose.position.y - referenceline.poses[i-1].pose.position.y),
                          (referenceline.poses[i].pose.position.x - referenceline.poses[i-1].pose.position.x)));
  }
}


void Dynamic_planning::Road_mags::clear_data()
{
  referenceline.poses.clear();
  k.clear();
  s.clear();
  theta.clear();
}

void Dynamic_planning::Road_mags::cope_data(const Dynamic_planning::Road_mags& R)
{
  clear_data();
  referenceline.header.frame_id = R.referenceline.header.frame_id;
  referenceline.header.stamp = R.referenceline.header.stamp;
  referenceline.poses = R.referenceline.poses;
  k = R.k;
  s = R.s;
  theta = R.theta;
}
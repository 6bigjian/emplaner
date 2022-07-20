#pragma once

#include "emplaner/head.h"

enum Obstacle_type{
  STATICOBSTACLE = 0,
  MOVEOBSTACLE
};

class Obstacle
{
public:
  Obstacle(double x, double y)
  {
    x_set = x;
    y_set = y;
  }
  double x_set;
  double y_set;
  double s_set;//相对于全局路径的s
  double l_set;//相对于全局路径的l
  double length = 2.0;
  double width = 2.0;
  bool obs_flag = false;   //这个障碍物是否靠近参考线
};

class MovObstacle : public Obstacle
{
public:
  /*仿真下构造动态障碍物*/
  MovObstacle(double (*moveobj)[3], double Vx, int16_t cacity, int16_t index = 0) : Obstacle(moveobj[index][0],moveobj[index][1])
  {
    objtraj = moveobj;
    V_set = Vx;
    startIndex = index;
    MotionDirect = moveobj[index][2];
    Capacity = cacity;
    now_s = 0;
    calc_s();
  }

  /*实际跑车构造*/
  MovObstacle(double x, double y, double len, double wid, double Vx, double direction): Obstacle(x, y)
  {
    length = len;//长度
    width = wid;//宽度
    V_set = Vx;//速度
    MotionDirect = direction;
  }

  void calc_s()
  {
    double points = 0;
    s.clear();
    s.reserve(Capacity);
    s.push_back(0);
    for(int16_t i = 1; i < Capacity; i++)
    {
      points += sqrt(pow(objtraj[i][0] - objtraj[i-1][0],2) + 
                     pow(objtraj[i][1] - objtraj[i-1][1],2));

      s.push_back(points);
    }
  }

  void calc_nowpoint(double nowtime)
  {
    now_s += nowtime*V_set;

    for(int16_t i = 0; i < Capacity; i++)
    {
      if(now_s < s[i+1])
      {
        x_set = objtraj[i][0] + (now_s - s[i])*cos(objtraj[i][2]);
        y_set = objtraj[i][1] + (now_s - s[i])*sin(objtraj[i][2]);
        MotionDirect = objtraj[i][2];
        break;
      }

      if(i == Capacity-1)//超出终点
      {
        V_set = 0.0;
      }
    }
  }

  double V_set;         //障碍物速度
  double MotionDirect;  //障碍物运动方向
  obstacle_strategy OBSSTRG;
  int16_t backIndex;  //动态障碍物预测相交区域的后方坐标
  int16_t frontIndex;//动态障碍物预测相交区域的前方坐标

  /*仿真下障碍物信息*/
  double startIndex;
  double (*objtraj)[3];
  int Capacity;
  double now_s;

private:
  std::vector<double> s;
};


class ObsProj
{
public:
  ObsProj();

  void Get_globline();
  void Set_StaticObstacle();
  void Set_simulateMovObst();
  void calc_movobjposition(double time);

  void obstacle_trans_frenet(Obstacle_type OBSTYPE);
  double calc_obstacle_cost(const double road_s, const double road_l, const double cost_obs = 1e6, const double cost_neib = 1000);

  static std::vector<Obstacle> Static_Obstacle;
  static std::vector<MovObstacle> Move_Obstacle;

private:
  bool simulation_flag;

  Dynamic_planning::Road_mags global_reference; //全局路劲  

  const std::string Frame_id = "planning_odom";

  ros::Publisher StaticMarker_pub_;
  ros::Publisher MoveMarker_pub_;
};







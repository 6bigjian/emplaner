#pragma once

#include"emplaner/head.h"


namespace Dynamic_planning {
class Frenet_mags
{
public:
    double s = 0;               //frenet横向距离
    double s_dot = 0;           //s_dot 车在道路几何上的投影速率
    double s_dot2 = 0;          //投影加速度
    double l = 0;               //横向距离
    double l_dot = 0;           //l_dot = l/t
    double l_dot2 = 0;          //l_dot2 = l_dot/t
    double dl = 0;              //dl = l/s
    double ddl = 0;             
    int16_t index = -1;         //该点在frenet曲线上的坐标，-1表示不再曲线上。每段5次多项式取20个点，坐标为 1-20
};

class Road_mags  //路径信息
{
public:
    nav_msgs::Path referenceline;
    std::vector<double> k;           //每个路径点的曲率
    std::vector<double> s;           //路径长度
    std::vector<double> l;           //横向
    std::vector<double> theta;       //轨迹在笛卡尔坐标系下的切线方向

    // Frenet_mags plan_start_mags;

    void Calc_Path_msgs();
    void Calc_k();
    void Calc_theta();
    void Calc_s();
    
    void clear_data();
    void cope_data(const Road_mags& R);
};

}

namespace Sline {

extern std::vector<Dynamic_planning::Frenet_mags> dynamic_frenet;

extern Dynamic_planning::Road_mags reference_path;   //从全局路劲提取出来的局部参考路劲
}


namespace global_date {

extern bool gloLine_build_;

extern bool partLine_build_;

extern bool smoLine_build_;

extern bool retraj_flag_;

extern bool reach_goal_;

extern const double vehicleWidth;
extern const double vehicleLength;

extern boost::recursive_mutex my_mutex;
extern boost::recursive_mutex obj_mutex;
}



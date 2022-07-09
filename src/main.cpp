#include "emplaner/head.h"
#include "global/global_planning.h"
#include "global/hdmap_building.h"
#include "control/verhic_control.h"
#include "dynamic/dynamic_path.h"
#include "obstacle/obstacle.h"
#include "QuadProg/smooth.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "emplaner_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    Global_Plan global_routing_generate;
    Dynamic_Plan dynamic_routing_generate;
    Ver_Ctrl vehicle_ctrl;
    SmoLine routing_smool;
    
    
    while(ros::ok())
    {
        //目前这里有bug，需要换成多线程的回调函数，不然会数据冲突
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
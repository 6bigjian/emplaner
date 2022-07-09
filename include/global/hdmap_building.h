#pragma once

#include "emplaner/head.h"

// extern lanelet::LaneletMapPtr hd_map;
// extern lanelet::routing::RoutingGraphUPtr routingGraph;

class Hdmap_Build
{
public:
    Hdmap_Build();
    lanelet::routing::RoutingGraphUPtr InitHdMap(lanelet::LaneletMapPtr& map);
    void Calc_origin_point();//计算坐标偏移量

    bool Get_ClosestLanelet(const geometry_msgs::Pose& search_pose, lanelet::Lanelet* closest_lanelet,
                            lanelet::LaneletMapPtr& hpmap, double distance_cost);
    bool Have_nearest_linelet(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
                              lanelet::Lanelet& start_lanelet, lanelet::Lanelet& goal_lanelet,
                              lanelet::LaneletMapPtr& hpmap);
    void Generate_Global_Routing(const lanelet::routing::LaneletPath& shortestPath,
                                geometry_msgs::PoseArray& stores_path_array_, const Eigen::Vector3d& offset_point);
    void search_ShortesrPath(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose, 
                            const Eigen::Vector3d& offset_point);
    void Insert_Point(const int& start_index, const int& end_index,   //全局路径插点
                      double spacing_distance = 0.5, bool nearly = true);
    void Generate_Original_Referenceline(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);
    double Calc_kappa(const geometry_msgs::Pose& point1, const geometry_msgs::Pose& point2, const geometry_msgs::Pose& point3);

    static Eigen::Vector3d origin_point_;
    static lanelet::projection::MGRSProjector projector; // MGRS,  Projection：提供全球地理坐标系到局部平面坐标系的准换
    static Dynamic_planning::Road_mags Global_Path_;

private:
    geometry_msgs::PoseArray stores_path_array_;
        /****路由图参数****/
    lanelet::LaneletMapPtr hd_map;
    lanelet::routing::RoutingGraphUPtr routingGraph;
    /****************/
    const std::string Frame_id = "planning_odom";
    std::string hdmap_file_map;         //地图文件
};







// extern lanelet::projection::MGRSProjector projector;




// void Generate_final_Referenceline(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose,
//                                   const nav_msgs::Path& referenceline);




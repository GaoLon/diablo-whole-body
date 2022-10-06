#ifndef _TRAJ_VISUALIZATION_H_
#define _TRAJ_VISUALIZATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "robo_map/robo_map_manager.h"
#include <Eigen/Eigen>

#include <minco_opt/poly_traj_utils.hpp>
#include <vector>

namespace ugv_planner {

    class TrajVisualization {

    public:
        TrajVisualization();
        ~TrajVisualization();
        void init(ros::NodeHandle &nh);
        void visTarget(geometry_msgs::PoseStamped& target_info);


        void renderPoints(std::vector<Eigen::Vector3d> pts, Eigen::Vector3d color, int type, double scale, int id);

        void visPolynomialTrajectory(Trajectory traj, Eigen::Vector3d color, int id);

    private:
        ros::NodeHandle nh;
        ros::Publisher  point_vis_pub;
        ros::Publisher  trajectory_vis_pub;
        ros::Publisher  target_vis_pub;
    };
}


#endif

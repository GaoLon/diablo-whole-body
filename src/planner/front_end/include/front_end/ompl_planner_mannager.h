#ifndef _OMPL_PLANNER_MANAGER_H_
#define _OMPL_PLANNER_MANAGER_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include "robo_map/robo_map_manager.h"

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ugv_planner
{
    class OMPLPlanner
    {
        private:
            MapManager::Ptr robo_map;
            double map_size_x, map_size_y, map_size_z;
            std::vector<Eigen::Vector4d> front_end_path;
            std::vector<Eigen::Vector3d> model_points;
            std::vector<visualization_msgs::Marker> model;
            
        public:
            OMPLPlanner() {}
            ~OMPLPlanner();

            ros::Publisher vis_pub;

            void setParam(ros::NodeHandle& nh);
            void init();
            std::vector<Eigen::Vector4d> plan(const Eigen::Vector4d start_state, const Eigen::Vector4d end_state);
            bool isStateValid(const ob::State *state);
            void setEnvironment(const MapManager::Ptr& env);
            void visWholeBodyPath();

            inline visualization_msgs::Marker getMarker(int id)
            {
                visualization_msgs::Marker mk;
                mk.id               = id;
                mk.header.stamp     = ros::Time::now();
                mk.header.frame_id  = "world";
                mk.action           = visualization_msgs::Marker::ADD;
                mk.type             = visualization_msgs::Marker::CUBE;
                mk.ns               = "ugv";
                mk.color.r          = 1.0;
                mk.color.g          = 1.0;
                mk.color.b          = 1.0;
                mk.color.a          = 0.8;
                mk.pose.orientation.w = 1.0;
                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                return mk;
            }

            typedef shared_ptr<OMPLPlanner> Ptr;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif
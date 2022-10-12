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
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
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
namespace oc = ompl::control;

namespace ugv_planner
{
    class DiabloProjection : public ob::ProjectionEvaluator
    {
        public:
        
        DiabloProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
        {
        }
        
        virtual unsigned int getDimension(void) const
        {
            return 3;
        }
        
        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(3);
            cellSizes_[0] = 0.02;
            cellSizes_[1] = 0.02;
            cellSizes_[2] = 0.3;
        }
        
        virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
        {
            const auto *se2 = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
            projection(0) = se2->getX();
            projection(1) = se2->getY();
            projection(2) = se2->getYaw();
        }
    };

    class OMPLPlanner
    {
        private:
            MapManager::Ptr robo_map;
            double map_size_x, map_size_y, map_size_z;
            double a_max, b_max;
            std::vector<Eigen::Vector4d> front_end_path;
            std::vector<Eigen::Vector3d> model_points;
            std::vector<visualization_msgs::Marker> model;

            // debug
            int check_times = 0;
            double colli_check_time = 0.0;
            
        public:
            OMPLPlanner() {}
            ~OMPLPlanner();

            ros::Publisher vis_pub;

            void setParam(ros::NodeHandle& nh);
            void init();
            std::vector<Eigen::Vector4d> plan(const Eigen::Vector4d start_state, const Eigen::Vector4d end_state);
            std::vector<Eigen::Vector4d> kinoPlan(const Eigen::VectorXd start_state, const Eigen::VectorXd end_state);
            bool isStateValid(const ob::State *state);
            bool isHStateValid(const ob::State *state);
            void propaGate(const oc::SpaceInformationPtr si, const ob::State *state, const oc::Control* control, const double duration, ob::State *result);
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

            inline int sign(double a)
            {
                if (fabs(a)<1e-4)
                    return 0;
                if (a > 0.0)
                    return 1;
                else
                    return -1;
            }

            typedef shared_ptr<OMPLPlanner> Ptr;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}

#endif
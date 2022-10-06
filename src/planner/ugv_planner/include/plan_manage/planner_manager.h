#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "robo_map/robo_map_manager.h"
#include <ros/ros.h>
#include <ugv_planner/traj_visualization.h>
#include <minco_opt/traj_opt.h>
#include "mpc/Polynome.h"

namespace ugv_planner
{
  class UGVPlannerManager
  {
  public:
    UGVPlannerManager();
    ~UGVPlannerManager();
    void init(ros::NodeHandle& nh);

    void targetRcvCallback(const geometry_msgs::PoseStamped target_info);
    void odomRcvCallback(const nav_msgs::Odometry odom);
    void trigRcvCallback(const std_msgs::Bool trig);

    bool globalReplan();
    bool localReplan(Eigen::Vector3d start_pos, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d local_target, Eigen::Vector3d end_v, Eigen::Vector3d end_a);
    void PublishTraj();

    bool generateMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, vector<Eigen::Vector3d> path);
    bool generateMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, Eigen::Vector3d end_v, Eigen::Vector3d end_a, vector<Eigen::Vector3d> path);
    void execFSMCallback(const ros::TimerEvent &e);

  private:

    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber  mpc_trig_sub;
    ros::Publisher  traj_pub;
    ros::Publisher  target_pub;
    ros::Timer exec_timer;
    TrajVisualization vis_render;
    double last_plan_time;
    double plan_horizon;
    double global_duration;
    double local_duration;

    ros::NodeHandle nh;
    ros::Time start_time;
    ros::Time local_start_time;

    bool has_odom;
    bool has_target;
    bool first_plan;
    bool has_local_traj;
    bool use_replan;
    bool back;
    bool mpc_trigger;

    geometry_msgs::PoseStamped    target;        //the global target
    Eigen::Vector3d               target_pos;    // = target
    Eigen::Vector3d               now_pos;       // = rt_odometry
    Eigen::Vector3d               now_vel;       // = rt_odometry'
    Eigen::Vector3d               now_acc;       // = rt_odometry''

    unique_ptr<TrajOpt> minco_traj_optimizer;
    Trajectory final_trajectory;
    Trajectory global_trajectory;
    Trajectory local_trajectory;
  public:

    MapManager::Ptr robo_map_manager;

  };
} 

#endif
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" 
#include <cstdlib>
#include <ctime>

const double pi = 3.1415926535;

namespace ugv_planner
{
  UGVPlannerManager::UGVPlannerManager() 
  {
    has_odom = false;
    has_target = false;
    first_plan = true;
    has_local_traj = false;
    use_replan = false;
    back = false;
    mpc_trigger = false;

    last_plan_time = 0.0;
    global_duration = 0.0;
    local_duration = 0.0;

    now_pos = now_vel = now_acc = Eigen::Vector3d(0,0,0);

    robo_map_manager.reset(new MapManager);
    minco_traj_optimizer.reset(new TrajOpt());
    ompl_manager.reset(new OMPLPlanner());
    kino_astar_manager.reset(new KinoAstar());

    ROS_INFO("trajectory planner is ready."); 
  }

  UGVPlannerManager::~UGVPlannerManager() {}

  void UGVPlannerManager::init(ros::NodeHandle& nh)
  {
    this->nh       = nh;

    nh.param("plan_horizon", plan_horizon, 1.0);
    nh.param("use_replan", use_replan, false);
    nh.param("front_end_type", front_end_type, 0);

    target_sub     = nh.subscribe("/move_base_simple/goal", 1, &UGVPlannerManager::targetRcvCallback, this);
    odom_sub       = nh.subscribe("odom",1 ,&UGVPlannerManager::odomRcvCallback, this);
    mpc_trig_sub   = nh.subscribe("/trigger",1 ,&UGVPlannerManager::trigRcvCallback, this);
    traj_pub       = nh.advertise<mpc::Polynome>("trajectory",3);
    target_pub     = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    if (use_replan)
    {
      exec_timer = nh.createTimer(ros::Duration(1.0), &UGVPlannerManager::execFSMCallback, this);
    }

    minco_traj_optimizer -> setParam(nh);
    minco_traj_optimizer -> setEnvironment(robo_map_manager);
    robo_map_manager -> initMap(nh);
    vis_render.init(nh);
    if (front_end_type == 0)
    {
      ompl_manager->setParam(nh);
      ompl_manager->setEnvironment(robo_map_manager);
      ompl_manager->init();
    }
    else
    {
      kino_astar_manager->setParam(nh);
      kino_astar_manager->setEnvironment(robo_map_manager);
      kino_astar_manager->init();
    }
  }

  void UGVPlannerManager::trigRcvCallback(const std_msgs::Bool trig)
  {
    if (use_replan)
    {
      return;
    }

    if (mpc_trigger == true)
    {
      PublishTraj();
      mpc_trigger = false;
    }
  }

  void UGVPlannerManager::targetRcvCallback(const geometry_msgs::PoseStamped target_info)
  { 
    if(has_target == false)
    {
        ROS_INFO("Get target");
        has_target = true;
        first_plan = true;
        has_local_traj = false;
    }
    else
    {
      return;
    }

    target = target_info;
    target_pos(0) = target.pose.position.x;
    target_pos(1) = target.pose.position.y;
    target_pos(2) = 0;
    vis_render.visTarget(target);

    ros::Time t1 = ros::Time::now();
    if (globalReplan())
    {
      global_duration = global_trajectory.getTotalDuration();
      start_time = ros::Time::now();
      ROS_INFO("global plannng time consuming = %fms", (start_time-t1).toSec()*1000);
      if (use_replan)
      {
        first_plan = false;
      }
      else
      {
        // PublishTraj();
        mpc_trigger = true;
        has_target = false;
      }
    }
    else
    {
      ROS_ERROR("global plannng failed!!!");
      has_target = false;
    }
  }

  void UGVPlannerManager::odomRcvCallback(const nav_msgs::Odometry odom)
  {
    static Eigen::Vector3d last_vel(0,0,0);
    if(has_odom == false){
      ROS_INFO("Get odometry");
      has_odom = true;
    }
    now_pos(0) = odom.pose.pose.position.x;
    now_pos(1) = odom.pose.pose.position.y;
    now_pos(2) = odom.pose.pose.position.z;
    
    now_vel(0) = odom.twist.twist.linear.x;
    now_vel(1) = odom.twist.twist.linear.y;
    now_vel(2) = odom.twist.twist.linear.z;

    now_acc    = (now_vel - last_vel) * 100;
    last_vel   = now_vel;
  }

  void UGVPlannerManager::execFSMCallback(const ros::TimerEvent &e)
  {
    if (!has_odom || !has_target || first_plan)
    {
      return;
    }
    
    if ((now_pos.head(2)-target_pos.head(2)).squaredNorm() < 0.09)
    {
      has_target = false;
      return;
    }

    // compute target
    geometry_msgs::PoseStamped target;
    Eigen::Vector3d local_target, end_v, end_a; 
    double now_time = (ros::Time::now() - start_time).toSec();
    if (now_time < global_duration - 2.0 && now_time>=0)
    {
      double t = now_time + 0.1;
      double length = 0.0;
      while (t<=global_duration && t>=0.0)
      {
        length += (global_trajectory.getPos(t) - global_trajectory.getPos(t-0.1)).norm();
        if (length > plan_horizon)
        {
          break;
        }
        t+=0.1;
      }
      local_target = global_trajectory.getPos(t);
      end_v = global_trajectory.getVel(t);
      end_a = global_trajectory.getAcc(t);
    }
    else
    {
      local_target = global_trajectory.getPos(global_duration);
      end_v = global_trajectory.getVel(global_duration);
      end_a = global_trajectory.getAcc(global_duration);
    }

    target.pose.position.x = local_target(0);
    target.pose.position.y = local_target(1);
    target.pose.position.z = local_target(2);
    vis_render.visTarget(target);

    // compute start state
    Eigen::Vector3d start_pos, start_v, start_a;
    if (has_local_traj)
    {
      double get_time = (ros::Time::now() - local_start_time).toSec() + last_plan_time;
      if (get_time <= local_duration)
      {
        start_pos = local_trajectory.getPos(get_time);
        start_v = local_trajectory.getVel(get_time);
        start_a = local_trajectory.getAcc(get_time);
      }
      else
      {
        start_pos = local_trajectory.getPos(local_duration);
        start_v = local_trajectory.getVel(local_duration);;
        start_a = local_trajectory.getAcc(local_duration);;
      }
    }
    else
    {
      start_pos = now_pos;
      start_v = now_vel;
      start_v(2) = 0.0;
      start_a = Eigen::Vector3d::Zero();
    }

    ros::Time t1 = ros::Time::now();
    if (localReplan(start_pos, start_v, start_a, local_target, end_v, end_a))
    {
      local_start_time = ros::Time::now();
      last_plan_time = (local_start_time - t1).toSec();
      ROS_INFO("Replan time consuming = %fms", last_plan_time*1000);
      has_local_traj = true;
      PublishTraj();
      local_duration = local_trajectory.getTotalDuration();
    }
  }

  void UGVPlannerManager::PublishTraj()
  {
      mpc::Polynome poly;
      Eigen::MatrixXd poses = final_trajectory.getPositions();
      Eigen::VectorXd ts    = final_trajectory.getDurations();

      for (int i = 0; i < poses.cols(); i++)
      {
        geometry_msgs::Point temp;
        temp.x = poses(0, i);
        temp.y = poses(1, i);
        temp.z = poses(2, i);
        // std::cout<<"z="<<poses(2, i)<<std::endl;
        poly.pos_pts.push_back(temp);
      }
      for (int i = 0; i < ts.size(); i++)
      {
        poly.t_pts.push_back(ts(i));
      }
      Eigen::Vector3d init_v = final_trajectory.getVel(0);
      Eigen::Vector3d init_a = final_trajectory.getAcc(0);
      poly.init_v.x = init_v(0);
      poly.init_v.y = init_v(1);
      poly.init_v.z = init_v(2);
      poly.init_a.x = init_a(0);
      poly.init_a.y = init_a(1);
      poly.init_a.z = init_a(2);
      poly.start_time = ros::Time::now();
      poly.back = back;//gs

      traj_pub.publish(poly);
  }

  bool UGVPlannerManager::generateMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, vector<Eigen::Vector3d> path)
  {
    return generateMincoTraj(start_pt, start_v, start_a, end_pt, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), path);
  }

  bool UGVPlannerManager::generateMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, Eigen::Vector3d end_v, Eigen::Vector3d end_a, vector<Eigen::Vector3d> path)
  {
    Eigen::MatrixXd initS, finalS;

    initS.setZero(3, 3);
    initS.col(0) = start_pt;
    initS.col(1) = start_v;
    initS.col(2) = start_a;

    finalS.setZero(3, 3);
    finalS.col(0) = end_pt;
    finalS.col(1) = end_v;
    finalS.col(2) = end_a;

    double space_resolution = 0.3;
    /// get waypoints
    vector<Eigen::Vector3d> waypoints;
    Eigen::Vector3d pos;
    double dt = 0.001;
    double mileage = 0.0 , lastwp_mileage = 0.0;

    int di = space_resolution / robo_map_manager -> getResolution().first;
    for(int i = path.size() - 1 ; i >= 0 ; i -= di)
    {
        pos = path[i];
        waypoints.push_back(pos);
    }
    waypoints.push_back(end_pt);
    
    int pieceN = waypoints.size();

    // global_map_manager -> publishESDFMap();
    if (minco_traj_optimizer -> generate_traj(initS, finalS, waypoints, pieceN, final_trajectory, false))
    {
      vis_render.visPolynomialTrajectory(final_trajectory, Eigen::Vector3d(1,1,0), 7);
      cout << "[minco optimizer]: optimization success." << endl;
      if (first_plan)
      {
        global_trajectory = final_trajectory;
      }
      else
      {
        local_trajectory = final_trajectory;
        // PublishTraj();
      }
      return true;
    }
    else
    {
      ROS_WARN("[minco optimizer]: optimization failed.");
      return false;
    }
  }

  bool UGVPlannerManager::globalReplan()
  {
    if (front_end_type == 0)
    {
      Eigen::Matrix<double, 6, 1> begin_p, target_p;
      begin_p.block<2, 1>(0, 0) = now_pos.head(2);
      target_p.block<2, 1>(0, 0) = target_pos.head(2);
      begin_p(2) = target_p(2) = 0.0;
      begin_p(3) = target_p(3) = 0.0;
      begin_p(4) = target_p(4) = 0.0;
      begin_p(5) = 0.15;
      target_p(5) = 0.0;
      vector<Eigen::Vector4d> front_end_path = ompl_manager->kinoPlan(begin_p, target_p);
    }
    else
    {
      Eigen::Vector4d begin_p, target_p;
      begin_p.block<2, 1>(0, 0) = now_pos.head(2);
      target_p.block<2, 1>(0, 0) = target_pos.head(2);
      begin_p(2) = target_p(2) = 0.0;
      begin_p(3) = 0.16;
      target_p(3) = 0.0;
      vector<Eigen::Vector4d> front_end_path = kino_astar_manager->plan(begin_p, target_p);
    }
    
    vector<Eigen::Vector3d> path_0;

    vis_render.renderPoints(path_0, Eigen::Vector3d(0.6,0.6,0.1),0, 0.05, 1);

    if (path_0.size() > 0)
    {
      if (generateMincoTraj(now_pos, now_vel, Eigen::Vector3d::Zero(), target_pos, path_0))
      {
        return true;
      }
    }
    else
    {
      ROS_WARN(" No path! ");
    }

    return false;
  }

  bool UGVPlannerManager::localReplan(Eigen::Vector3d start_pos, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d local_target, Eigen::Vector3d end_v, Eigen::Vector3d end_a)
  { 
    vector<Eigen::Vector3d> path_0;
    vis_render.renderPoints(path_0, Eigen::Vector3d(0.6,0.6,0.1),0, 0.05, 1);

    if (path_0.size() > 0)
    {
      if (generateMincoTraj(start_pos, start_v, start_a, local_target, end_v, end_a, path_0))
      {
        return true;
      }
    }
    else
    {
      ROS_WARN(" No path! ");
    }
    return false;
  }

} //namespace
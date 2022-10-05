#include <ugv_planner/traj_visualization.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Eigen>

namespace ugv_planner {


    TrajVisualization::TrajVisualization(){}
    TrajVisualization::~TrajVisualization(){}


    void TrajVisualization::init(ros::NodeHandle &nh)
    {
        this->nh = nh;
        trajectory_vis_pub  = nh.advertise<visualization_msgs::Marker>("trajectory/vis", 2);
        target_vis_pub      = nh.advertise<visualization_msgs::Marker>("target/vis", 20);
        point_vis_pub       = nh.advertise<visualization_msgs::Marker>("point2/vis", 20);
    }


    void TrajVisualization::visTarget(geometry_msgs::PoseStamped& target_info)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id  = "world";
        sphere.header.stamp     = ros::Time::now();
        sphere.type             = visualization_msgs::Marker::SPHERE;
        sphere.action           = visualization_msgs::Marker::ADD;
        sphere.id               = 1;

        sphere.pose.orientation.w   = 1.0;
        sphere.color.r              = 1.0;
        sphere.color.g              = 0.2;
        sphere.color.b              = 0.2;
        sphere.color.a              = 0.7;
        sphere.scale.x              = 0.5;
        sphere.scale.y              = 0.5;
        sphere.scale.z              = 0.5;
        sphere.pose.position.x      = target_info.pose.position.x;
        sphere.pose.position.y      = target_info.pose.position.y;
        sphere.pose.position.z      = target_info.pose.position.z;

        target_vis_pub.publish(sphere);
    }

    void TrajVisualization::visPolynomialTrajectory(Trajectory traj, Eigen::Vector3d color, int id = 10)
    {
        visualization_msgs::Marker traj_vis;
        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "world";
        traj_vis.id = id;
        traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
        traj_vis.scale.x = 0.05;
        traj_vis.scale.y = 0.05;
        traj_vis.scale.z = 0.05;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;

        traj_vis.color.a = 1.0;
        traj_vis.color.r = color(0);
        traj_vis.color.g = color(1);
        traj_vis.color.b = color(2);
        geometry_msgs::Point pt;
        Eigen::Vector3d pos;

        double t_duration = traj.getTotalDuration();
        for(double t = 0; t < t_duration; t += 0.05)
        {
            pos = traj.getPos(t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            // pt.z = 0.05;
            traj_vis.points.push_back(pt);
        }
        trajectory_vis_pub.publish(traj_vis);
    }

    void TrajVisualization::renderPoints(vector<Eigen::Vector3d> pts, Eigen::Vector3d color , int type, double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id  = "world";
        sphere.header.stamp     = ros::Time::now();
        if(type == 0)
            sphere.type             = visualization_msgs::Marker::LINE_STRIP;
        else if(type == 1)
            sphere.type             = visualization_msgs::Marker::SPHERE_LIST;
        sphere.action           = visualization_msgs::Marker::ADD;
        sphere.id               = id;

        sphere.pose.orientation.w   = 1.0;
        sphere.color.r              = color(0);
        sphere.color.g              = color(1);
        sphere.color.b              = color(2);
        sphere.color.a              = 0.8;
        sphere.scale.x              = scale;
        sphere.scale.y              = scale;
        sphere.scale.z              = scale;
        geometry_msgs::Point pt;
        Eigen::Vector3d ptv;
        for(int i = 0 ; i < pts.size(); i++)
        {
            ptv = pts[i];
            pt.x = ptv(0);
            pt.y = ptv(1);
            pt.z = ptv(2);
            sphere.points.push_back(pt);
        }

        point_vis_pub.publish(sphere);
    }
}



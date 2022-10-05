#include <ros/console.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class UGVVis{
public:
  void init(ros::NodeHandle &nh);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  ros::Subscriber waypoints_sub, odom_sub;
  ros::Publisher visugv_pub;

  double max_height;
private:
  std::string mesh_resource ,mesh_resource2 ,mesh_resource3 ,mesh_resource4;
  std::string frame;
  double ugv_l, ugv_w, ugv_h;
  visualization_msgs::Marker getMarker(int id);
};

void UGVVis::init(ros::NodeHandle &nh)
{
  /*  param  */
  nh.param("ugv/ugv_l",ugv_l,0.6);
  nh.param("ugv/ugv_w",ugv_w,0.4);
  nh.param("ugv/ugv_h",ugv_h,0.3);
  nh.param("ugv/mesh" ,mesh_resource, std::string("package://diablo_model/model/car.dae"));
  nh.param("ugv/mesh2" ,mesh_resource2, std::string("package://diablo_model/model/car.dae"));
  nh.param("ugv/mesh3" ,mesh_resource3, std::string("package://diablo_model/model/car.dae"));
  nh.param("ugv/mesh4" ,mesh_resource4, std::string("package://diablo_model/model/car.dae"));
  nh.param("ugv/frame",frame,std::string("world"));

  nh.param("max_height",max_height, 1.0);

  /* callback */
  visugv_pub    = nh.advertise<visualization_msgs::MarkerArray>("odom_mesh", 100,true);
  odom_sub      = nh.subscribe("odom", 1, &UGVVis::odomCallback, this);
}

visualization_msgs::Marker UGVVis::getMarker(int id)
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
  mk.color.a          = 1.0;
  return mk;
}


void UGVVis::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker mk;
  Eigen::Vector3d odom_pose(odom->pose.pose.position.x,\
                            odom->pose.pose.position.y,\
                            odom->pose.pose.position.z);
  Eigen::Quaterniond q( odom->pose.pose.orientation.w,\ 
                        odom->pose.pose.orientation.x,\ 
                        odom->pose.pose.orientation.y,\
                        odom->pose.pose.orientation.z );
  Eigen::Matrix3d R(q);
  Eigen::Vector3d left_odom = odom_pose + R*Eigen::Vector3d(0, 0.2, 0);
  Eigen::Vector3d right_odom = odom_pose + R*Eigen::Vector3d(0, -0.2, 0);

  mk = getMarker(0);
  mk.pose.orientation = odom->pose.pose.orientation;
  mk.pose.position.x = odom_pose[0];
  mk.pose.position.y = odom_pose[1];
  mk.pose.position.z = odom_pose[2] - 0.4 / 2;
  mk.scale.x = 0.4;
  mk.scale.y = 0.3;
  mk.scale.z = 0.4;
  ma.markers.push_back(mk);

  mk = getMarker(1);
  mk.pose.orientation = odom->pose.pose.orientation;
  mk.pose.position.x = left_odom[0];
  mk.pose.position.y = left_odom[1];
  mk.pose.position.z = 0.15;
  mk.scale.x = 0.2;
  mk.scale.y = 0.1;
  mk.scale.z = 0.3;
  ma.markers.push_back(mk);

  mk = getMarker(2);
  mk.pose.orientation = odom->pose.pose.orientation;
  mk.pose.position.x = right_odom[0];
  mk.pose.position.y = right_odom[1];
  mk.pose.position.z = 0.15;
  mk.scale.x = 0.2;
  mk.scale.y = 0.1;
  mk.scale.z = 0.3;
  ma.markers.push_back(mk);

  visugv_pub.publish(ma);

  // static double w_ang = 0;
  // w_ang += odom->twist.twist.angular.y * 0.1 ;
  // visualization_msgs::Marker WpMarker;
  // WpMarker.id               = 0;
  // WpMarker.header.stamp     = ros::Time::now();
  // WpMarker.header.frame_id  = "world";
  // WpMarker.action           = visualization_msgs::Marker::ADD;
  // WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
  // WpMarker.ns               = "ugv_mesh";
  // WpMarker.mesh_use_embedded_materials = true;
  // WpMarker.color.r          = 0.0;
  // WpMarker.color.g          = 0.0;
  // WpMarker.color.b          = 0.0;
  // WpMarker.color.a          = 0.0;
  // WpMarker.scale.x          = ugv_l/4.5;
  // WpMarker.scale.y          = ugv_l/4.5;
  // WpMarker.scale.z          = ugv_l/4.5;

  // Eigen::Quaterniond q( odom->pose.pose.orientation.w, 
  //                       odom->pose.pose.orientation.x, 
  //                       odom->pose.pose.orientation.y,
  //                       odom->pose.pose.orientation.z );

  // Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
  // Eigen::Matrix3d       R(q);
  // Eigen::Vector3d odomm(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);//gs
  // Eigen::Vector3d new_oodom = odomm + R * Eigen::Vector3d(0.5,0,0);//gs

  // double odom_yaw 	    = atan2(R.col(0)[1],R.col(0)[0]); 
  // if(odom_yaw > 0){eulerAngle[0] *= -1;} 

  // Eigen::AngleAxisd rollTrans(Eigen::AngleAxisd( eulerAngle[1],Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchTrans(Eigen::AngleAxisd(-eulerAngle[0],Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd yawTrans(Eigen::AngleAxisd(eulerAngle[2],Eigen::Vector3d::UnitZ())); 
  // Eigen::Quaterniond qtr = yawTrans * pitchTrans * rollTrans;

  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd( M_PI/2,Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())); 
  // Eigen::Quaterniond qder = yawAngle * pitchAngle * rollAngle;

  // double d = max_height - odom -> pose.pose.position.z;
  // Eigen::AngleAxisd rollDir(Eigen::AngleAxisd( 0,Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchDir0(Eigen::AngleAxisd(d,Eigen::Vector3d::UnitZ()));
  // Eigen::Quaterniond qdir0 = pitchDir0 * rollDir;
  // Eigen::AngleAxisd pitchDir1(Eigen::AngleAxisd(-2*d,Eigen::Vector3d::UnitZ()));
  // Eigen::Quaterniond qdir1 = pitchDir1 * rollDir;
  // Eigen::AngleAxisd pitchDir2(Eigen::AngleAxisd(2*d + w_ang ,Eigen::Vector3d::UnitZ()));
  // Eigen::Quaterniond qdir2 = pitchDir2 * rollDir;

  // double jump_height = odom->twist.twist.angular.x;

  // double z_buf = ( 0.5 * jump_height ) > (max_height - 0.8) ? (max_height - 0.8) : ( 0.5 * jump_height ) ;
  // d += 0.5*z_buf;

  // Eigen::Quaterniond q0,q1,q2;
  // q0 = qder * qtr * qdir1;
  // WpMarker.pose.orientation.w = q0.w();
  // WpMarker.pose.orientation.x = q0.x();
  // WpMarker.pose.orientation.y = q0.y();
  // WpMarker.pose.orientation.z = q0.z();  //leg
  // WpMarker.pose.position.x      = new_oodom[0] + 1.0 * sin(odom_yaw) + 0.5*d * cos(odom_yaw);
  // WpMarker.pose.position.y      = new_oodom[1] - 1.0 * cos(odom_yaw) + 0.5*d * sin(odom_yaw);
  // WpMarker.pose.position.z      = jump_height + 0.8*d   - z_buf;
  // WpMarker.mesh_resource      = mesh_resource2;
  // visugv_pub.publish(WpMarker);

  // q1 = qder * qtr ;//* qdir1;
  // WpMarker.pose.orientation.w = q1.w();
  // WpMarker.pose.orientation.x = q1.x();
  // WpMarker.pose.orientation.y = q1.y();
  // WpMarker.pose.orientation.z = q1.z();
  // WpMarker.mesh_resource      = mesh_resource;   //height_pure
  // WpMarker.pose.position.x      = new_oodom[0] + 0.2 * sin(odom_yaw);
  // WpMarker.pose.position.y      = new_oodom[1] - 0.2 * cos(odom_yaw);
  // WpMarker.pose.position.z      = jump_height - d + 0.5   -z_buf;
  // WpMarker.id               = 1;
  // visugv_pub.publish(WpMarker);

  // q2 = qder * qtr * qdir0;
  // WpMarker.pose.orientation.w = q2.w();
  // WpMarker.pose.orientation.x = q2.x();
  // WpMarker.pose.orientation.y = q2.y();
  // WpMarker.pose.orientation.z = q2.z();
  // WpMarker.mesh_resource      = mesh_resource3; // mid
  // WpMarker.pose.position.x      = new_oodom[0] + 1.0 * sin(odom_yaw) - 0.8*d * cos(odom_yaw);
  // WpMarker.pose.position.y      = new_oodom[1] - 1.0 * cos(odom_yaw) - 0.8*d * sin(odom_yaw);
  // WpMarker.pose.position.z      = jump_height - 1.4 * d   - z_buf;
  // WpMarker.id               = 2;
  // visugv_pub.publish(WpMarker);

  // q2 = qder * qtr * qdir2;
  // WpMarker.pose.orientation.w = q2.w();
  // WpMarker.pose.orientation.x = q2.x();
  // WpMarker.pose.orientation.y = q2.y();
  // WpMarker.pose.orientation.z = q2.z();
  // WpMarker.mesh_resource      = mesh_resource4; // wheels

  // double a = 2.7, r = 0.4;

  // WpMarker.pose.position.x      = new_oodom[0] + 1.0 * sin(odom_yaw) + 0.2 * d * cos(odom_yaw) + r * (cos(a) -  cos(a - w_ang - 2*d)) * cos(odom_yaw);
  // WpMarker.pose.position.y      = new_oodom[1] - 1.0 * cos(odom_yaw) + 0.2 * d * sin(odom_yaw) + r * (cos(a) -  cos(a - w_ang - 2*d)) * sin(odom_yaw);
  // WpMarker.pose.position.z      = jump_height    - z_buf + r * (sin(a) -  sin(a - w_ang - 2*d));
  // WpMarker.id               = 3;
  // visugv_pub.publish(WpMarker);   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ugv_vis_node");
  ros::NodeHandle nh("~");
  UGVVis diablo;
  diablo.init(nh);
  ros::spin();
  return 0;
}
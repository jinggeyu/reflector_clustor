#ifndef REFLECTOR_EXTRACT_H_
#define REFLECTOR_EXTRACT_H_

#include <ros/ros.h>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "reflector_locate/reflector_msgs.h"

class ReflectorExtract
{
 public:
  ReflectorExtract();
  ~ReflectorExtract();
  bool init();
  void clusterPointCallback(const sensor_msgs::PointCloud2 &msg);
//  void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher reflector_pub_;

  // ROS Topic Subscribers
  ros::Subscriber pointcloud_sub_;
};

#endif 

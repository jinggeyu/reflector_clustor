#ifndef CLUSTER_CLOUDPOINT_H_
#define CLUSTER_CLOUDPOINT_H_

#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h> 

#define pi 3.1415926
 typedef pcl::PointXYZ PointT;
boost::random::mt19937 randomGen;

//用pcl显示工具分别显示输入点云和聚类后的点云
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("cloud_input"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("final_result"));

class ClusterCloudpoint
{
 public:
    ClusterCloudpoint();
    ~ClusterCloudpoint();
    void init();
    void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg);
private:
    ros::NodeHandle nh;
    ros::Publisher clusterPointPub;
    ros::Subscriber pointcloudSub;
};

bool enforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance);
double getAngleOfCluster(const pcl::PointCloud<pcl::PointXYZI> &cloud );
void sortClusterClockwise(std::vector< pcl::PointCloud<pcl::PointXYZI> > &input_cloud, std::vector<double> &angleIndex);
void addClusterCloud(const std::vector< pcl::PointCloud<pcl::PointXYZI> > &input_cloud, pcl::PointCloud<pcl::PointXYZI> &output_cloud);

#endif
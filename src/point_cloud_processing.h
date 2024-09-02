#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/median_filter.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#include <vector>
#include <chrono>

pcl::PointCloud<pcl::PointXYZ>::Ptr helperSORFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr helperDenoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr helperDistanceFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud, double minDist, double maxDist);
void helperGroundFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& groundPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr& nonGroundPoints);
void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& inputCloudMsg, ros::Publisher& pub_ground, ros::Publisher& pub_nonGround);

#endif // POINT_CLOUD_FILTER_H


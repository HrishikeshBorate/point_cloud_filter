// MEDIAN FILTERING AND RANSAC
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/median_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub_ground;     // Ground points
ros::Publisher pub_nonGround;  // Non-ground points

pcl::PointCloud<pcl::PointXYZ>::Ptr helperMedianFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr medianFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MedianFilter<pcl::PointXYZ> medianFilter;
    medianFilter.setInputCloud(ptCloud);
    medianFilter.setWindowSize(5);      
    medianFilter.filter(*medianFilteredCloud);
    
    return medianFilteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr helperDistanceFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud, 
    double minDist, double maxDist) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr distanceFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < ptCloud->points.size(); i++) {
        const auto& point = ptCloud->points[i];
        double distanceXY = std::sqrt(point.x*point.x + point.y*point.y);

        if (distanceXY >= minDist && distanceXY <= maxDist) {
            distanceFilteredCloud->points.push_back(point);
        }
    }

    return distanceFilteredCloud;
}

void helperGroundFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& groundPoints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& nonGroundPoints) {
    
    // Ground plane segmentation using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr groundInliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);
    seg.setInputCloud(ptCloud);
    seg.segment(*groundInliers, *coefficients);

    if (groundInliers->indices.empty()) {
        std::cerr << "No ground plane found." << std::endl;
        return;
    }

    // Extract the ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(ptCloud);
    extract.setIndices(groundInliers);
    extract.setNegative(false);
    extract.filter(*groundPoints);

    // Extract non-ground points
    extract.setNegative(true);
    extract.filter(*nonGroundPoints);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& inputCloudMsg) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*inputCloudMsg, *ptCloud);

    // // TODO: CONSIDER REMOVING THIS Apply Median Filtering 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr medianFilteredPtCloud = helperMedianFiltering(ptCloud);

    // // Apply Distance Filtering
    // pcl::PointCloud<pcl::PointXYZ>::Ptr distanceFilteredPtCloud = helperDistanceFiltering(medianFilteredPtCloud, 5, 30);
    
    // Apply Distance Filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr distanceFilteredPtCloud = helperDistanceFiltering(ptCloud, 2, 30);

    // Apply Ground Filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundPoints(new pcl::PointCloud<pcl::PointXYZ>());
    helperGroundFiltering(distanceFilteredPtCloud, groundPoints, nonGroundPoints);

    // Publish ground points
    sensor_msgs::PointCloud2 outputGround;
    pcl::toROSMsg(*groundPoints, outputGround);
    outputGround.header = inputCloudMsg->header;
    pub_ground.publish(outputGround);

    // Publish non-ground points
    sensor_msgs::PointCloud2 outputNonGround;
    pcl::toROSMsg(*nonGroundPoints, outputNonGround);
    outputNonGround.header = inputCloudMsg->header;
    pub_nonGround.publish(outputNonGround);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    // Subscribe to the input point cloud topic
    ros::Subscriber sub = nh.subscribe("/mbuggy/os3/points", 1, cloudCallback);

    // Publishers for different outputs
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
    pub_nonGround = nh.advertise<sensor_msgs::PointCloud2>("/non_ground_points", 1);

    ros::spin();

    return 0;
}

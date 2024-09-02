#include "point_cloud_processing.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr helperSORFiltering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(ptCloud);
    sor.setMeanK(75);
    sor.setStddevMulThresh(3.0);
    sor.filter(*sorFilteredCloud);
    
    return sorFilteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr helperDenoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ptCloud){
    // int K = 6;
    // int threshold = 2;
    // int K = 8;
    // int threshold = 4;

    int K = 60;
    double stdDThreshold = 1.2;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(ptCloud);
    
    std::vector<double> meanDist;
    meanDist.reserve(ptCloud->points.size());

    for (std::size_t i = 0; i < ptCloud->points.size(); ++i) {
        std::vector<int> pointIdxKNNSearch(K + 1);  // Querying K+1 to exclude the point itself
        std::vector<float> pointKNNSquaredDistance(K + 1);

        if (kdtree.nearestKSearch(ptCloud->points[i], K + 1, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
            double dist = 0.0;
            for (std::size_t j = 1; j < pointKNNSquaredDistance.size(); ++j) {
                dist += std::sqrt(pointKNNSquaredDistance[j]);  // Taking sqrt to get actual distances
            }
            double meanDistance = dist / K;
            meanDist.push_back(meanDistance);
        } else {
            meanDist.push_back(std::numeric_limits<double>::quiet_NaN());
        }
    }

    // Remove NaN values and calculate mean and standard deviation
    double minDistance = 10000; double maxDistance = 0;
    std::vector<double> finiteMeanDist;
    for (const auto& dist : meanDist) {
        if (std::isfinite(dist)) {
            finiteMeanDist.push_back(dist);
            minDistance = std::min(dist, minDistance);
            maxDistance = std::max(dist, maxDistance);
        }
    }
    std::cout << "Minimum Distance: " << minDistance << std::endl;
    std::cout << "Maximum Distance: " << maxDistance << std::endl;

    // Compute the distance threshold 
    double meanD = std::accumulate(finiteMeanDist.begin(), finiteMeanDist.end(), 0.0) / finiteMeanDist.size();
    double stdD = std::sqrt(std::inner_product(finiteMeanDist.begin(), finiteMeanDist.end(), finiteMeanDist.begin(), 0.0) / finiteMeanDist.size() - meanD * meanD);
    double distThreshold = meanD + stdDThreshold * stdD;
    std::cout << "Distance Threshold: " << distThreshold << std::endl;
    
    // Select the inlier points
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoisedPtCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t i = 0; i < meanDist.size(); ++i) {
        if (std::isfinite(meanDist[i]) && meanDist[i] <= distThreshold) {
            denoisedPtCloud->points.push_back(ptCloud->points[i]);
        }
    }
    denoisedPtCloud->width = static_cast<uint32_t>(denoisedPtCloud->points.size());
    denoisedPtCloud->height = 1;
    denoisedPtCloud->is_dense = true;

    return denoisedPtCloud;
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
    
    // Ground plane segmentation using RANSAC algorithm
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
        ROS_WARN("No ground plane found.");
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

void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& inputCloudMsg, ros::Publisher& pub_ground, ros::Publisher& pub_nonGround) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*inputCloudMsg, *ptCloud);

    // Apply Distance Filtering
    auto distFilterStartTime = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr distanceFilteredPtCloud = helperDistanceFiltering(ptCloud, 5, 100);
    auto distFilterEndTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> distFilterRunTime = distFilterEndTime - distFilterStartTime;
    std::cout << "Time taken to perform distance filtering: " << distFilterRunTime.count() << " seconds" << std::endl;
    
    // // Perform SOR Filtering 
    // auto sorStartTime = std::chrono::high_resolution_clock::now();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr denoisedPtCloud = helperSORFiltering(distanceFilteredPtCloud);
    // auto sorEndTime = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> sorRunTime = sorEndTime - sorStartTime;
    // std::cout << "Time taken to perform sor filtering: " << sorRunTime.count() << " seconds" << std::endl;

    // Perform Denoising of the point cloud
    auto denoisingStartTime = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoisedPtCloud = helperDenoise(distanceFilteredPtCloud);
    auto denoisingEndTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> denoisingRunTime = denoisingEndTime - denoisingStartTime;
    std::cout << "Time taken to perform denoising: " << denoisingRunTime.count() << " seconds" << std::endl;

    // Apply Ground Filtering
    auto gndFilterStartTime = std::chrono::high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundPoints(new pcl::PointCloud<pcl::PointXYZ>());
    helperGroundFiltering(denoisedPtCloud, groundPoints, nonGroundPoints);
    
    auto gndFilterEndTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> gndFilterRunTime = gndFilterEndTime - gndFilterStartTime;
    std::cout << "Time taken to perform ground filtering cloud: " << gndFilterRunTime.count() << " seconds" << std::endl;

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

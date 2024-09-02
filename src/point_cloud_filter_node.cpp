#include "point_cloud_processing.h"
#include <std_srvs/SetBool.h>

class PlaybackController
{
public:
    PlaybackController()
    {
        ROS_INFO("PlaybackController Constructor Start");

        // Publishers for different outputs
        pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(gndPointsTopic, 1);
        pub_nonGround_ = nh_.advertise<sensor_msgs::PointCloud2>(nonGndPointsTopic, 1);

        // Subscribe to the point cloud topic
        sub_ = nh_.subscribe(pointCloudTopic, 1, &PlaybackController::cloudCallback, this);

        // Service client for pausing and unpausing the playback
        pause_client_ = nh_.serviceClient<std_srvs::SetBool>(playbackTopic);

        is_paused_ = false;

        // Wait for the service to be available
        while (!pause_client_.waitForExistence(ros::Duration(5.0))) {
            ROS_WARN("Waiting for /rosbag/pause_playback service to be available...");
        }

        ROS_INFO("PlaybackController Constructor End");
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& inputCloudMsg) {
        ROS_INFO("Processing point cloud...");
        processPointCloud(inputCloudMsg, pub_ground_, pub_nonGround_);

        // After processing, pause the playback if not already paused
        if (!is_paused_) {
            std_srvs::SetBool srv;
            srv.request.data = true;  // To pause the playback
            if (pause_client_.call(srv)) {
                if (srv.response.success) {
                    is_paused_ = true;
                    ROS_INFO("Paused playback. Waiting for processing to complete...");
                } else {
                    ROS_WARN("Service call succeeded, but response indicates failure: %s", srv.response.message.c_str());
                }
            } else {
                ROS_ERROR("Failed to call service /rosbag/pause_playback");
            }
        } else {
            ROS_INFO("Playback is already paused.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_nonGround_;
    ros::ServiceClient pause_client_;
    bool is_paused_; 

    std::string pointCloudTopic = "/mbuggy/os3/points";
    std::string gndPointsTopic = "/ground_points";
    std::string nonGndPointsTopic = "/non_ground_points";
    std::string playbackTopic = "/rosbag/pause_playback";
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processing");
    
    ROS_INFO("Starting point_cloud_processing");

    // Initialize the PlaybackController which handles point cloud processing
    PlaybackController controller;
    ros::spin();

    return 0;
}

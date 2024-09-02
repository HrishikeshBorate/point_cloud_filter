# Point Cloud Filtering
This repository contains code related to filtering noise from a point cloud, and identifying the ground and non-ground points in the denoised point cloud.

Documentation: https://docs.google.com/document/d/1VOYkYyMB4yNuOl4p9EPLalVLu4yvOBJhL_GHxsejSXc/edit#heading=h.wve1cmpj6obx

Follow these steps to run the code:

1) Start Docker Desktop
   
3) Navigate to the directory with LiDARFilteringAssignment.bag file. 
For eg., if the LiDARFilteringAssignment.bag file is present in ~/Documents/Library/TII, 
run the following command:
docker run -it --rm -v ~/Documents/Library/TII:/catkin_ws/ -p 6080:80 hrishikeshb185/tii-assignment:version1

4) Open a browser and go to the following URL: http://127.0.0.1:6080/. Click “Connect”.

5) Update the point cloud topic, update line 62 in /catkin_ws/src/point_cloud_filter/src/point_cloud_filter_node.cpp to any of the following:
std::string pointCloudTopic = "/mbuggy/os1/points";
std::string pointCloudTopic = "/mbuggy/os2/points";
std::string pointCloudTopic = "/mbuggy/os3/points";

4) Open terminal. Run the following in the terminal:
cd /catkin_ws/
chmod -R a+rwX /catkin_ws
mkdir src; cd src;
sudo git clone https://github.com/HrishikeshBorate/point_cloud_filter.git --branch pointCloudProcessing
cd ..
source devel/setup.bash
catkin_make
cd src/point_cloud_filter
roslaunch point_cloud_filter point_cloud_filter.launch

Then, click Spacebar key.

Note: Update the rviz visualization topic to the one corresponding to the topic in step 4.

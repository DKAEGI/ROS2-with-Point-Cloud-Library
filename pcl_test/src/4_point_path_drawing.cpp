#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <iostream>
#include <algorithm>

int main() {
    // Load plane.pcd
    std::string input_cloud = "plane.pcd";
	std::string path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/";
    std::string pcd_file_path = path + input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud);

    // Start the pcl_viewer
    pcl::visualization::PCLVisualizer viewer("Point Cloud with a simple Path");
    viewer.addPointCloud(cloud,"Ground Plane ");

    // Find min and max values of x and y coordinates
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::min();

    for (const pcl::PointXYZ& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    // Create a ball and place it into the cloud
    float ball_radius = 0.2;
    float ball_x = min_x + 1.2;
    float ball_y = min_y;

    // Create a path with balls
    int i=0;
    while (ball_y <= max_y) {
        pcl::PointXYZ point;
        point.x =ball_x;
        point.y=ball_y;

        viewer.addSphere(point,0.1,0.0,1.0,0.0,"Sphere"+std::to_string(i++));
        ball_y += ball_radius * 1.5;

    }

    // Spin to keep viewer running
    viewer.spin();


    return 0;
}

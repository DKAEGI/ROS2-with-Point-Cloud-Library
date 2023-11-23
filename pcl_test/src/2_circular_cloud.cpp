#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>  // Add this header for PCD file I/O

int main() {
    // Create a PointCloud object to store the points with XYZRGB information
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Define parameters for the circular point cloud
    int num_points = 360;  // Number of points in the circle
    double radius = 1.0;   // Radius of the circle

    for (int i = 0; i < num_points; ++i) {
        // Calculate the angle for each point on the circle
        double angle = 2.0 * M_PI * i / num_points;

        // Calculate the X and Y coordinates for the point
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        // Create a point with XYZ coordinates and a unique RGB color
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        point.z = 0.0;  // All points are at Z=0 in a 2D plane

        // Assign a unique RGB color to each point based on its position in the circle
        uint8_t r = static_cast<uint8_t>(i % 255);
        uint8_t g = static_cast<uint8_t>((2 * i) % 255);
        uint8_t b = static_cast<uint8_t>((3 * i) % 255);
        point.r = r;
        point.g = g;
        point.b = b;

        // Add the point to the point cloud
        cloud->push_back(point);
    }

    // Define the path where the PCD file will be saved
    std::string path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/circular_point_cloud.pcd";

    // Save the PointCloud data to a PCD file in binary format
    pcl::io::savePCDFileBinary(path, *cloud);

    std::cout << "PointCloud saved to " << path << std::endl;

    return 0;
}

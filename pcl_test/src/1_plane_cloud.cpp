#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main()
{
    // Create a PointCloud object of type pcl::PointXYZ to store 3D points
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Add five 3D points to the cloud
    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    cloud.push_back(pcl::PointXYZ(7.0, 8.0, 9.0));
    cloud.push_back(pcl::PointXYZ(10.0, 11.0, 12.0));
    cloud.push_back(pcl::PointXYZ(12.0, 14.0, -15.0));

    // Define the path where the PCD file will be saved
    std::string path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/5_points.pcd";

    // Save the PointCloud data to a PCD file in ASCII format
    pcl::io::savePCDFileASCII(path, cloud);

    // Print the number of points in the cloud
    std::cout << "Number of points: " << cloud.size() << std::endl;

    // Return 0 to indicate successful execution
    return 0;


}
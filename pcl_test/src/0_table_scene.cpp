#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <filesystem>

int main() {
    // Create a pointer to a PCLPointCloud2 object to store the point cloud data
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());

    // Create a PCDReader object to read PCD files
    pcl::PCDReader cloud_reader;

    // Define the name of the PCD file to read
    std::string pcd_file_name = "table_scene.pcd";

    // Get the current working directory
    std::filesystem::path ros2_ws_path = std::filesystem::current_path();

    // Define the directory where the point cloud data is located
    std::filesystem::path point_cloud_dir = ros2_ws_path / "pcl_test/point_clouds_data/";

    // Create the full path to the PCD file
    std::string pcd_file_path = point_cloud_dir / pcd_file_name;

    // Read the PCD file into the PCLPointCloud2 object
    cloud_reader.read(pcd_file_path, *cloud);

    // Print the number of points in the point cloud
    std::cout << "Number of points: " << cloud->width * cloud->height << std::endl;

    // Return 0 to indicate successful execution
    return 0;
}

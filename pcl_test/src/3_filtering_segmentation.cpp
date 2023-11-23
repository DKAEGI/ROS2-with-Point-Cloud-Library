#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <fstream>

using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;

// Function to save a point cloud to a PCD file
void cloud_saver(const std::string& file_name, std::string& path, pcl::PointCloud<PointXYZ>::Ptr cloud_arg) {
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<PointXYZ>(path + std::string(file_name), *cloud_arg);
}

// Function for cylinder segmentation
void segmentCylinder(const PointCloudXYZ::Ptr& input_cloud,
                     PointCloudXYZ::Ptr& cloud_cylinder,
                     PointCloudXYZ::Ptr& remaining_cloud) {
    // Estimate point normals
    pcl::NormalEstimation<PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(input_cloud);
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>());
    normal_estimator.setSearchMethod(tree);
    PointCloudNormal::Ptr cloud_normals(new PointCloudNormal);
    normal_estimator.setKSearch(10); // not so good with 30 or 50
    normal_estimator.compute(*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set up
    pcl::SACSegmentationFromNormals<PointXYZ, pcl::Normal> cylinder_segmentor;
    cylinder_segmentor.setOptimizeCoefficients(true);
    cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentor.setNormalDistanceWeight(0.5); // with 0.5 ok
    cylinder_segmentor.setMaxIterations(10000);
    cylinder_segmentor.setDistanceThreshold(0.06); // good is 0.05 until 0.06
    cylinder_segmentor.setRadiusLimits(0.1, 0.4);
    cylinder_segmentor.setInputCloud(input_cloud);
    cylinder_segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    cylinder_segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder cloud
    cloud_cylinder.reset(new PointCloudXYZ);
    pcl::ExtractIndices<PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    // Now, extract the rest of the cloud excluding the cylinder
    remaining_cloud.reset(new PointCloudXYZ);
    extract.setNegative(true); // Extract everything else but the cylinder points
    extract.filter(*remaining_cloud);
}


int main() {
    // *** READING THE CLOUD DATA *** 
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>);
    pcl::PCDReader cloud_reader;
    std::string path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/";
    std::string input_cloud = "tb3_world_v2.pcd";
    cloud_reader.read(path + input_cloud, *cloud);


    // *** VOXEL FILTER ***
    // Downsampling - noise reduction - smoothing
    // Create a new point cloud object to store the filtered point cloud
    pcl::PointCloud<PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<PointXYZ>); // New point cloud object to store the filtered point cloud
    pcl::VoxelGrid<PointXYZ> voxel_filter; // Instance of the VoxelGrid filter
    voxel_filter.setInputCloud(cloud); // Set the input point cloud for the voxel filter
    voxel_filter.setLeafSize(0.05, 0.05, 0.05); // Define the size of each voxel in x, y, and z dimensions (0.05 units each)
    voxel_filter.filter(*voxel_cloud); // Apply the voxel filter to the input cloud and store the result in voxel_cloud


    // *** PASS THROUGH FILTER ***
    // Filter through x and y axis
    pcl::PointCloud<PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<PointXYZ>);
    pcl::PassThrough<PointXYZ> passing_filter;
    passing_filter.setInputCloud(voxel_cloud);
    passing_filter.setFilterFieldName("x"); // can be x, y or z
    passing_filter.setFilterLimits(-1.7, 1.7);
    passing_filter.filter(*passthrough_cloud);
    passing_filter.setInputCloud(passthrough_cloud); // Update input cloud for the next passing filter
    passing_filter.setFilterFieldName("y");
    passing_filter.filter(*passthrough_cloud);

    cloud_saver("roi_passthrough.pcd", path, passthrough_cloud); // Save cloud data


    // *** PLANE SEGMENTATION ***
    // For object and obstacle recognition   
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // Pointer to store point indices that belong to the detected plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // Pointer to store model coefficients of the detected plane (e.g., normal vector and distance to origin)
    pcl::PointCloud<PointXYZ>::Ptr plane_segmented_cloud(new pcl::PointCloud<PointXYZ>); // New point cloud object to store the segmented plane
    pcl::SACSegmentation<PointXYZ> plane_segmentor; // Instance of SACSegmentation (RANSAC) for plane segmentation
    pcl::ExtractIndices<PointXYZ> indices_extractor; // Instance of ExtractIndices filter to extract the segmented plane

    plane_segmentor.setInputCloud(passthrough_cloud); // Set input cloud for plane segmentation
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE); // Set model type to "plane" for plane segmentation
    plane_segmentor.setMethodType(pcl::SAC_RANSAC); // Set method type to RANSAC for robust plane fitting
    plane_segmentor.setDistanceThreshold(0.01); // Set distance threshold for determining inliers to the plane
    plane_segmentor.segment(*inliers, *coefficients); // Perform plane segmentation and store inliers and model coefficients

    indices_extractor.setInputCloud(passthrough_cloud); // Set input cloud for ExtractIndices filter
    indices_extractor.setIndices(inliers); // Set indices to be extracted as inliers detected as part of the plane
    indices_extractor.setNegative(false); // Set "negative" flag to false to extract the plane (non-negative values), true would filter the circles of cylinders
    indices_extractor.filter(*plane_segmented_cloud); // Filter the original point cloud to obtain the segmented plane

    cloud_saver("plane.pcd", path, plane_segmented_cloud); // Save cloud data


    // *** MULTIPLE CYLINDER SEGMENTATION ***
    PointCloudXYZ::Ptr all_cloud_cylinder(new PointCloudXYZ);
    PointCloudXYZ::Ptr cloud_cylinder(new PointCloudXYZ);
    PointCloudXYZ::Ptr remaining_cloud(new PointCloudXYZ);
    *remaining_cloud = *passthrough_cloud;

    // Path for cylinder clouds
    path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/cylinder_clouds/";

    // Expected cylinders less than 20
    for (int i = 0; i < 20; ++i) {
        // Perform segmentation
        segmentCylinder(remaining_cloud, cloud_cylinder, remaining_cloud);

        if (cloud_cylinder->size() == 0){
            std::cout << "No more cylinders found" << std::endl;
            break;
        }

        // Save each cylinder cloud
        std::stringstream ss_cylinder;
        ss_cylinder << "cylinder_cloud_" << i << ".pcd";
        cloud_saver(ss_cylinder.str(), path, cloud_cylinder);
        std::cout << "Number of points of cylinder_cloud_" << i << " are " << cloud_cylinder->size() << std::endl;

        // Save remaining cloud
        std::stringstream ss_remaining;
        ss_remaining << "remaining_cloud_" << i << ".pcd";
        cloud_saver(ss_remaining.str(), path, remaining_cloud);

        // Put all cylinders together
        *all_cloud_cylinder += *cloud_cylinder;
    }

    // Save all cylinders in one file
    path = "/home/dkae/Workspaces/ros2/src/ROS2-Point-Cloud-Library/pcl_test/point_clouds_data/";
    cloud_saver("all_cylinders.pcd", path, all_cloud_cylinder);

    return 0;
}

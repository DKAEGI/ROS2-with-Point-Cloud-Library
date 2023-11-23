#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <iostream>

int main()
{
    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate random points and add them to the point cloud
    for (int i = 0; i < 100; i++)
    {
        pcl::PointXYZ p;
        p.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        p.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        p.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        cloud->push_back(p);
    }

    // Create a KD-Tree object and set the input cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Define a search point
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.6f;
    searchPoint.y = 0.6f;
    searchPoint.z = 0.6f;

    // Define the number of nearest neighbors to search for
    int K = 5;

    // Initialize vectors to store indices and distances of the nearest neighbors
    std::vector<int> indices(K);
    std::vector<float> distances(K);

    // Perform the nearest neighbor search
    kdtree.nearestKSearch(searchPoint, K, indices, distances);

    // Print the indices and distances of the nearest neighbors
    for (int i = 0; i < indices.size(); i++)
    {
        std::cout << "Nearest Neighbor " << i + 1 << ": Index " << indices[i] << ", Distance " << distances[i] << std::endl;
    }

    return 0;
}

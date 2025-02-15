#include"../lib.h"
class don
{
private:
    
public:
    void VoxelGrid_filter(pcl::PointCloud<PointXYZRGB>::Ptr cloud_,double leaf_size);
    void VoxelGrid_filter(pcl::PointCloud<PointXYZ>::Ptr cloud_,double leaf_size);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_pointcloud(std::vector<pcl::PointIndices> cluster_indices,
                                                                    PointCloud<PointNormal>::Ptr doncloud);

    void performEuclideanClustering(const pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud, 
                                float cluster_tolerance, 
                                int min_cluster_size, 
                                int max_cluster_size, 
                                std::vector<pcl::PointIndices>& cluster_indices);
                                

    void calculateMeanRadiusAndMultiply(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                        pcl::search::Search<PointXYZRGB>::Ptr tree, 
                                        double& scale1, double& scale2, double& segradius);
                                        
    void computeSurfaceNomalsForPointClouds( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_, 
                                         std::vector<pcl::PointCloud<pcl::Normal>::Ptr>& normals);

};

void performRegionGrowingClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, std::vector<pcl::PointIndices>& cluster_indices);

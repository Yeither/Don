#include "don.h"

/**
 * 使用VoxelGrid滤波器对点云进行降采样
 * @param cloud_ 输入点云的指针
 * @param leaf_size 体素大小
 */
void don::VoxelGrid_filter(pcl::PointCloud<PointXYZRGB>::Ptr cloud_,double leaf_size){
    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    // 设置输入点云
    voxel_grid.setInputCloud(cloud_);
    // 执行体素抽取滤波
    voxel_grid.filter(*cloud_);
}
void don::VoxelGrid_filter(pcl::PointCloud<PointXYZ>::Ptr cloud_,double leaf_size){
    // 创建VoxelGrid滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    // 设置输入点云
    voxel_grid.setInputCloud(cloud_);
    // 执行体素抽取滤波
    voxel_grid.filter(*cloud_);
}


/**
 * 获取聚类后的点云集合
 * @param cluster_indices 聚类索引的向量
 * @param doncloud 包含点云和法线信息的指针
 * @return 聚类后的点云集合的指针向量
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> don::get_pointcloud(std::vector<pcl::PointIndices> cluster_indices,PointCloud<PointNormal>::Ptr doncloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr doncloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*doncloud, *doncloud_xyz);

    for (const auto& cluster_indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& index : cluster_indices.indices)
        {
            cloud_cluster->points.push_back(doncloud_xyz->points[index]);
        }

        cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clouds_.push_back(cloud_cluster);
    }
    return clouds_;
}

/**
 * 执行欧几里德聚类算法,耗时过长
 * @param input_cloud 输入点云数据的指针
 * @param cluster_tolerance 聚类的容差范围
 * @param min_cluster_size 最小聚类大小
 * @param max_cluster_size 最大聚类大小
 * @param cluster_indices 存储聚类索引的向量
 */
void don::performEuclideanClustering(const pcl::PointCloud<pcl::PointNormal>::Ptr& input_cloud, 
                                    float cluster_tolerance, 
                                    int min_cluster_size, 
                                    int max_cluster_size, 
                                    std::vector<pcl::PointIndices>& cluster_indices)
{
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    tree->setInputCloud(input_cloud);
    ec.setSearchMethod(tree);

    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

//cluster_indices输出
//     for (int i = 0; i < cluster_indices.size(); ++i) {
//     std::cout << "Cluster " << i << " : "<<endl;
//     for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
//         std::cout << cluster_indices[i].indices[j] << " "<< std::endl;
//     }
// }
}

/**
 * 计算点云的平均半径并进行缩放
 * @param cloud 点云数据的指针
 * @param tree 搜索树对象的指针
 * @param scale1 第一个尺度的缩放因子
 * @param scale2 第二个尺度的缩放因子
 * @param segradius 分割半径的缩放因子
 */
void don::calculateMeanRadiusAndMultiply(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                        pcl::search::Search<PointXYZRGB>::Ptr tree, 
                                        double& scale1,double& scale2, double& segradius){
    int size_cloud = cloud->size();
    int step = size_cloud / 10;
    double total_distance = 0;
    int i, j = 1;
    for (i = 0; i < size_cloud; i += step, j++){
        std::vector<int> pointIdxNKNSearch(2);
        std::vector<float> pointNKNSquaredDistance(2);
        tree->nearestKSearch(cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance);
        total_distance += pointNKNSquaredDistance[1] + pointNKNSquaredDistance[0];
    }
    double mean_radius = sqrt(total_distance / j);
    std::cout << "Mean radius of cloud is: " << mean_radius << std::endl;
    scale1 *= mean_radius; // 5 * mean_radius
    scale2 *= mean_radius; // 10 * mean_radius
    segradius *= mean_radius;

    if (scale1 >= scale2)
    {
        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

/**
 * 计算点云的平均半径，并根据平均半径的值对一些变量进行缩放
 * @param clouds_ 点云集合的指针向量
 * @param normals 存储计算得到的法线的指针向量
 */
void don::computeSurfaceNomalsForPointClouds( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_, 
                                         std::vector<pcl::PointCloud<pcl::Normal>::Ptr>& normals)
{  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointXYZ>);
	for (int it = 0; it < clouds_.size(); ++it)
	{
		for (const auto& point : clouds_[it]->points) {
		cloud_cluster_don->points.push_back(point);
	    }

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_cluster_don);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setKSearch(100); // 设置 k 邻域搜索的参数
        ne.compute(*cloud_normals);
        normals.push_back(cloud_normals);
	}
 
    // //normals输出
    // for (int i = 0; i < normals.size(); ++i) {
    // std::cout << "Normals for Point Cloud " << i << " : " << std::endl;
    // for (const auto& normal : normals[i]->points) {
    //     std::cout << "Normal: (" << normal.normal_x << ", " << normal.normal_y << ", " << normal.normal_z << ")" << std::endl;
    // }
    // std::cout << std::endl;
    // }
}

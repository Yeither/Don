#include "./view/view.h"
#include "tools.hpp"
#include "./don/don.h"

int main(int argc, char *argv[])
{
	//初始化设定
	string yaml_addr="./setting.yml";
	bool SAVE = LoadConfigFile<bool>(yaml_addr,"SAVE");
	bool Get_Nomal = LoadConfigFile<bool>(yaml_addr,"Get_Nomal");
	bool VoxelGrid_filter = LoadConfigFile<bool>(yaml_addr,"VoxelGrid_filter");
	double scale1 = LoadConfigFile<double>(yaml_addr,"scale1");
    double scale2 = LoadConfigFile<double>(yaml_addr,"scale2");
	double segradius = LoadConfigFile<double>(yaml_addr,"segradius");
	double leaf_size = LoadConfigFile<double>(yaml_addr,"leaf_size");
    string PCD_cloud_addr = LoadConfigFile<string>(yaml_addr,"address");
	string save_address = LoadConfigFile<string>(yaml_addr,"save_address");
	double mean_radius;
	don don;
	pcl::PointCloud<PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<PointXYZRGB>);
	pcl::io::loadPCDFile(PCD_cloud_addr, *cloud_);

    // 获取开始时间点
    auto start = std::chrono::high_resolution_clock::now();
    
	if(VoxelGrid_filter)
    	don.VoxelGrid_filter(cloud_,leaf_size);
	pcl::search::Search<PointXYZRGB>::Ptr tree;
	if (cloud_->isOrganized())
		tree.reset(new pcl::search::OrganizedNeighbor<PointXYZRGB>());
	else
		tree.reset(new pcl::search::KdTree<PointXYZRGB>(false));

	// 设置搜索树的输入点云
	tree->setInputCloud(cloud_);

	//计算点云的平均半径，并与相应的输入进行乘法运算
	don.calculateMeanRadiusAndMultiply(cloud_, tree, scale1, scale2, segradius);

	// 在每个点上使用小尺度和大尺度计算法线
	pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
	ne.setInputCloud(cloud_);
	ne.setSearchMethod(tree);
	
	// 设置视点，确保法线都指向同一个方向
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    // 获取开始时间点
    auto start1 = std::chrono::high_resolution_clock::now();
	// 使用小尺度计算法线
	cout << "Calculating normals for scale1..." << scale1 << endl;
	cout << "Calculating normals for scale2..." << scale2 << endl;
	cout << "segradius  "<<segradius<<endl;
	pcl::PointCloud<PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<PointNormal>);
    std::thread t(setNormalEstimationOMP,ne, scale1, normals_small_scale);
    std::thread t1(setNormalEstimationOMP,ne, scale2, normals_large_scale);
	// 使用大尺度计算法线
    t.join();
    t1.join();

	auto start2 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(start2 - start1);
    std::cout << "Time taken by function: " << ((double)duration1.count()/1000000)<< " seconds" << std::endl;
	// 创建用于存储DoN结果的输出点云
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	copyPointCloud<PointXYZRGB, PointNormal>(*cloud_, *doncloud);

	cout << "Calculating DoN... " << endl;
	// 创建DoN算子
	pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> Don;
	Don.setInputCloud(cloud_);
    
	Don.setNormalScaleLarge(normals_large_scale);
	Don.setNormalScaleSmall(normals_small_scale);

	if (!Don.initCompute())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}

	//对输入点集，计算每一个点的DON特征向量，并输出
	Don.computeFeature(*doncloud);

	//输出一些不同的曲率
	{
		cout << "You may have some sense about the input threshold（curvature） next time for your data" << endl;
		int size_cloud = doncloud->size();
		int step = size_cloud / 10;
		for (int i = 0; i < size_cloud; i += step)cout << " " << doncloud->points[i].curvature << " " << endl;
	}

	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointNormal> ec;

	//聚类，耗时
	std::cout<<"Clustering..."<<endl;
	don.performEuclideanClustering(doncloud,segradius,50,100000,cluster_indices);
	std::cout<<"size of cluster_indices: "<<cluster_indices.size()<<std::endl;

	//储存分割后的点云
	if (SAVE)   save_pointcloud(cluster_indices,doncloud,save_address);

	//点云分割
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
	clouds_ = don.get_pointcloud(cluster_indices,doncloud);

	//这个函数能计算每个点的法向量感觉暂时用不到就没有启用
	if (Get_Nomal){
	std::vector<pcl::PointCloud<pcl::Normal>::Ptr> normals;
	don.computeSurfaceNomalsForPointClouds(clouds_, normals);
	}
	
    // 获取结束时间点
    auto stop = std::chrono::high_resolution_clock::now();

    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by function: " << ((double)duration.count()/1000000) << " seconds" << std::endl;

	cout<<"press any key to continue..."<<endl;
	getchar();
	if(!VoxelGrid_filter)
		for(auto cloud_:clouds_)don.VoxelGrid_filter(cloud_,leaf_size);

	// 显示结果点云
	view view_tool;
	view_tool.ransac(clouds_,cluster_indices,doncloud);
	view_tool.set_color(cluster_indices,doncloud);
	view_tool.ParallelFilter();
	view_tool.final_viewer();
}

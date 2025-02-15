#include "lib.h"

pcl::PCDWriter writer;
/**
 * 储存聚类后的点云数据
 * @param cluster_indices 聚类的索引集合
 * @param doncloud 原始点云数据
 * @return 储存的聚类数量
 */
int save_pointcloud(std::vector<pcl::PointIndices> cluster_indices, PointCloud<PointNormal>::Ptr doncloud,string save) {
    int j = 0;

    // 遍历每个聚类
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++) {
        // 创建一个新的点云对象，用于储存当前聚类的点云数据
        pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don_save(new pcl::PointCloud<PointNormal>);

        // 将当前聚类的点云数据添加到新的点云对象中
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster_don_save->points.push_back(doncloud->points[*pit]);
        }

        // 设置新点云对象的宽度、高度和是否稠密
        cloud_cluster_don_save->width = int(cloud_cluster_don_save->points.size());
        cloud_cluster_don_save->height = 1;
        cloud_cluster_don_save->is_dense = true;

        // 保存聚类的点云数据到PCD文件
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don_save->points.size() << " data points." << std::endl;
        stringstream ss;
        ss << save <<"/don_cluster_" << j << ".pcd";
        writer.write<pcl::PointNormal>(ss.str(), *cloud_cluster_don_save, false);
    }

    return j; // 返回储存的聚类数量
}

/**
 * 从YAML文件中读取配置数据
 * @tparam T 配置数据的类型
 * @param file_name YAML文件的路径
 * @param name 配置数据的名称
 * @param mod 配置数据的修饰符（可选）
 * @return 读取到的配置数据
 * @throws std::runtime_error 如果打开配置文件失败、配置文件中没有指定的数据或者配置文件中没有指定的转换项时抛出异常
 */
template<typename T>
T LoadConfigFile(const std::string &file_name, const std::string &name) {
    // 加载配置文件
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
        throw std::runtime_error("Open config File:" + file_name + " failed.");
    }

    // 检查配置文件中是否存在指定的转换项
    if (!config["DON"]) {
        throw std::runtime_error("Open config File:" + file_name + " has no transform.");
    }

    T data;
    // 读取指定的配置数据
    if (config["DON"]) {
        if (config["DON"][name]) {
            data = config["DON"][name].as<T>();
            return data;
        } 
        else {
            throw std::runtime_error("config File:" + file_name + " has no " + name + " value.");
        }
    } 
    else {
        throw std::runtime_error("config File:" + file_name + " has no transform:translation.");
    }
}

/**
 * 设置 NormalEstimationOMP 的参数并执行计算
 * @param ne NormalEstimationOMP 对象
 * @param scale1 搜索半径的比例尺度
 * @param normals_small_scale 存储计算得到的法线的点云对象
 */
void setNormalEstimationOMP(pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne, double scale1, pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale) {
    // 设置 NormalEstimationOMP 的线程数
    ne.setNumberOfThreads(16);

    // 设置搜索半径的比例尺度
    ne.setRadiusSearch(scale1);

    // 执行法线估计计算
    ne.compute(*normals_small_scale);
}

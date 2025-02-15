#include "../lib.h"

class view{
public:
view() : viewer("Point Cloud Viewer"), 
plane_count(0),
count(0),
clusterIndex(0),
coloredCluster(new pcl::PointCloud<pcl::PointXYZRGB>)  {};
void seePlane(const pcl::PointCloud<pcl::PointXYZRGB>& plane, const pcl::ModelCoefficients::Ptr& coefficients);
double get_scale(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,int i);
void seePlanes(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& coloredClusters, const std::vector<pcl::ModelCoefficients::Ptr>& planeCoefficients);
void ransac(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_,std::vector<pcl::PointIndices> cluster_indices,PointCloud<PointNormal>::Ptr doncloud);
void ParallelFilter();
bool areParallel(const pcl::ModelCoefficients::Ptr& plane1, const pcl::ModelCoefficients::Ptr& plane2);
bool isPlanePerpendicularToGround(const pcl::ModelCoefficients::Ptr& coefficients,float threshold);
double calculatePlaneDistance(const pcl::ModelCoefficients::Ptr& plane1, const pcl::ModelCoefficients::Ptr& plane2);
void set_color(std::vector<pcl::PointIndices> cluster_indices,PointCloud<PointNormal>::Ptr doncloud);
void final_viewer();
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& result);
private:

pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCluster;
std::vector<pcl::ModelCoefficients::Ptr> planeCoefficients; // 添加存储平面系数的向量
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> coloredClusters2;
pcl::PointXYZRGB planeCentroid;
std::vector<pcl::ModelCoefficients::Ptr> filteredPlaneCoefficients;
std::vector<int> is;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> coloredClusters;

pcl::visualization::PCLVisualizer viewer;
int plane_count;
int count;
int clusterIndex;
};

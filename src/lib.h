#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <mutex>
#include <iostream>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <pcl/common/concatenate.h>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "vtkSmartPointer.h"
#include "vtkPlaneSource.h"
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkPolyData.h>
#include <yaml-cpp/yaml.h>

using namespace pcl;
using namespace std;
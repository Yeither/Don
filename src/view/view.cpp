#include "view.h"

/**
 * 可视化平面模型
 * @param plane 包含点云数据的平面对象
 * @param coefficients 平面模型的系数
 */
void view::seePlane(const pcl::PointCloud<pcl::PointXYZRGB>& plane, const pcl::ModelCoefficients::Ptr& coefficients) {
    pcl::PointXYZRGB min_pt,max_pt;
    pcl::getMinMax3D(plane,min_pt,max_pt);
    // double scalee = FLAGS_plane_size;
    double length, width;
    double d_x = (max_pt.x - min_pt.x);
    double d_y = (max_pt.y - min_pt.y);
    double d_z = (max_pt.z - min_pt.z);

    double centerr[3];
    centerr[0] = min_pt.x + d_x/2.0;
    centerr[1] = min_pt.y + d_y/2.0;
    centerr[2] = min_pt.z + d_z/2.0;
    
    if (centerr[2] < 0.3) {
        double maxy_x, maxx_y, miny_x, minx_y;
        for (auto& point : plane.points){
            if(point.x == max_pt.x){
                maxx_y = point.y;
            }
            if(point.y == max_pt.y){
                maxy_x = point.x;
            }
            if(point.x == min_pt.x){
                minx_y = point.y;
            }
            if(point.y == min_pt.y){
                miny_x = point.x;
            }
        }
        length = sqrt((pow((max_pt.x-maxy_x),2) + pow((max_pt.y-maxx_y),2)));
        width = sqrt((pow((max_pt.y-minx_y),2) + pow((maxy_x-min_pt.x),2)));
    } else {
        length = sqrt((pow(d_x,2) + pow(d_y,2)));
        width = d_z;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(plane, centroid);

    vtkSmartPointer<vtkPlaneSource> plane_ = vtkSmartPointer<vtkPlaneSource>::New();
    double norm_sqr = 1.0 / (coefficients->values[0] * coefficients->values[0] +
                             coefficients->values[1] * coefficients->values[1] +
                             coefficients->values[2] * coefficients->values[2]);

    plane_->SetNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    double x = centroid(0);
    double y = centroid(1);
    double z = centroid(2);
    // double x = centerr[0];
    // double y = centerr[1];
    // double z = centerr[2];
    double t = x * coefficients->values[0] + y * coefficients->values[1] + z * coefficients->values[2] + coefficients->values[3];
    x -= coefficients->values[0] * t * norm_sqr;
    y -= coefficients->values[1] * t * norm_sqr;
    z -= coefficients->values[2] * t * norm_sqr;
    //while(z<-1)z+=3;
    plane_->SetCenter(x, y, z);

    {
        double pt1[3], pt2[3], orig[3],center[3];
        plane_->GetPoint1(pt1);
        plane_->GetPoint2(pt2);
        plane_->GetOrigin(orig);
        plane_->GetCenter(center);

        double u[3], v[3];
        for(int i = 0; i < 3; i++) {
            u[i] = (orig[i] - pt1[i]) / 2.0;
            v[i] = (orig[i] - pt2[i]) / 2.0;
            pt1[i] = center[i] - length * u[i] + width * v[i];
            pt2[i] = center[i] + length * u[i] - width * v[i];
            orig[i] = center[i] + length * u[i] + width * v[i];
        }
        
        plane_->SetOrigin(orig);
        plane_->SetPoint1(pt1);
        plane_->SetPoint2(pt2);
    }
    
    // vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // transform->RotateWXYZ(0, coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    
    // vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    // transformFilter->SetInputConnection(plane_->GetOutputPort());
    // transformFilter->SetTransform(transform);
    // transformFilter->Update();
    plane_->Update();
    viewer.addModelFromPolyData(plane_->GetOutput(), ("plane"+std::to_string(plane_count)));
}

/**
 * 获取点云数据在指定维度上的尺度
 * @param cloud 包含点云数据的指针
 * @param i 指定的维度索引
 * @return 点云数据在指定维度上的尺度
 */
double view::get_scale(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int i) {
    // 假设点云数据已经填充到 cloud 中
    
    pcl::PointXYZRGB minPt, maxPt;
    
    // 获取点云数据的最小和最大坐标
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    double scale[3] = {
        abs(minPt.x - maxPt.x),   // X轴上的尺度
        abs(minPt.y - maxPt.y),   // Y轴上的尺度
        abs(minPt.z - maxPt.z)    // Z轴上的尺度
    };
    
    return scale[i];
}

/**
 * 可视化多个平面模型
 * @param coloredClusters 包含彩色点云数据的平面对象的向量
 * @param planeCoefficients 平面模型的系数的向量
 */
void view::seePlanes(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& coloredClusters, const std::vector<pcl::ModelCoefficients::Ptr>& planeCoefficients) {
    for (size_t i = 0; i < planeCoefficients.size(); ++i) {
        cout << i << " value0 " << planeCoefficients[i]->values[0] << " " << "value1 " << planeCoefficients[i]->values[1] << " value2 " << planeCoefficients[i]->values[2] << " value3 " << planeCoefficients[i]->values[3] << endl;
        
        // 使用之前的 seePlane 函数可视化平面
        if ((abs(planeCoefficients[i]->values[3]) < 0.01)||(*coloredClusters[i]).size()<100) {
            continue;
        }
        
        seePlane(*coloredClusters[i], planeCoefficients[i]);
        plane_count++;
    }
}

/**
 * 使用RANSAC算法进行平面拟合
 * @param clouds_ 包含点云数据的向量
 * @param cluster_indices 点云聚类的索引
 * @param doncloud 包含点云数据和法向量的指针
 */
void view::ransac(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_, std::vector<pcl::PointIndices> cluster_indices, PointCloud<PointNormal>::Ptr doncloud) {
    // 遍历聚类索引向量
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // 遍历当前聚类的索引
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            pcl::PointXYZ point;
            point.x = doncloud->points[*pit].x;
            point.y = doncloud->points[*pit].y;
            point.z = doncloud->points[*pit].z;
            cluster->points.push_back(point);
        }

        if(cluster->points.size()<100){
            continue;
            cout<<"point"<<it->header<<" too small"<<endl;
            }

        // 使用RANSAC进行平面拟合
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cluster);
        seg.segment(*inliers, *coefficients);
        if((coefficients->values).empty())continue;
        // 计算聚类点云的质心
        pcl::PointXYZRGB clusterCentroid;
        pcl::computeCentroid(*clouds_[count], clusterCentroid);
        
        // 平移点云使平面中心与点云质心对齐
        // 将彩色聚类点云添加到 coloredClusters
        coloredClusters.push_back(convertPointCloud(clouds_[count]));
        
        // 将拟合的平面参数存储在 planeCoefficients 中
        planeCoefficients.push_back(coefficients);
        
        count++;
    }
}

/**
 * 平行平面过滤器
 */
void view::ParallelFilter() {
    for (size_t i = 0; i < planeCoefficients.size(); ++i) {
        bool isParallel = false;
        // 检查当前平面与之前的平面是否平行
        for (size_t j = 0; j < filteredPlaneCoefficients.size(); ++j) {
            // 比较法向量的方向来判断平面是否平行
            if (areParallel(planeCoefficients[i], filteredPlaneCoefficients[j])) {
                isParallel = true;
                break;
            }
        }

        // 如果没有与当前平面平行的平面，则将其保留
        if (!isParallel) {
            filteredPlaneCoefficients.push_back(planeCoefficients[i]);
            is.push_back(i);
        }
    }
};

/**
 * 判断两个平面是否平行
 * @param plane1 平面1的系数指针
 * @param plane2 平面2的系数指针
 * @return 如果平面平行，则返回true；否则返回false
 */
bool view::areParallel(const pcl::ModelCoefficients::Ptr& plane1, const pcl::ModelCoefficients::Ptr& plane2) {

    // 获取平面1的法向量
    double nx1 = plane1->values[0];
    double ny1 = plane1->values[1];
    double nz1 = plane1->values[2];

    // 获取平面2的法向量
    double nx2 = plane2->values[0];
    double ny2 = plane2->values[1];
    double nz2 = plane2->values[2];

    // 判断法向量是否平行
    double dotProduct = nx1 * nx2 + ny1 * ny2 + nz1 * nz2;
    double magnitude1 = std::sqrt(nx1 * nx1 + ny1 * ny1 + nz1 * nz1);
    double magnitude2 = std::sqrt(nx2 * nx2 + ny2 * ny2 + nz2 * nz2);
    double cosAngle = dotProduct / (magnitude1 * magnitude2);

    // 如果法向量之间的夹角接近于0度或180度，则认为平面平行
    const double threshold = 0.8; // 可根据需要调整阈值
    bool judge = ((std::fabs(cosAngle) >= threshold) && (calculatePlaneDistance(plane1, plane2) < 0.5)) || ((std::fabs(cosAngle) >= threshold) && (isPlanePerpendicularToGround(plane1, 10)));
    if(judge)cout<<"areParallel!"<<"  "<<((std::fabs(cosAngle) >= threshold) && (calculatePlaneDistance(plane1, plane2) < 0.4))<<" "<<((std::fabs(cosAngle) >= threshold) && (isPlanePerpendicularToGround(plane1, 10)))<<endl;
    return judge;
}

/**
 * 判断平面是否垂直于地面
 * @param coefficients 平面的系数指针
 * @param threshold 判断垂直性的阈值
 * @return 如果平面垂直于地面，则返回true；否则返回false
 */
bool view::isPlanePerpendicularToGround(const pcl::ModelCoefficients::Ptr& coefficients, float threshold) {
    // 地面法向量
    Eigen::Vector3f groundNormal(0, 1, 0);

    // 平面法向量
    Eigen::Vector3f planeNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // 计算两个向量之间的夹角的余弦值
    float cosAngle = groundNormal.dot(planeNormal) / (groundNormal.norm() * planeNormal.norm());

    // 将余弦值转换为角度
    float angle = std::acos(cosAngle) * 180.0 / M_PI;

    // 定义判断垂直性的阈值角度（例如，10度）

    // 检查角度是否接近于90度（在阈值范围内）
    return (std::abs(angle - 90.0) < threshold);
}

/**
 * 计算两个平面之间的距离
 * @param plane1 平面1的系数指针
 * @param plane2 平面2的系数指针
 * @return 两个平面之间的距离
 */
double view::calculatePlaneDistance(const pcl::ModelCoefficients::Ptr& plane1, const pcl::ModelCoefficients::Ptr& plane2) {
    // 提取平面1的法向量和原点到平面的距离
    double nx1 = plane1->values[0];
    double ny1 = plane1->values[1];
    double nz1 = plane1->values[2];
    double d1 = std::fabs(plane1->values[3]);

    // 提取平面2的法向量和原点到平面的距离
    double nx2 = plane2->values[0];
    double ny2 = plane2->values[1];
    double nz2 = plane2->values[2];
    double d2 = std::fabs(plane2->values[3]);

    // 计算两个平面之间的距离
    double distance = std::fabs((d2 - d1) / std::sqrt(nx1 * nx1 + ny1 * ny1 + nz1 * nz1 + nx2 * nx2 + ny2 * ny2 + nz2 * nz2));

    return distance;
}

/**
 * 为聚类设置颜色并可视化不同颜色的点云
 * @param cluster_indices 聚类的索引集合
 * @param doncloud 原始点云数据
 */
void view::set_color(std::vector<pcl::PointIndices> cluster_indices, PointCloud<PointNormal>::Ptr doncloud) {
    // 在聚类操作后，为每个聚类设置不同的颜色
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 为聚类生成随机颜色
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            pcl::PointXYZRGB point;
            point.x = doncloud->points[*pit].x;
            point.y = doncloud->points[*pit].y;
            point.z = doncloud->points[*pit].z;
            point.r = r;
            point.g = g;
            point.b = b;
            coloredCluster->points.push_back(point);
        }
        
        coloredClusters.push_back(coloredCluster);
    }

    // 可视化不同颜色的点云
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    for (const auto& coloredCluster : coloredClusters) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler(coloredCluster);
        viewer.addPointCloud<pcl::PointXYZRGB>(coloredCluster, colorHandler, "cluster" + std::to_string(clusterIndex));
        clusterIndex++;
    }
}

/**
 * 最终的可视化函数，将指定索引的聚类点云和平面系数可视化
 */
void view::final_viewer() {
    // 根据索引提取指定的聚类点云
    for (auto it : is) {
        coloredClusters2.push_back(coloredClusters[it]);
    }
    // 可视化聚类点云和平面系数
    seePlanes(coloredClusters2, filteredPlaneCoefficients);

    // 进行可视化展示
    viewer.spin();
}

/**
 * 将点云从pcl::PointXYZ类型转换为pcl::PointXYZRGB类型
 * @param result 待转换的点云数据（pcl::PointXYZ类型）
 * @return 转换后的点云数据（pcl::PointXYZRGB类型）
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr view::convertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& result) {
    // 创建一个新的pcl::PointXYZRGB类型的点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newResult(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 设置新点云对象的宽度、高度和是否稠密
    newResult->width = result->width;
    newResult->height = result->height;
    newResult->is_dense = result->is_dense;

    // 调整新点云对象的点数
    newResult->points.resize(result->points.size());

    // 遍历原始点云数据，将每个点的坐标和颜色（黑色）赋值给新点云对象
    for (size_t i = 0; i < result->points.size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = result->points[i].x;
        point.y = result->points[i].y;
        point.z = result->points[i].z;
        point.r = 0;  // 设置RGB颜色为黑色，可以根据需要修改颜色
        point.g = 0;
        point.b = 0;
        newResult->points[i] = point;
    }

    return newResult;
}

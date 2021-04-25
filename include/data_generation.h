#ifndef DATA_GENERATION_H
#define DATA_GENERATION_H
#include <iostream>
#include <thread>
#include <Eigen/Eigen>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
class DataGeneration
{
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr line_point_cloud_ref_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_point_cloud_ref_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr line_point_cloud_per_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_point_cloud_per_;
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d translation_;
public:
    DataGeneration();
    ~DataGeneration();
    void CloudViewer(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReturnLinePointCloudRef();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReturnLinePointCloudPer();
    void SavePointCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr LinePointCloudGeneration();
    pcl::PointCloud<pcl::PointXYZI>::Ptr PlanePointCloudGeneration();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReturnPlanePointCloudRef();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReturnPlanePointCloudPer();
};
#endif

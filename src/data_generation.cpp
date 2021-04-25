#include "data_generation.h"
DataGeneration::DataGeneration()
{
    std::cout << "Starting" << std::endl;
    line_point_cloud_ref_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    line_point_cloud_per_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    plane_point_cloud_ref_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    plane_point_cloud_per_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    rotation_ = Eigen::AngleAxisd(M_PI/10,Eigen::Vector3d::UnitZ());
    translation_ << -1.4,3.5,0;

}

DataGeneration::~DataGeneration()
{
}
// generating points in line
// line  y = 2
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::LinePointCloudGeneration()
{

    for(int i = 0; i < 1000; i++)
    {
        pcl::PointXYZI p;
        p.x = i;
        p.y = 4.234;
        p.z = 1;
        p.intensity = 200;
        Eigen::Vector3d p1;
        p1 = rotation_ * Eigen::Vector3d(p.x,p.y,1) + translation_;
        pcl::PointXYZI p2;
        p2.x = p1(0);
        p2.y = p1(1);
        p2.z = 1;
        p2.intensity = 255;
        line_point_cloud_ref_->push_back(p);
        line_point_cloud_per_->push_back(p2);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_out = *line_point_cloud_ref_ ;//+ *line_point_cloud_per_;
    return(cloud_out);
}
void DataGeneration::CloudViewer(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud,"line_cloud");
    viewer->initCameraParameters();
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"line_cloud");
    viewer->addCoordinateSystem (1.0, "global");
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(10);
    
    }

}
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::PlanePointCloudGeneration()
{

    for(int i = 0; i < 1000; i++)
    {
        pcl::PointXYZI p;
        if(i < 500)
        {
            p.x = i;
            p.y = 5;
            p.z = 1;
            p.intensity = 200;
        }
        else
        {
            p.x = i;
            p.y = i*i;
            p.z = 1;
            p.intensity = 255;
        }
                
        Eigen::Vector3d p1;
        p1 = rotation_ * Eigen::Vector3d(p.x,p.y,1) + translation_;
        pcl::PointXYZI p2;
        p2.x = p1(0);
        p2.y = p1(1);
        p2.z = 1;
        p2.intensity = 255;

      
        plane_point_cloud_ref_->push_back(p);

        plane_point_cloud_per_->push_back(p2);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_out = *plane_point_cloud_ref_ ;//+ *plane_point_cloud_per_;
    return(cloud_out);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::ReturnLinePointCloudRef()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    *out = *line_point_cloud_ref_;
    return out;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::ReturnLinePointCloudPer()
{
     pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    *out = *line_point_cloud_per_;
    return out;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::ReturnPlanePointCloudRef()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    *out = *plane_point_cloud_ref_;
    return out;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr DataGeneration::ReturnPlanePointCloudPer()
{
     pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    *out = *plane_point_cloud_per_;
    return out;
}
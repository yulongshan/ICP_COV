#include "data_generation.h"
#include "scan_matching_plicp.h"
using namespace std;

int main()
{
    DataGeneration dg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud = dg.LinePointCloudGeneration();
    // dg.CloudViewer(cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_plane = dg.PlanePointCloudGeneration();
    // dg.CloudViewer(cloud_plane);

    ScanMatchingPLICP sm;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ref(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr per(new pcl::PointCloud<pcl::PointXYZI>);

    // ref = dg.ReturnLinePointCloudRef();
    // per = dg.ReturnLinePointCloudPer();

    ref = dg.ReturnPlanePointCloudRef();
    per = dg.ReturnPlanePointCloudPer();
    sm.ScanMatching(ref,per);
    return 0;
}
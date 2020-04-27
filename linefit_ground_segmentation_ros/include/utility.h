#ifndef _UTILITY_LIDAR_ODOMETRY_Hooo
#define _UTILITY_LIDAR_ODOMETRY_Hooo

#include <pcl_ros/point_cloud.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define PI 3.14159265
#define OUT_POINTS 4096

using namespace std;

typedef pcl::PointXYZI  PointType;


int32_t createDirectory(const std::string &directoryPath);
void writeKittiPclBinData(pcl::PointCloud<PointType>::Ptr input_pointcloud,string dir);
int getTimeStamp(string seq);
void replace_str(std::string& str, const std::string& before, const std::string& after);
void loadBin(string binfile, pcl::PointCloud<PointType>::Ptr laserCloudIn);
// int rmGround(string seq);
#endif

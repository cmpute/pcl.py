#include <pcl/io/pcd_io.h>

int savePCDFileBinaryCompressed2(
    const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
    const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
    const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ())
{
    pcl::PCDWriter w;
    return w.writeBinaryCompressed (file_name, cloud, origin, orientation);
}
#include "_test_types_src.h"

int test(pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    return cloud.size() * cloud.size();
}

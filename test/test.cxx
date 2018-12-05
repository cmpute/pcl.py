#include "test.h"

int test(pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    int sz = cloud.size();
    return sz*sz;
}

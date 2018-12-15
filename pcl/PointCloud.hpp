#ifndef _HEADER_FOR_CYTHON_POINT_CLOUD
#define _HEADER_FOR_CYTHON_POINT_CLOUD

#include <boost/variant.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CyPointCloud;

namespace pcl
{
    // XXX: workaround for that cython doesn't support variadic template type arguments
    typedef boost::variant<
        PointCloud<Normal>,
        PointCloud<PointXY>,
        PointCloud<PointXYZ>
    > PointCloudVariant; // TODO: add more types
    class PointCloudFused : public PointCloudVariant
    {
    public:
        template <typename PointT>
        variant& operator= (PointCloud<PointT>&& rhs)
        {
            PointCloudVariant::operator=(rhs); return *this;
        }

        template <typename PointT>
        variant& operator= (const PointCloud<PointT>& rhs)
        {
            PointCloudVariant::operator=(rhs); return *this;
        }
    };

    void CyTemplatize(const CyPointCloud &input, PointCloudFused &output);
    void CyInstantiate(CyPointCloud &output, const PointCloudFused &input);
}

#endif //_HEADER_FOR_CYTHON_POINT_CLOUD

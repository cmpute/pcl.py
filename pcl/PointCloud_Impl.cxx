#include "Python.h"
#include "PointCloud.hpp"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

// Include at last
#include "PointCloud.h"

namespace pcl
{
    class instantiate_visitor : public boost::static_visitor<>
    {
        private:
            CyPointCloud &_output;
        public:
            instantiate_visitor(CyPointCloud &output) : _output(output) {}

            template <typename PointT>
            void operator()(const PointCloud<PointT> &input) const
            {
                toPCLPointCloud2(input, _output._base);
                _output._origin = input.sensor_origin_;
                _output._orientation = input.sensor_orientation_;
                // TODO: infer type name
            }
    };

    void CyTemplatize(const CyPointCloud &input, PointCloudFused &output)
    {
        if (input._ptype == "XY")
        {
            PointCloud<PointXY> cloud;
            fromPCLPointCloud2(input._base, cloud);
            cloud.sensor_origin_ = input._origin;
            cloud.sensor_orientation_ = input._orientation;
            output = cloud;
        }
        // TODO: add more types
    }
    void CyInstantiate(CyPointCloud &output, const PointCloudFused &input)
    {
        boost::apply_visitor(instantiate_visitor(output), input);
    }
} // pcl


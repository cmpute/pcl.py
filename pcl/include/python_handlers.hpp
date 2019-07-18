#ifndef _PCL_CYTHON_PYTHON_HANDLER
#define _PCL_CYTHON_PYTHON_HANDLER

#include <pcl/visualization/point_cloud_color_handlers.h>

class PointCloudColorHandlerPython : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
{
    typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud; //pcl::PCLPointCloud2
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> Base;

private:
    // Python function should return n*4 array (RGBA)
    unsigned char* (*func_handler_)(void*);
    void* func_object_;

public:
    typedef boost::shared_ptr<PointCloudColorHandlerPython> Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerPython> ConstPtr;

    PointCloudColorHandlerPython (const PointCloudConstPtr &cloud,
        unsigned char* (*func_handler)(void*),
        void* func_object) :
        PointCloudColorHandler<pcl::PCLPointCloud2> (cloud),
        func_handler_(func_handler),
        func_object_(func_object)
        { capable_ = true; }
    
    virtual ~PointCloudColorHandlerPython () {}
    virtual std::string getName () const { return ("PointCloudColorHandlerPython"); }
    virtual std::string getFieldName () const { return (""); }

    void setCapable(bool capable) { capable_ = capable; }

    virtual bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (4);

        vtkIdType nr_points = cloud_->width * cloud_->height;
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples (nr_points);
        unsigned char* colors = func_handler_(func_object_);
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetArray (colors, 3*nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
        free(colors);
    }
};

#endif // _PCL_CYTHON_PYTHON_HANDLER

#ifndef _PCL_CYTHON_PYTHON_HANDLER
#define _PCL_CYTHON_PYTHON_HANDLER

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <Python.h>

class PointCloudColorHandlerPython : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
{
    typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud; //pcl::PCLPointCloud2
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> Base;

private:
    // Python function should return n*4 array (RGBA)
    int (*func_handler_)(PyObject*, unsigned char*);
    PyObject* func_object_;

public:
    typedef boost::shared_ptr<PointCloudColorHandlerPython> Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerPython> ConstPtr;

    PointCloudColorHandlerPython (const PointCloudConstPtr &cloud,
        int (*func_handler)(PyObject*, unsigned char*),
        PyObject* func_object) :
        PointCloudColorHandler<pcl::PCLPointCloud2> (cloud),
        func_handler_(func_handler),
        func_object_(func_object)
        { 
            capable_ = true;
            Py_XINCREF(func_object_);
        }

    virtual ~PointCloudColorHandlerPython () { Py_XDECREF(func_object_); }
    virtual std::string getName () const { return ("PointCloudColorHandlerPython"); }
    virtual std::string getFieldName () const { return (""); }

    void setCapable(bool capable) { capable_ = capable; }

    virtual bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return false;

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (4);

        vtkIdType nr_points = cloud_->width * cloud_->height;
        unsigned char* colors = new unsigned char[nr_points * 4];
        int ret = func_handler_(func_object_, colors);

        if (ret == -1)
        {
            PCL_ERROR("Returned color array should be n*4 shape\n");
            return false;
        }
        else if (ret == -2)
        {
            PCL_ERROR("Unrecognized color type\n");
            return false;
        }
        else if (ret == -3)
        {
            PCL_ERROR("Error occurred in function calling\n");
            return false;
        }

        reinterpret_cast<vtkUnsignedCharArray*>(scalars.Get())->SetNumberOfTuples (nr_points);
        reinterpret_cast<vtkUnsignedCharArray*>(scalars.Get())->SetArray (colors, 4*nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
        return true;
    }
};

#endif // _PCL_CYTHON_PYTHON_HANDLER

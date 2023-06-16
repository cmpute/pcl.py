#ifndef _PCL_CYTHON_PYTHON_HANDLER
#define _PCL_CYTHON_PYTHON_HANDLER

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <Python.h>
#include "pcl/pcl_config.h"

#if PCL_VER >= 11100
    #include <memory>
    #define ccast std::const_pointer_cast
#else
    #include <boost/shared_ptr.hpp>
    #define ccast boost::const_pointer_cast
#endif

class PointCloudColorHandlerPython : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
{
    typedef PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud; //pcl::PCLPointCloud2
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> Base;

private:
    // Python function should fill data in format of unsigned char
    int (*func_handler_)(PyObject*, PointCloudPtr, unsigned char*);
    PyObject* func_object_;

public:
    PointCloudColorHandlerPython (const PointCloudConstPtr &cloud,
        int (*func_handler)(PyObject*, PointCloudPtr, unsigned char*),
        PyObject* func_object) :
        PointCloudColorHandler<pcl::PCLPointCloud2> (cloud),
        func_handler_(func_handler),
        func_object_(func_object)
        { 
            capable_ = true;
            Py_XINCREF(func_object_);
        }

    virtual ~PointCloudColorHandlerPython () { Py_XDECREF(func_object_); }
    std::string getName () const override { return ("PointCloudColorHandlerPython"); }
    std::string getFieldName () const override { return (""); }

    void setCapable(bool capable) { capable_ = capable; }

// the API of getColor has changed in https://github.com/PointCloudLibrary/pcl/commit/a7edaea20b3817983038fece27a70039d137b766
#if PCL_VER >= 11000
    vtkSmartPointer<vtkDataArray> getColor () const override
    {
        if (!capable_ || !cloud_)
            return nullptr;

        auto scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (4);

        vtkIdType nr_points = cloud_->width * cloud_->height;
        unsigned char* colors = new unsigned char[nr_points * 4];
        int ret = func_handler_(func_object_, ccast<PointCloud, const PointCloud>(cloud_), colors);

        if (ret == -1)
        {
            PCL_ERROR("Returned color array should be n*4 shape\n");
            return nullptr;
        }
        else if (ret == -2)
        {
            PCL_ERROR("Unrecognized color type\n");
            return nullptr;
        }
        else if (ret == -3)
        {
            PCL_ERROR("Error occurred in function calling\n");
            return nullptr;
        }

        scalars->SetNumberOfTuples (nr_points);
        scalars->SetArray (colors, 4*nr_points, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
        return scalars;
    }

#else
    bool getColor (vtkSmartPointer<vtkDataArray> &scalars) const override
    {
        if (!capable_ || !cloud_)
            return false;

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        scalars->SetNumberOfComponents (4);

        vtkIdType nr_points = cloud_->width * cloud_->height;
        unsigned char* colors = new unsigned char[nr_points * 4];
        int ret = func_handler_(func_object_, ccast<PointCloud, const PointCloud>(cloud_), colors);

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
#endif
};

#endif // _PCL_CYTHON_PYTHON_HANDLER

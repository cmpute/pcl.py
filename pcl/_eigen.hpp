#ifndef _PYPCL_EIGEN_HPP
#define _PYPCL_EIGEN_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename T, typename U> T _toEigen(U *input)
{
    return Eigen::Map<T>(input);
}

Eigen::Affine3f _toEigenAffine(float *input)
{
    Eigen::Affine3f ret;
    ret.matrix() = Eigen::Map<Eigen::Matrix4f>(input);
    return ret;
}

#endif // _PYPCL_EIGEN_HPP

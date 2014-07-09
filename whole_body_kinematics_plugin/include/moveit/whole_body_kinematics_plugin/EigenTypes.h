#ifndef __EIGEN_TYPES_H__
#define __EIGEN_TYPES_H__

#ifdef random
#undef random
#endif
#include <Eigen/Eigen>

namespace hrp{
    typedef Eigen::Vector3d Vector3;
    typedef Eigen::Matrix3d Matrix33;
    typedef Eigen::MatrixXd dmatrix;
    typedef Eigen::VectorXd dvector;
    typedef Eigen::VectorXi ivector;
    typedef Eigen::Matrix<double, 6,1> dvector6;
    typedef Eigen::Quaternion<double> dquaternion;
};

#endif

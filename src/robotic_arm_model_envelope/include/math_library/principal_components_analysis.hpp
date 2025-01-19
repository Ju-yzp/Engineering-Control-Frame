#ifndef MATH_LIBRARY_PRINCIPAL_COMPONENT_ANALYSIS_HPP_
#define MATH_LIBRARY_PRINCIPAL_COMPONENT_ANALYSIS_HPP_
#include<eigen3/Eigen/Eigen>
#include<eigen3/Eigen/Dense>

#include<eigen3/Eigen/src/Core/Matrix.h>
#include<vector>
namespace PointCloud{
    namespace  PCA {
    typedef  std::vector<Eigen::Vector3f> Point3f ;
    typedef  std::vector<Eigen::Vector2f> Point2f ;
    void decentralization(Point3f &points);
    Eigen::Matrix3f covarianceMatrix(Point3f &points);
    }
    
}

#endif
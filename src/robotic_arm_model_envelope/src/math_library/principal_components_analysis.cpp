#include"math_library/principal_components_analysis.hpp"

#include<algorithm>
#include<cmath>

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/src/Eigenvalues/EigenSolver.h>
namespace PointCloud {
    namespace  PCA{
        //考虑使用多线程，将数据分成多次进行计算

        //数据去中心化
        void decentralization(Point3f &points){
            Eigen::Vector3f average;
            std::for_each(points.begin(), points.end(), [&average](Eigen::Vector3f &point){
                average(0)+=point(0);
                average(1)+=point(1);
                average(2)+=point(2);
            });
            average(0)/=points.size();
            average(1)/=points.size();
            average(2)/=points.size();
            std::for_each(points.begin(), points.end(),[&average](Eigen::Vector3f &point){
                point(0)-=average(0);
                point(1)-=average(1);
                point(2)-=average(2);
            });
        }
        //协方差矩阵
        Eigen::Matrix3f covarianceMatrix(Point3f &points){
            Eigen::Matrix3f covariance_matrix_;
            std::for_each(points.begin(),points.end(),[&covariance_matrix_](Eigen::Vector3f point){
                //x,x x,y x,z
                covariance_matrix_(0,0) += powf(point(0),2);
                covariance_matrix_(0,1) += point(0)*point(1);
                covariance_matrix_(0,2) += point(0)*point(2);
                //y,x y,y y,z
                covariance_matrix_(1,1) += powf(point(1),2);
                covariance_matrix_(1,0) += point(0)*point(1);
                covariance_matrix_(1,2) += point(2)*point(1);
                //z,x z,y z,z
                covariance_matrix_(2,2) += powf(point(2),2);
                covariance_matrix_(2,0) += point(2)*point(0);
                covariance_matrix_(2,1) += point(2)*point(1);
            });
            return covariance_matrix_;
        }
        //数据降维
        // void dataDimensionalityReduction(Eigen::Matrix3f matrix,Point3f &data,Point2f &process_data){
        //     Eigen::EigenSolver<Eigen::Matrix3f> es_(matrix);
        //     es_.eigenvectors();
        //     std::for_each(data.begin(),data.end(),[&](Eigen::Vector3f point){
        //         Eigen::Vector2f point_cloud_;
        //         process_data.emplace_back(point_cloud_);
        //     });
        // }
    }
}
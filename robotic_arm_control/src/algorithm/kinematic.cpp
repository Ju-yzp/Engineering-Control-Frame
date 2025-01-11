#include"algorithm/kinematic.hpp"

#include<cmath>

namespace RoboticArm{
    namespace Algorithm{
        inline const float& Kinematic<float>::GetConstParam(const int col,const int row){
            return DH_Table[col][row];
        }

        inline float& Kinematic<float>::GetParam(const int col,const int row){
            return DH_Table[col][row];
        }

        bool Kinematic<float>::SolveWristPart(Eigen::Matrix4f pose,Solutions &solutions){

        }

        Eigen::Matrix4f Kinematic<float>::FrameRealtionShip(float theta,float alpha,float a ,float d){
            Eigen::Matrix4f matrix_;
            float cos_alpha = cos(alpha);
            float sin_alpha = sin(alpha);
            float sin_theta = sin(theta);
            float cos_theta = cos(theta);
            matrix_<<cos_theta          ,-sin_theta         ,0         ,           a,
                     sin_theta*cos_alpha,cos_theta*cos_alpha,-sin_theta,-sin_theta*d,
                     sin_alpha*sin_alpha,cos_theta*sin_alpha, cos_theta, cos_alpha*d,
                     0                  ,                  0,         0,           1;
            return matrix_;
        }

        /*
        *逆运动学求解采用解析解方法，通过几何关系与旋转变换关系得出解集
        *pose:机械臂末端目标位姿
        *solutions:关节偏移角度解
        */
       
        bool Kinematic<float>::SolveIK(Eigen::Matrix4f pose,Solutions &solutions){
            float x_{pose(0,3)},y_{pose(1,3)},z_{pose(2,3)};
            float a1_,d2_,d4_,a2_;
            a1_ = Kinematic<float>::GetConstParam(1,1);
            d2_ = Kinematic<float>::GetConstParam(2,1);
            d4_ = Kinematic<float>::GetConstParam(1,2);
            a2_ = Kinematic<float>::GetConstParam(3,2);

            float len_ = sqrtf(powf(x_,2)+powf(y_,2));
            float xoy_len_ = sqrtf(powf(len_,2)-powf(d2_,2));
        }

        /*
        *通过正运动学验证解是否在误差允许范围内
        *pose：机械臂末端目标位姿
        *solutions:通过逆运动学解算得到的解集
        */
        bool Kinematic<float>::SolveFk(Eigen::Matrix4f pose,const Solutions solutions){
            Eigen::Matrix4f Transfrom1to0_,Transfrom2to1_,Transfrom3to2_,
                            Transfrom4to3_,Transfrom5to4_,Transfrom6to5_;
            Transfrom1to0_ = FrameRealtionShip(solutions.theta1_,GetConstParam(0,0),GetConstParam(0,1),GetConstParam(0,2));
            Transfrom2to1_ = FrameRealtionShip(solutions.theta2_,GetConstParam(1,0),GetConstParam(1,1),GetConstParam(1,2));
            Transfrom3to2_ = FrameRealtionShip(solutions.theta3_,GetConstParam(2,0),GetConstParam(2,1),GetConstParam(2,2));
            Transfrom4to3_ = FrameRealtionShip(solutions.theta4_,GetConstParam(3,0),GetConstParam(3,1),GetConstParam(3,2));
            Transfrom5to4_ = FrameRealtionShip(solutions.theta5_,GetConstParam(4,0),GetConstParam(4,1),GetConstParam(4,2));
            Transfrom6to5_ = FrameRealtionShip(solutions.theta6_,GetConstParam(5,0),GetConstParam(5,1),GetConstParam(5,2));

            Eigen::Matrix4f Transfrom6to0 = Transfrom1to0_ *
                                            Transfrom2to1_ *
                                            Transfrom3to2_ *
                                            Transfrom3to2_ *
                                            Transfrom5to4_ *
                                            Transfrom6to5_;
        }
    }
}
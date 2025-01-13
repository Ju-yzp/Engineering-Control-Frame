#include"algorithm/kinematic.hpp"

#include<cmath>

namespace RoboticArm{
    namespace Algorithm{
        Kinematic::Kinematic(rclcpp::Node *node):
        node_(node)
        {}

        Kinematic::Kinematic():
        node_(new rclcpp::Node(""))
        {}

        inline const float& Kinematic::GetConstParam(const int col,const int row)const{
            return DH_Table[col][row];
        }

        inline float& Kinematic::GetParam(const int col,const int row){
            return DH_Table[col][row];
        }

        Eigen::Matrix4f FrameRealtionShip(float theta,float alpha,float a ,float d){
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
       
        bool Kinematic::SolveIK(Eigen::Matrix4f pose,Solutions &solutions){
            float x_{pose(0,3)},y_{pose(1,3)};//,z_{pose(2,3)};
            float a1_,d2_,d4_,a2_;
            a1_ = GetConstParam(1,1);
            d2_ = GetConstParam(2,1);
            d4_ = GetConstParam(1,2);
            a2_ = GetConstParam(3,2);

            float len_ = sqrtf(powf(x_,2)+powf(y_,2));
            float xoy_len_ = sqrtf(powf(len_,2)-powf(d2_,2));

            float theta1_,temp_theta_;
            temp_theta_ = acosf(xoy_len_/len_);
            theta1_ = atan2f(pose(1,3)/len_,pose(0,3)/len_);
            theta1_  -= temp_theta_;

            //通过三角形构成的长度原理判定能否到达目标位姿
            if((a2_+d4_) <xoy_len_ || (d4_-a2_) > xoy_len_){
                RCLCPP_WARN(node_->get_logger(),"");
                return false;
            }

            //继续求解电机2，3的角度解
            Eigen::Matrix4f Transfrom1to0 = FrameRealtionShip(theta1_,GetConstParam(0,0),GetConstParam(0,1),GetConstParam(0,2));
            Eigen::Matrix4f temp_pose_ = Transfrom1to0.inverse() * pose;
            temp_pose_(0,3) += a1_;
            temp_pose_(2,3) -= d2_;

            float xoz_len_ = sqrtf(powf(temp_pose_(0,3),2)+powf(temp_pose_(2,3),2));
            //先把前提条件求出来
            float theta2_,theta3_,temp_theta_1;
            temp_theta_1 = atan2f(temp_pose_(2,3)/xoz_len_,temp_pose_(0,3)/xoz_len_);
            theta2_ = acosf(powf(a2_,2)+powf(xoz_len_,2)-powf(d4_,2)/
                            (2.0f*a2_*xoz_len_));

            theta3_ = acosf(powf(a2_,2)+powf(d4_,2)-powf(xoz_len_,2)/
                            (2.0f*a2_*d4_));
            //第一种情况
            float theta2_1 = theta2_ - temp_theta_1;
            float theta3_1 = theta3_ - M_PIf/2.0f;
            //第二种情况
            float theta2_2 = -theta2_ - temp_theta_1;
            float theta3_2 = M_PIf*1.5f - theta3_;
            //如果出现nan值，说明解不存在
            if(std::isnan(theta2_1)||std::isnan(theta2_2)||std::isnan(theta3_2)||std::isnan(theta3_1)){
                RCLCPP_WARN(node_->get_logger(),"");
                return false;
            }
            Solutions solution1_,solution2_;
            solution1_.theta1_ = theta1_;
            solution1_.theta2_ = theta2_1;
            bool flag1 = SolveWristPart(pose,solution1_);
            bool flag2 = SolveWristPart(pose,solution2_);
            if(!flag1)
            {
                if(!flag2){
                    RCLCPP_WARN(node_->get_logger(),"Don't has vail solutions !");
                    return false;
                }
                else{
                    solutions = solution2_;
                    return true;
                }
            }else{
                solutions = solution1_;
                return true;
            }
        }
        /*
        *求解腕部关节角度
        */
        bool Kinematic::SolveWristPart(Eigen::Matrix4f pose,Solutions &solutions){
            Eigen::Matrix4f transfrom1to0_,transfrom2to1_,transfrom3to2_;
            transfrom1to0_ = FrameRealtionShip(solutions.theta1_,GetConstParam(0,0),GetConstParam(0,1),GetConstParam(0,2));
            transfrom2to1_ = FrameRealtionShip(solutions.theta2_,GetConstParam(1,0),GetConstParam(1,1),GetConstParam(1,2));
            transfrom3to2_ = FrameRealtionShip(solutions.theta3_,GetConstParam(2,0),GetConstParam(2,1),GetConstParam(2,2));

            Eigen::Matrix4f transfrom6to3_ = transfrom3to2_.inverse()*
                                            transfrom2to1_.inverse()*
                                            transfrom1to0_.inverse()*
                                            pose;
            
            //分情况讨论
            float theta4_,theta5_,theta6_;
            if(transfrom6to3_(1,2) < -0.95f ){
                theta4_ = 0.0f;
                theta5_ = 0.0f;
                theta6_ = atan2f(-transfrom6to3_(0,1),transfrom6to3_(0,0));
            }
            else if(transfrom6to3_(1,2) > 0.95f){
                theta4_ = 0.0f;
                theta5_ = M_PIf;
                theta6_ = atan2f(transfrom6to3_(0,1),-transfrom6to3_(0,0));
            }
            else{
                theta5_ = atan2f(sqrtf(powf(transfrom6to3_(0,2),2)+powf(transfrom6to3_(2,2),2)),-transfrom6to3_(1,2));
                theta4_ = atan2f(transfrom6to3_(2,2)/sinf(theta5_),transfrom6to3_(0,2)/sinf(theta5_));
                theta6_ = atan2f(transfrom6to3_(1,1)/sinf(theta5_),-transfrom6to3_(1,0)/sinf(theta5_));
            }
            if(std::isnan(theta4_)||std::isnan(theta5_)||std::isnan(theta6_))
              return false;

            solutions.theta4_ = theta4_;
            solutions.theta5_ = theta5_;
            solutions.theta6_ = theta6_;
            return true;
        }
        /*
        *通过正运动学验证解是否在误差允许范围内
        *pose：机械臂末端目标位姿
        *solutions:通过逆运动学解算得到的解集
        */
        bool Kinematic::SolveFk(Eigen::Matrix4f pose,const Solutions solutions){
            Eigen::Matrix4f transfrom1to0_,transfrom2to1_,transfrom3to2_,
                            Transfrom4to3_,Transfrom5to4_,Transfrom6to5_;
            transfrom1to0_ = FrameRealtionShip(solutions.theta1_,GetConstParam(0,0),GetConstParam(0,1),GetConstParam(0,2));
            transfrom2to1_ = FrameRealtionShip(solutions.theta2_,GetConstParam(1,0),GetConstParam(1,1),GetConstParam(1,2));
            transfrom3to2_ = FrameRealtionShip(solutions.theta3_,GetConstParam(2,0),GetConstParam(2,1),GetConstParam(2,2));
            Transfrom4to3_ = FrameRealtionShip(solutions.theta4_,GetConstParam(3,0),GetConstParam(3,1),GetConstParam(3,2));
            Transfrom5to4_ = FrameRealtionShip(solutions.theta5_,GetConstParam(4,0),GetConstParam(4,1),GetConstParam(4,2));
            Transfrom6to5_ = FrameRealtionShip(solutions.theta6_,GetConstParam(5,0),GetConstParam(5,1),GetConstParam(5,2));

            Eigen::Matrix4f Transfrom6to0 = transfrom1to0_ *
                                            transfrom2to1_ *
                                            transfrom3to2_ *
                                            transfrom3to2_ *
                                            Transfrom5to4_ *
                                            Transfrom6to5_;
            Eigen::Matrix4f error_matrix_ = pose - Transfrom6to0;
            float position_error_{0},rotation_error_{0};
            for(int i = 0 ;i < 3;i++){
                for(int j = 0;j < 3;j++){
                    rotation_error_ += powf(error_matrix_(i,j),2);
                }
                position_error_ += powf(error_matrix_(i,3),2);
            }
            constexpr float position_error_stand{20};
            constexpr float rotation_error_stand{0.12};
            if(rotation_error_ < rotation_error_stand && position_error_ < position_error_stand){
                return true;
            }else{
                RCLCPP_WARN(node_->get_logger(),"");
                return false;
            }
        }
    }
}
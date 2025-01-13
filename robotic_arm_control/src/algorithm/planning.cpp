#include"algorithm/planning.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace RoboticArm{
    namespace Algorithm{
    Planning::Planning(RoboticArm::Algorithm::Kinematic * kinematic_base,rclcpp::Node * node):
    kinematic_base_(kinematic_base),
    node_(node)
    {}

    Planning::Planning():
    kinematic_base_(nullptr),
    node_(new rclcpp::Node(""))
    {}
    
    bool Planning::Executable(const float step,Eigen::Matrix4f start_pose,Eigen::Matrix4f end_pose,
                                    Trajectory &tarjectory){
            if(kinematic_base_ == nullptr){
                RCLCPP_WARN(node_->get_logger(),"Kinematic base is not initialized !");
                return false;
            }
            Eigen::Vector3f middle_position_;
            middle_position_<<end_pose(0,3)-start_pose(0,3),
                              end_pose(1,3)-start_pose(1,3),
                              end_pose(2,3)-start_pose(2,3);
            float distance_ = middle_position_.norm();
            int step_num = distance_/step;

            float z_scale = middle_position_(2)/distance_;
            float y_scale = middle_position_(1)/distance_;
            float x_scale = middle_position_(0)/distance_;

            Trajectory result;
            Eigen::Matrix4f temp_pose = start_pose;
            int fail_count = 0;
            for(int i = 0 ; i <  step_num - 1; i++){
                start_pose(0,3) += x_scale*step;
                start_pose(1,3) += y_scale*step;
                start_pose(2,3) += z_scale*step;
                Solutions solutions_;
                if(kinematic_base_->SolveIK(start_pose,solutions_)&&kinematic_base_->SolveFk(temp_pose,solutions_)){
                    result.push_back(solutions_);
                }
                else{
                    fail_count++;
                }
            }     
            int limit_count_  =   step_num * 0.2;
            if(fail_count > limit_count_){
                RCLCPP_WARN(node_->get_logger(),"Too many failed points !");
                return false;
            }
            return true;
        }   
    }
}
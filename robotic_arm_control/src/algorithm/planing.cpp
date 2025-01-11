#include"algorithm/plannig.hpp"

namespace RoboticArm{
    namespace Algorithm{
    bool Planning<float>::Executable(const float step,Eigen::Matrix4f start_pose,Eigen::Matrix4f end_pose,
                                    Trajectory &tarjectory){
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
                Solutions solution;
                if(kinematic_base_->SolveIK(start_pose,solution)){
                    result.push_back(solution);
                }
                else{
                    fail_count++;
                }
            }          
            if(fail_count > 0){
            }
        }   
    }
}
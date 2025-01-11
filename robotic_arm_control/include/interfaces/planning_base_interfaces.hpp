#ifndef INTERFACES_PLANNING_BASE_INTERFACES_HPP_
#define INTERFACES_PLANNING_BASE_INTERFACES_HPP_
//需要给他增加错误检查，以保证程序的正确运行
//eigen  library
#include<Eigen/Eigen>
#include<Eigen/Dense>
//cpp system
#include<memory>
//custom data types
#include"data_types/trajectory_struct.hpp"
//base interfaces
namespace RoboticArm{
    namespace Interfaces{
        class PlanningBaseInterface{
            public:
            virtual ~PlanningBaseInterface()=default;
            virtual Trajectory Executable(const float step,Eigen::Matrix4f pose) = 0;
            virtual Trajectory Executable(Eigen::Matrix4f pose)=0;
            virtual Trajectory Executable(Eigen::Matrix4f now_pose,Eigen::Matrix4f target_pose,
            const float step)=0;
        };
    }
}
#endif
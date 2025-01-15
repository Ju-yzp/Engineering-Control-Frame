#ifndef ALGORITHM_ROBOTIC_PLANNING_HPP_
#define ALGORITHM_ROBOTIC_PLANNING_HPP_
//base interfaces
#include"interfaces/planning_base_interfaces.hpp"
#include"interfaces/kinematic_base_interface.hpp"
//core algotithm
#include"algorithm/kinematic.hpp"
//cpp system
#include<memory>
//macros
#include"detail/macros.hpp"
namespace RoboticArm{
    namespace Algorithm{
        class Planning: public Interfaces::PlanningBaseInterface{
            public:
            REGISTER_SAMRT_PTR(Planning)
            explicit Planning(RoboticArm::Algorithm::Kinematic * kinematic_base,rclcpp::Node * node);
            Planning();
            bool Executable(const float step,Eigen::Matrix4f start_pose,Eigen::Matrix4f end_pose,
                                    Trajectory &tarjectory)override;
            private:
            RoboticArm::Interfaces::KinematicBaseInterface * kinematic_base_;
            rclcpp::Node * node_;
        };
    }
}
#endif
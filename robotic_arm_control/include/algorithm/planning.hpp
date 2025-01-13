#ifndef ALGORITHM_ROBOTIC_PLANNING_HPP_
#define ALGORITHM_ROBOTIC_PLANNING_HPP_
//base interfaces
#include"interfaces/planning_base_interfaces.hpp"
#include"interfaces/kinematic_base_interface.hpp"

#include"algorithm/kinematic.hpp"
namespace RoboticArm{
    namespace Algorithm{
        class Planning: public Interfaces::PlanningBaseInterface{
            public:
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
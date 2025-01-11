#ifndef ALGORITHM_ROBOTIC_PLANNING_HPP_
#define ALGORITHM_ROBOTIC_PLANNING_HPP_
//base interfaces
#include"interfaces/planning_base_interfaces.hpp"
#include"interfaces/kinematic_base_interface.hpp"

namespace RoboticArm{
    namespace Algorithm{
        template <typename Ty_>
        class Planning: public Interfaces::PlanningBaseInterface{
            public:
            Planning();
            Trajectory Executable(const float step,Eigen::Matrix4f pose)override;
            Trajectory Executable(Eigen::Matrix4f pose)override;
            Trajectory Executable(Eigen::Matrix4f now_pose,Eigen::Matrix4f target_pose,
            const float step)override;
            private:
            RoboticArm::Interfaces::KinematicBaseInterface * kinematic_base_;
        };
    }
}
#endif
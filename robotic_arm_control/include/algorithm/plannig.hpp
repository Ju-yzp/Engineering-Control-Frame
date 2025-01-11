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
            bool Executable(const float step,Eigen::Matrix4f start_pose,Eigen::Matrix4f end_pose,
                                    Trajectory &tarjectory)override;
            virtual bool Executable(Eigen::Matrix4f start_pose,Eigen::Matrix4f end_pose,Trajectory &tarjectory)override;
            private:
            RoboticArm::Interfaces::KinematicBaseInterface * kinematic_base_;
        };
    }
}
#endif
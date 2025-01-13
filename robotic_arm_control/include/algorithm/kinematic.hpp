#ifndef ALGORITHM_ROBOTIC_KINAMATIC_HPP_
#define ALGORITHM_ROBOTIC_KINAMATIC_HPP_
//base interfaces
#include"interfaces/kinematic_base_interface.hpp"

//ros
#include<rclcpp/node.hpp>
#include"detail/macros.hpp"

//cpp system
#include<memory>

namespace RoboticArm{
    namespace Algorithm{
        Eigen::Matrix4f FrameRealtionShip(float theta,float alpha,float a ,float d);
        class Kinematic: public Interfaces::KinematicBaseInterface{
            public:
            REGISTER_SAMRT_PTR(Kinematic)
            explicit Kinematic(rclcpp::Node * node);
            explicit Kinematic();
            const float& GetConstParam(const int col,const int row)const override;
            float & GetParam(const int col,const int row)override;
            bool SolveIK(Eigen::Matrix4f pose,Solutions &solutions)override;
            bool SolveFk(Eigen::Matrix4f pose,const Solutions solutions)override;
            private:
            bool SolveWristPart(Eigen::Matrix4f pose,Solutions &solutions);
            protected:
            float DH_Table[6][4];
            rclcpp::Node * node_;
        };
    }
}
#endif

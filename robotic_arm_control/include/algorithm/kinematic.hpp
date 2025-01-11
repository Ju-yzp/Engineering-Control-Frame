#ifndef ALGORITHM_ROBOTIC_KINAMATIC_HPP_
#define ALGORITHM_ROBOTIC_KINAMATIC_HPP_
//base interfaces
#include"interfaces/kinematic_base_interface.hpp"

namespace RoboticArm{
    namespace Algorithm{
        template <typename Ty_>
        class Kinematic:public Interfaces::KinematicBaseInterface<Ty_>{
            public:
            const Ty_& GetConstParam(const int col,const int row)override;
            Ty_ & GetParam(const int col,const int row)override;
            bool SolveIK(Eigen::Matrix4f pose,Solutions &solutions)override;
            bool SolveFk(Eigen::Matrix4f pose,const Solutions solutions)override;
            private:
            bool SolveWristPart(Eigen::Matrix4f pose,Solutions &solutions);
            Eigen::Matrix4f FrameRealtionShip(float theta,float alpha,float a ,float d);
            protected:
            Ty_ DH_Table[6][4];
        };
    }
}
#endif

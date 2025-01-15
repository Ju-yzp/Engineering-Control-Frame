#ifndef INTERFACES_KINEMATIC_BASE_INTERFACE_HPP_
#define INTERFACES_KINEMATIC_BASE_INTERFACE_HPP_
//eigen library
#include<eigen3/Eigen/Eigen>
#include<eigen3/Eigen/Dense>
//custom data types
#include"data_types/solutions_struct.hpp"

namespace RoboticArm{
    namespace Interfaces{
    class KinematicBaseInterface{
        public:
        virtual ~KinematicBaseInterface()=default;
        virtual const float& GetConstParam(const int col,const int row)const=0;
        virtual float & GetParam(const int col,const int row)=0;
        virtual bool SolveIK(Eigen::Matrix4f pose,Solutions &solutions)=0;
        virtual bool SolveFk(Eigen::Matrix4f pose,const Solutions solutions)=0;
    };
    }
}
#endif
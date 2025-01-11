#ifndef INTERFACES_KINEMATIC_BASE_INTERFACE_HPP_
#define INTERFACES_KINEMATIC_BASE_INTERFACE_HPP_
//cpp system
#include<memory>
#include<thread>
#include<mutex>
#include<algorithm>
#include<functional>
//eigen library
#include<Eigen/Eigen>
#include<Eigen/Dense>
//custom data types
#include"structs/solutions_struct.hpp"

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
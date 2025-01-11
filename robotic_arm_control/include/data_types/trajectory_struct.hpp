#ifndef STRUCTS_TRAJECTOTY_STRUCT_HPP_
#define STRUCTS_TRAJECTOTY_STRUCT_HPP_

#include<deque>
#include<memory>

#include"structs/solutions_struct.hpp"

namespace RoboticArm{
    typedef struct{
        std::deque<Solutions> plan_tarjectory;
    }Trajectory;
}

#endif
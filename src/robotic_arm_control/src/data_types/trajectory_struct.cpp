#include "data_types/trajectory_struct.hpp"
#include "data_types/solutions_struct.hpp"

namespace RoboticArm{
    inline void Trajectory::push_back(Solutions &solutions){
        plan_tarjectory_.push_back(solutions);
    }

    inline void Trajectory::emplace_back(Solutions &solutions){
        plan_tarjectory_.emplace_back(solutions);
    }

    inline const std::size_t Trajectory::size(){
        return plan_tarjectory_.size();
    }

    Solutions Trajectory::pop_front(){
        Solutions result = plan_tarjectory_.front();
        plan_tarjectory_.pop_front();
        return result;
    }

    inline bool Trajectory::isEmpty(){
        return plan_tarjectory_.empty();
    }
}
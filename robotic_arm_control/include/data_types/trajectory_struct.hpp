#ifndef STRUCTS_TRAJECTOTY_STRUCT_HPP_
#define STRUCTS_TRAJECTOTY_STRUCT_HPP_

#include<deque>
#include<memory>

#include"data_types/solutions_struct.hpp"

namespace RoboticArm{
    typedef struct{
        public:
        Solutions pop_front(){
            Solutions result = plan_tarjectory.front();
            plan_tarjectory.pop_front();
            return result;
        }
        void push_back(Solutions solution){
            plan_tarjectory.push_back(solution);
        }
        void clear(){
            plan_tarjectory.clear();
        }
        bool empty(){
            return plan_tarjectory.empty();
        }
        const size_t size()const{
            return plan_tarjectory.size();
        }
        Solutions front()const{
            return plan_tarjectory.front();
        }
        Solutions back()const{
            return plan_tarjectory.back();
        }   
        Solutions operator[](size_t index){
            return plan_tarjectory[index];
        }
        private:
        std::deque<Solutions> plan_tarjectory;
    }Trajectory;
}

#endif
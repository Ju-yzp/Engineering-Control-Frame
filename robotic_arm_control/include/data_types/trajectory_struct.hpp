#ifndef DATA_TYPES_TRAJECTOTY_STRUCT_HPP_
#define DATA_TYPES_TRAJECTOTY_STRUCT_HPP_

#include<deque>

#include<mutex>
#include"data_types/solutions_struct.hpp"

namespace RoboticArm{
    struct Trajectory{
        public:
        void push_back(Solutions &solutions);
        void emplace_back(Solutions &solutions);
        const std::size_t size();
        Solutions pop_front();
        private:
        std::mutex data_safe_mutex_;
        std::deque<Solutions> plan_tarjectory;
    };
}

#endif
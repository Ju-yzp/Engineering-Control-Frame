#ifndef DATA_TYPES_TRAJECTOTY_STRUCT_HPP_
#define DATA_TYPES_TRAJECTOTY_STRUCT_HPP_
//stl
#include<deque>
//mutil thread
#include<mutex>
//data types
#include"data_types/solutions_struct.hpp"

//考虑多线程生成轨迹的情况下，使用互斥锁避免数据竞争

namespace RoboticArm{
    struct Trajectory{
        public:
        void push_back(Solutions &solutions);
        void emplace_back(Solutions &solutions);
        const std::size_t size();
        Solutions pop_front();
        bool isEmpty();
        private:
        std::mutex data_safe_mutex_;
        std::deque<Solutions> plan_tarjectory_;
    };
}

#endif
#ifndef DATA_TYPES_LIMITS_STRUCT_HPP_
#define DATA_TYPES_LIMITS_STRUCT_HPP_

#include<vector>

namespace RoboticArm{
    struct Range{
        float upper;
        float lower;
    };//范围

    //暴露的接口满足基本需求就行,使用容器的好处就是它的功能完善
    
    struct JointLimits{
        public:
        JointLimits(const unsigned int init_size);
        JointLimits()=default;
        void reserve(const unsigned int size);
        void clear();
        void emplace_back(Range &range);
        Range& operator[](const int index);
        std::size_t size();
        template <typename  Ty_>
        bool isSatisfyConstraints(const Ty_ comparison);
        private:
        std::vector<Range> joint_limits_;
    };
}

#endif
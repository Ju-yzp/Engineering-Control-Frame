#include "data_types//limits_struct.hpp"

#include<stdexcept>

namespace RoboticArm{
    //虽然是否成为内联函数是由编译器决定的，但是显式声明为内联这是我的个人习惯问题
    
    JointLimits::JointLimits(const unsigned int init_size){
        joint_limits_.reserve(init_size);
    }
    inline void JointLimits::reserve(const unsigned int size_){
        joint_limits_.reserve(size_);
    }
    inline void JointLimits::clear(){
        joint_limits_.clear();
    }

    inline Range& JointLimits::operator[] (const int index){
        return joint_limits_[index];
    }

    inline void JointLimits::emplace_back(Range &range){
        joint_limits_.emplace_back(range);
    }

    inline std::size_t JointLimits::size(){
        return joint_limits_.size();
    }

    /*
    *用于判断是否满足约束条件
    *comparison:用于比较的
    *返回true则满足约束条件，反之则不满足
    */
    template <typename  Ty_>
    bool JointLimits::isSatisfyConstraints(const Ty_ comparison){
        if(comparison.size() != joint_limits_.size()){
            throw  std::runtime_error("The number of containers entered does not match the number of built-in container elements ");
        }
        for(std::size_t index = 0;index < comparison.size();index++){
            if(comparison[index] > joint_limits_[index].upper || 
               comparison[index] < joint_limits_[index].lower)
               return false;
        }
        return true;
    }
}
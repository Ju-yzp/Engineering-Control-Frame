#include "algorithm/kinematic.hpp"
#include "algorithm/planning.hpp"

#include<memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>

class test_node:public rclcpp::Node{
    public:
    test_node():
    Node("Test_node",rclcpp::NodeOptions())
    {
        kinematic_ = std::make_shared<RoboticArm::Algorithm::Kinematic>(this);
        planing_ = std::make_shared<RoboticArm::Algorithm::Planning>(kinematic_.get(),this);
    }
    private:
    RoboticArm::Algorithm::Kinematic::SharedPtr kinematic_;
    RoboticArm::Algorithm::Planning::SharedPtr planing_;
};


int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    std::shared_ptr<test_node> node_ = std::make_shared<test_node>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}
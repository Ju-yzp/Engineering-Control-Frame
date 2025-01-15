#include"thread_pool.hpp"
#include"parse_stl_tool.hpp"

int main(){
    ThreadPool thread_pool_(4);
    std::vector<Parser> parsers_;
    std::vector<std::string> file_paths_;
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/base_link.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link2.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link3.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link4.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link5.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link6.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link7.STL"));
    int count{};
    thread_pool_.init();
    for(auto file:file_paths_){
        parsers_.push_back(Parser());
        auto task = std::bind(&Parser::ParseBinary,&parsers_[count],file);
        thread_pool_.submit(task);
        count++;
    }
    thread_pool_.shutdown();
    return 0;
}
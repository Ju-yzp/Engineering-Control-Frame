#include"thread_pool.hpp"
#include"parse_stl_tool.hpp"

#include"math_library/principal_components_analysis.hpp"
#include <eigen3/Eigen/src/Eigenvalues/EigenSolver.h>

int main(){
    ThreadPool thread_pool_(2);
    // std::vector<Parser> parsers_;
    std::vector<std::string> file_paths_;
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/base_link.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link2.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link3.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link4.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link5.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link6.STL"));
    file_paths_.push_back(std::string("/home/zy_jp/planning_test/src/robotic_arm_description/meshes/visual/Link7.STL"));
    Parser paser_;
    paser_.ParseBinary(file_paths_[0]);

    PointCloud::PCA::decentralization(paser_.traingles_);
    Eigen::Matrix3f covariance_matrix_ = PointCloud::PCA::covarianceMatrix(paser_.traingles_);

    Eigen::EigenSolver<Eigen::Matrix3f> es_(covariance_matrix_);
    std::cout<<es_.eigenvectors()<<std::endl;

    // int count{};
    // thread_pool_.init();
    // for(auto file:file_paths_){
    //     parsers_.push_back(Parser());
    //     auto task = std::bind(&Parser::ParseBinary,&parsers_[count],file);
    //     thread_pool_.submit(task);
    //     count++;
    // }
    // thread_pool_.shutdown();
    return 0;
}
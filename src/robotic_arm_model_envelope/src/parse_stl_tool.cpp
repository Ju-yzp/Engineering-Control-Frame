#include"parse_stl_tool.hpp"

#include<stdexcept>
//stream
#include<ios>
#include<fstream>
#include<iostream>

void Parser::ParseBinary(const std::string file_path){
        std::ifstream ifs_(file_path.c_str(),std::ios::binary);
    std::cout<<"----strat parse stl document "+ file_path +"----"<<std::endl;
    if(!ifs_){
        throw std::runtime_error("directory is no exsit !");
    }
    //具体结合stl二进制文件的组成进行理解
    int num_{0};//文件大小
    const int float_type_size_ = sizeof(float);
    char fileHead[3] ;//文件头
    ifs_.read(fileHead,sizeof(char)*3);
    std::cout<<"file header "<<fileHead<<std::endl;
    char fileInfo[77];//文件描述性息
    ifs_.read(fileInfo,sizeof(char)*77);
    std::cout<<"file info "<<fileInfo<<std::endl;
    ifs_.read((char *)(&num_),sizeof(int));
    std::cout<<"the number of traingles "<<num_<<std::endl;

    //提前分配内存
    traingles_.reserve(num_);
    //循环读取数据，保存至traingles_
    for(int loop_count_ = 0 ; loop_count_ < num_ ;loop_count_++){
        Traingle traingle_;
        float point_value_[3];
        for(int i = 0 ; i < 3;i++){
            ifs_.read((char *)(&point_value_[0]),float_type_size_);
            ifs_.read((char *)(&point_value_[1]),float_type_size_);
            ifs_.read((char *)(&point_value_[2]),float_type_size_);
            // std::cout<<"data "<<point_value_[0]<<" "<<point_value_[1]<<" "<<point_value_[2]<<std::endl;
            traingle_.vectex[i] = cv::Point3f(point_value_[0],point_value_[1],point_value_[2]);
        }
        traingles_.emplace_back(traingle_);
    }
    ifs_.close();
}
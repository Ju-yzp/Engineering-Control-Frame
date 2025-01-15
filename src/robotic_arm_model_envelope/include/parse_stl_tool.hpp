#ifndef PARSE_STL_TOOL_HPP_
#define PARSE_STL_TOOL_HPP_
#include<iostream>
//stl 
#include<vector>
//math library
#include<math.h>
//opencv
#include<opencv4/opencv2/opencv.hpp>


class Parser{
    private:
    
    typedef cv::Point3f Vertex;

    struct Traingle{
    Vertex vectex[3];
    };

    std::vector<Traingle> traingles_;
    public:
    Parser()=default;
    void ParseBinary(const std::string file_path);
};

#endif
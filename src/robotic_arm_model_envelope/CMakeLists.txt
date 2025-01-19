cmake_minimum_required(VERSION 3.10)
project(custom_tools)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(OpenCV REQUIRED)

add_executable(parse_stl_tool
${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/parse_stl_tool.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/math_library/principal_components_analysis.cpp)


if(MSVC)
    target_compile_options(parse_stl_tool PRIVATE /W4 /WX)  
else()
    target_compile_options(parse_stl_tool PRIVATE -Wall -Wextra -pedantic -Werror)
endif()
target_include_directories(parse_stl_tool
PRIVATE 
${OpenCV_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(parse_stl_tool 
PRIVATE ${OpenCV_LIBS})

install(TARGETS parse_stl_tool
DESTINATION ${project}/bin
)



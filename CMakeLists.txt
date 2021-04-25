# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

#项目名称
project(ICP)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")
# 显示指定使用的C++编译器 
set(CMAKE_CXX_COMPILER "g++")

# #指定二进制文件保存路径
# set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)

#message(${PROJECT_SOURCE_DIR})
#message(${PROJECT_BINARY_DIR})

find_package(PCL  REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres)
include_directories("/usr/include/eigen3")

include_directories(${PCL_INCLUDE_DIRS}
include/ 
${OpenCV_INCLUDE_DIRS})


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加include路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 指定生成目标
aux_source_directory(${PROJECT_SOURCE_DIR}/src SrcFiles)
add_executable(${PROJECT_NAME} ${SrcFiles})
target_link_libraries(ICP ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
add_definitions(-std=c++11)

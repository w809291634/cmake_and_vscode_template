CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# 项目名称
PROJECT(template)

# 找opencv package----------记得添加这个
# find_package(OpenCV REQUIRED) 

set(CMAKE_C_COMPILER "g++")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 11)

# 设置工程文件夹路径
set(PROJECT_PATH ${PROJECT_SOURCE_DIR}/template)
# 源文件路径
aux_source_directory(${PROJECT_PATH}/src PROJECT_SRC)
# 添加头文件目录
include_directories("${PROJECT_PATH}/include")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/release) 
# 制作库文件
add_library(template SHARED    ${PROJECT_PATH}/src/test_class.cpp 
                        ${PROJECT_PATH}/src/TestWrapper.c )  #编译成静态库如libtemplate.so


# 这里只制作so库文件
install(TARGETS
    template
  LIBRARY DESTINATION ${PROJECT_PATH}/src
  RUNTIME DESTINATION ${PROJECT_PATH}/src)

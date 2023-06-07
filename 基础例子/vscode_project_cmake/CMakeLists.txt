CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# 项目名称
PROJECT(marm_visual_control)

# 找opencv package----------记得添加这个
find_package(OpenCV REQUIRED) 

set(CMAKE_C_COMPILER "g++")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)                # 显示makefile的编译详细信息
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -Wall -std=c++11")

# 设置工程文件夹路径
set(PROJECT_PATH ${PROJECT_SOURCE_DIR}/marm_visual_control)
# 根目录/代码路径
aux_source_directory(${PROJECT_PATH}/src PROJECT_SRC)

# dubug 模式------------------这个非常重要，否则无法进入断点
if(DEBUG_TARGET EQUAL  1)      #注意DEBUG默认定义了
    set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g3")   #添加DEBUG选项
    # 编译成可执行文件
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/debug) 
    add_executable(main_D ${PROJECT_SRC})
    set(TARGET_NAME main_D)
else()
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/release) 
    add_executable(main ${PROJECT_SRC})
    set(TARGET_NAME main)
endif()

# 添加子目录
include_directories("${PROJECT_SOURCE_DIR}/include")
add_subdirectory(${PROJECT_PATH}/src)

# # 链接hellolib库，注意下面子目录src的CMakeLists
# link_directories("${PROJECT_SOURCE_DIR}/lib")
# target_link_libraries(opencvTest hellolib)

# 链接opencv库
# target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})

MESSAGE("------")
MESSAGE("TARGET_NAME: "${TARGET_NAME})
MESSAGE("DEBUG_TARGET: "${DEBUG_TARGET})


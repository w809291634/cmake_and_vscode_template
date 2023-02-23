CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# 项目名称
PROJECT(template)

# 找opencv package----------记得添加这个
find_package(OpenCV REQUIRED) 

set(CMAKE_C_COMPILER "g++")                     # 设置 C文件 的编译器
set(CMAKE_CXX_COMPILER "g++")                   # 设置 Cpp文件 的编译器
set(CMAKE_CXX_STANDARD 11)                      # C++ 的编译版本
set(CMAKE_VERBOSE_MAKEFILE ON)                  # 显示makefile的编译详细信息
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -Winline -std=c++11")        #设置 Cpp文件 的编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O2 -Winline -std=c++11")            #设置 C文件 的编译选项

# 设置工程文件夹路径
set(PROJECT_PATH ${PROJECT_SOURCE_DIR}/template)
# 代码路径
aux_source_directory(${PROJECT_PATH}/src PROJECT_SRC)
# 头文件路径
include_directories("${PROJECT_SOURCE_DIR}/include")
add_subdirectory(${PROJECT_PATH}/src)

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

# # 链接hellolib库，注意下面子目录src的CMakeLists
# link_directories("${PROJECT_SOURCE_DIR}/lib")
# target_link_libraries(opencvTest hellolib)

# 链接opencv库
# target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})

MESSAGE("------")
MESSAGE("TARGET_NAME: "${TARGET_NAME})
MESSAGE("DEBUG_TARGET: "${DEBUG_TARGET})


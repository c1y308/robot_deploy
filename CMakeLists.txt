cmake_minimum_required(VERSION 3.8)
project(MYACTUA_EtherCat)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(IGH_PATH /home/cat/ethercat)

# 库目录设置
link_directories(
    ${IGH_PATH}/lib
    /opt/etherlab/lib
)

# 编译静态/动态库
add_library(MYACTUA_EtherCat
    src/motor_control.cpp
    src/EthercatAdapterIGH.cpp
)

target_include_directories(MYACTUA_EtherCat PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${IGH_PATH}/include
    /opt/etherlab/include
)

# 链接依赖库
target_link_libraries(MYACTUA_EtherCat
    ethercat
    pthread
    rt
)

# 编译可执行文件
add_executable(simple_test examples/simple_test.cpp)
add_executable(debug_tool examples/debug_tool.cpp)

# 链接你自己的库
target_link_libraries(simple_test MYACTUA_EtherCat)
target_link_libraries(debug_tool  MYACTUA_EtherCat)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
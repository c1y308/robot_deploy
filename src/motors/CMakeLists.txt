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
    src/drivers/myact/motor_control.cpp
    src/protocol/ethercat/EthercatAdapterIGH.cpp
)

target_include_directories(MYACTUA_EtherCat PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/myact
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol/ethercat
    ${IGH_PATH}/include
    /opt/etherlab/include
)

# 目标库链接依赖库
target_link_libraries(MYACTUA_EtherCat
    ethercat
    pthread
    rt
)

# 编译可执行文件
add_executable(simple_test examples/simple_test.cpp)
add_executable(ID_test examples/ID_test.cpp)
add_executable(linear_test examples/linear_test.cpp)
add_executable(squat_stand examples/squat_stand.cpp)
add_executable(stop_read_status examples/stop_read_status.cpp)

# 链接库
target_link_libraries(simple_test MYACTUA_EtherCat)
target_link_libraries(ID_test MYACTUA_EtherCat)
target_link_libraries(linear_test MYACTUA_EtherCat)
target_link_libraries(squat_stand MYACTUA_EtherCat)
target_link_libraries(stop_read_status MYACTUA_EtherCat)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

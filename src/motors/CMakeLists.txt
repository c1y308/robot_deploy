cmake_minimum_required(VERSION 3.8)
project(MYACTUA_EtherCat)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)

set(IGH_PATH /home/cat/ethercat)

# 库目录设置
link_directories(
    ${IGH_PATH}/lib
    /opt/etherlab/lib
)

# 编译静态/动态库
add_library(MYACTUA_EtherCat
    src/drivers/myact/motor_control.cpp
    src/drivers/myact/motor_status_channel.cpp
    src/drivers/myact/motor_status_monitor.cpp
    src/drivers/myact/rt_event_dispatcher.cpp
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
add_executable(raw_pos_test examples/raw_pos_test.cpp)
add_executable(squat_stand examples/squat_stand.cpp)
add_executable(MIT_test examples/MIT_test.cpp)
add_executable(mit_pos_test examples/mit_pos_test.cpp)
add_executable(motor_response_latency_test examples/motor_response_latency_test.cpp)
add_executable(motor_realtime_channel_test tests/motor_realtime_channel_test.cpp)
target_include_directories(motor_response_latency_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../inference/include
)

# 链接库
target_link_libraries(simple_test MYACTUA_EtherCat)
target_link_libraries(ID_test MYACTUA_EtherCat)
target_link_libraries(linear_test MYACTUA_EtherCat)
target_link_libraries(raw_pos_test MYACTUA_EtherCat)
target_link_libraries(squat_stand MYACTUA_EtherCat)
target_link_libraries(MIT_test MYACTUA_EtherCat)
target_link_libraries(mit_pos_test MYACTUA_EtherCat)
target_link_libraries(motor_response_latency_test MYACTUA_EtherCat)
target_link_libraries(motor_realtime_channel_test MYACTUA_EtherCat)

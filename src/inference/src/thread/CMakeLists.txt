add_library(robot_thread_runtime STATIC thread_runtime.cpp)
target_compile_features(robot_thread_runtime PUBLIC cxx_std_17)
target_include_directories(robot_thread_runtime PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)
target_link_libraries(robot_thread_runtime PUBLIC pthread rt)

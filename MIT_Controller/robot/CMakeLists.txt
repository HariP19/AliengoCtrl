file(GLOB sources "src/*.cpp")
add_library(robot SHARED ${sources})

target_link_libraries(robot unitree_sdk common pthread lcm dynacore_param_handler)
target_include_directories(robot PUBLIC
                        "include/"
                        "../common/include/"
                        "../third-party"
                        "../third-party/ParamHandler"
                        ${CMAKE_BINARY_DIR}
)
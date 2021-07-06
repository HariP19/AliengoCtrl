file(GLOB_RECURSE sources "src/*.cpp")

add_library(common SHARED ${sources})
target_include_directories(common PUBLIC 
                        "${PROJECT_SOURCE_DIR}/MIT_Controller/third-party/osqp/include"
                        "${PROJECT_SOURCE_DIR}/MIT_Controller/third-party/ParamHandler"
                        "include/"
                        "${CMAKE_BINARY_DIR}"
)
target_link_libraries(common dynacore_param_handler osqp)


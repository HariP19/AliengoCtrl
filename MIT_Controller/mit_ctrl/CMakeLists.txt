include_directories("./")
include_directories("${PROJECT_SOURCE_DIR}/MIT_Controller/common/include/")
include_directories("${PROJECT_SOURCE_DIR}/MIT_Controller/third-party/")
include_directories("${PROJECT_SOURCE_DIR}/MIT_Controller/")
include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(convexMPC)
add_subdirectory(WBC)
add_subdirectory(WBC_Ctrl)
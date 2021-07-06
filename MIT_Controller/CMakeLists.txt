SET (THIS_COM "${CMAKE_CURRENT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

add_subdirectory(common)
add_subdirectory(third-party)
add_subdirectory(robot)
add_subdirectory(mit_ctrl)
add_subdirectory(fsm_states)

add_executable(run_controller main.cpp MIT_Controller.cpp)
target_include_directories(run_controller PRIVATE "robot/include/" "common/include" ${CMAKE_BINARY_DIR})
target_link_libraries(run_controller robot common fsm_states convexMPC WBC_Ctrl)
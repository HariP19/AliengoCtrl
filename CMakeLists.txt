cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

set(BUILD_TYPE_RELEASE TRUE)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -Wall \
-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
-Winit-self -Wmissing-include-dirs -Wno-overloaded-virtual \
-Wshadow -Wsign-promo -Werror -Wno-missing-field-initializers -Wno-unused-result -Wno-unused-parameter")

set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")

add_subdirectory(unitree_sdk)
add_subdirectory(MIT_Controller)
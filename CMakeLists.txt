cmake_minimum_required(VERSION 3.5)

project(gps_waypoint CXX)

# Find Dronecode SDK library
find_package(dronecode_sdk REQUIRED)

# Add executable
add_executable(gps_waypoint fly_there.cpp)

# Link Dronecode SDK library
target_link_libraries(gps_waypoint dronecode_sdk)

# Set C++ standard to 11
set_target_properties(gps_waypoint PROPERTIES CXX_STANDARD 11)

# Enable warnings
if (MSVC)
    target_compile_options(gps_waypoint PRIVATE /W4 /WX)
else()
    target_compile_options(gps_waypoint PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

# Install executable
install(TARGETS gps_waypoint DESTINATION bin)

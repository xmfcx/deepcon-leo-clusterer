cmake_minimum_required(VERSION 2.8.3)
project(clusterer)

set(CMAKE_CXX_STANDARD 14)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3")
set(CMAKE_BUILD_TYPE RELEASE)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        sensor_msgs
        roscpp
        rospy
        )

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}_exec
        src/Clusterer.cpp
        src/PclStuff.cpp)

add_dependencies(${PROJECT_NAME}_exec
        ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_exec
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
cmake_minimum_required(VERSION 2.8.3)
project(move_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        move_base_msgs
        nav_core
        std_srvs
        tf
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenSSL)
find_package(ZLIB)

set (WEBSOCKETPP_ROOT ${CMAKE_CURRENT_LIST_DIR}/include)
set (WEBSOCKETPP_INCLUDE ${WEBSOCKETPP_ROOT}/websocketpp)
#set (WEBSOCKETPP_BUILD_ROOT ${CMAKE_BINARY_DIR})
#set (WEBSOCKETPP_BIN ${WEBSOCKETPP_BUILD_ROOT}/bin)
set (WEBSOCKETPP_LIB ${CMAKE_CURRENT_LIST_DIR}/lib)

add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
 	${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Add main library
#add_subdirectory (websocketpp)

# move_base
add_library(move_base
  src/move_base.cpp
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11")

target_link_libraries(move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    -lrt
    )
    
add_dependencies(move_base move_base_gencfg)
add_dependencies(move_base geometry_msgs_generate_messages_cpp)

add_executable(move_base_node
  src/move_base_node.cpp
)
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

install(
    TARGETS
        move_base
        move_base_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

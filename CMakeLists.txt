cmake_minimum_required(VERSION 2.8.3)
project(quadSYSspline)
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
find_package(Eigen3 REQUIRED)

# enable dashboard scripting
include (CTest)
set (CTEST_PROJECT_NAME "quadSYSspline")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS)

###########
## Build ##
###########
include_directories( ${PROJECT_SOURCE_DIR}/include )
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
                     
#add_library(function_approximation_lib src/quadSYSspline.cpp)
#add_library(actor_critic_lib src/actor_critic_FA.cpp)

add_executable(quadSYSspline src/quadSYSspline.cpp)

#target_link_libraries(actor_critic_lib function_approximation_lib)
#target_link_libraries(CartPoleBalancing actor_critic_lib)

#############
## Install ##
#############

# To Do

#############
## Testing ##
#############

# To Do

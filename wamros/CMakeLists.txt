#WAMROS
#Dang - EPFL
#TODO: Should this deal with the different setups?
#           Xenomai/RTAI  PCAN/PCI  etc
#      Should this deal with building btclient?

#Boilerplate-----------------------------------------
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#----------------------------------------------------
set(CMAKE_VERBOSE_MAKEFILE true)

#don't need since they are provided by the wam_msgs package
#we have both messages and services
#rosbuild_genmsg()
#rosbuild_gensrv()

#Link bt - maybe find with a system locate call?
include_directories("/usr/local/include/btclient")
link_directories("/usr/local/lib/btclient")

#Link Xenomai - Likewise
include_directories("/usr/xenomai/include")
link_directories("/usr/xenomai/lib")

#Here's what we're making
rosbuild_add_executable(wamros_node src/wamros_node.cpp src/WamNode.cpp)

#Flags ---------------------------------------------

#These may not be necessary
rosbuild_add_link_flags(wamros_node -Wl -rdynamic --cref)
rosbuild_add_compile_flags(wamros_node -Wall -pipe)

#These are necessary
rosbuild_add_compile_flags(wamros_node -DXENOMAI)  #Tells bt headers


#Linking----------------------------------
target_link_libraries(wamros_node btwam pthread m btsystem curses)
target_link_libraries(wamros_node xenomai native) #might change if not xenomai
target_link_libraries(wamros_node pcan) #might change if not pcan


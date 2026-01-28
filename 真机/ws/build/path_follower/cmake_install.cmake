# Install script for directory: /home/xu/ws1/src/path_follower

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/xu/ws1/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/xu/ws1/build/path_follower/catkin_generated/installspace/path_follower.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_follower/cmake" TYPE FILE FILES
    "/home/xu/ws1/build/path_follower/catkin_generated/installspace/path_followerConfig.cmake"
    "/home/xu/ws1/build/path_follower/catkin_generated/installspace/path_followerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_follower" TYPE FILE FILES "/home/xu/ws1/src/path_follower/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/path_follower" TYPE EXECUTABLE FILES "/home/xu/ws1/devel/lib/path_follower/path_follower")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower"
         OLD_RPATH "/usr/lib/liborocos-kdl.so:/opt/ros/noetic/lib/libtf2_ros.so:/opt/ros/noetic/lib/libactionlib.so:/opt/ros/noetic/lib/libmessage_filters.so:/opt/ros/noetic/lib/libroscpp.so:/usr/lib/x86_64-linux-gnu/libpthread.so:/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0:/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0:/opt/ros/noetic/lib/librosconsole.so:/opt/ros/noetic/lib/librosconsole_log4cxx.so:/opt/ros/noetic/lib/librosconsole_backend_interface.so:/usr/lib/x86_64-linux-gnu/liblog4cxx.so:/usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0:/opt/ros/noetic/lib/libxmlrpcpp.so:/opt/ros/noetic/lib/libtf2.so:/opt/ros/noetic/lib/libroscpp_serialization.so:/opt/ros/noetic/lib/librostime.so:/usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0:/opt/ros/noetic/lib/libcpp_common.so:/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0:/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0:/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/path_follower/path_follower")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_follower/launch" TYPE DIRECTORY FILES "/home/xu/ws1/src/path_follower/launch/")
endif()


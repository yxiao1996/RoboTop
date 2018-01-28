# Install script for directory: /home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yxiao1996/workspace/RoboTop/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocon_msgs/msg" TYPE FILE FILES
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocon_msgs/cmake" TYPE FILE FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/Robocon/robocon_msgs/catkin_generated/installspace/robocon_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/include/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/share/roseus/ros/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/share/common-lisp/ros/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/share/gennodejs/ros/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/lib/python2.7/dist-packages/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/devel/lib/python2.7/dist-packages/robocon_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/Robocon/robocon_msgs/catkin_generated/installspace/robocon_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocon_msgs/cmake" TYPE FILE FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/Robocon/robocon_msgs/catkin_generated/installspace/robocon_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocon_msgs/cmake" TYPE FILE FILES
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/Robocon/robocon_msgs/catkin_generated/installspace/robocon_msgsConfig.cmake"
    "/home/yxiao1996/workspace/RoboTop/catkin_ws/build/Robocon/robocon_msgs/catkin_generated/installspace/robocon_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robocon_msgs" TYPE FILE FILES "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/package.xml")
endif()


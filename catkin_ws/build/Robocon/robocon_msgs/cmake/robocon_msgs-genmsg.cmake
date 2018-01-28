# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robocon_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Irobocon_msgs:/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robocon_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_custom_target(_robocon_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robocon_msgs" "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" ""
)

get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_custom_target(_robocon_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robocon_msgs" "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" ""
)

get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_custom_target(_robocon_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robocon_msgs" "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_custom_target(_robocon_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robocon_msgs" "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" ""
)

get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_custom_target(_robocon_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robocon_msgs" "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_cpp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_cpp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_cpp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_cpp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(robocon_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robocon_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robocon_msgs_generate_messages robocon_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_cpp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_cpp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_cpp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_cpp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_cpp _robocon_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robocon_msgs_gencpp)
add_dependencies(robocon_msgs_gencpp robocon_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robocon_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
)
_generate_msg_eus(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
)
_generate_msg_eus(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
)
_generate_msg_eus(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
)
_generate_msg_eus(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(robocon_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robocon_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robocon_msgs_generate_messages robocon_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_eus _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_eus _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_eus _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_eus _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_eus _robocon_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robocon_msgs_geneus)
add_dependencies(robocon_msgs_geneus robocon_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robocon_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_lisp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_lisp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_lisp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
)
_generate_msg_lisp(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(robocon_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robocon_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robocon_msgs_generate_messages robocon_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_lisp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_lisp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_lisp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_lisp _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_lisp _robocon_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robocon_msgs_genlisp)
add_dependencies(robocon_msgs_genlisp robocon_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robocon_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
)
_generate_msg_nodejs(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
)
_generate_msg_nodejs(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
)
_generate_msg_nodejs(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
)
_generate_msg_nodejs(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robocon_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robocon_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robocon_msgs_generate_messages robocon_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_nodejs _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_nodejs _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_nodejs _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_nodejs _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_nodejs _robocon_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robocon_msgs_gennodejs)
add_dependencies(robocon_msgs_gennodejs robocon_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robocon_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
)
_generate_msg_py(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
)
_generate_msg_py(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
)
_generate_msg_py(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
)
_generate_msg_py(robocon_msgs
  "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(robocon_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robocon_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robocon_msgs_generate_messages robocon_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/chassis_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_py _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/remoter_msg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_py _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/LaserScan.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_py _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/commonMsg.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_py _robocon_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yxiao1996/workspace/RoboTop/catkin_ws/src/Robocon/robocon_msgs/msg/pwmset.msg" NAME_WE)
add_dependencies(robocon_msgs_generate_messages_py _robocon_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robocon_msgs_genpy)
add_dependencies(robocon_msgs_genpy robocon_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robocon_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robocon_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robocon_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robocon_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(robocon_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robocon_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robocon_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robocon_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(robocon_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robocon_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robocon_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robocon_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(robocon_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robocon_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robocon_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robocon_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(robocon_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robocon_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robocon_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robocon_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(robocon_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()

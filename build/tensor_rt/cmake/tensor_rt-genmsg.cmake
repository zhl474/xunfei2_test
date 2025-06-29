# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tensor_rt: 2 messages, 2 services")

set(MSG_I_FLAGS "-Itensor_rt:/home/zythyra/gazebo_test_ws/src/tensor_rt/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tensor_rt_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_custom_target(_tensor_rt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tensor_rt" "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" ""
)

get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_custom_target(_tensor_rt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tensor_rt" "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" ""
)

get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_custom_target(_tensor_rt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tensor_rt" "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" ""
)

get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_custom_target(_tensor_rt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tensor_rt" "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
)
_generate_msg_cpp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
)

### Generating Services
_generate_srv_cpp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
)
_generate_srv_cpp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
)

### Generating Module File
_generate_module_cpp(tensor_rt
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tensor_rt_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tensor_rt_generate_messages tensor_rt_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_cpp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_cpp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_cpp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_cpp _tensor_rt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensor_rt_gencpp)
add_dependencies(tensor_rt_gencpp tensor_rt_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensor_rt_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
)
_generate_msg_eus(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
)

### Generating Services
_generate_srv_eus(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
)
_generate_srv_eus(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
)

### Generating Module File
_generate_module_eus(tensor_rt
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tensor_rt_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tensor_rt_generate_messages tensor_rt_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_eus _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_eus _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_eus _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_eus _tensor_rt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensor_rt_geneus)
add_dependencies(tensor_rt_geneus tensor_rt_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensor_rt_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
)
_generate_msg_lisp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
)

### Generating Services
_generate_srv_lisp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
)
_generate_srv_lisp(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
)

### Generating Module File
_generate_module_lisp(tensor_rt
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tensor_rt_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tensor_rt_generate_messages tensor_rt_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_lisp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_lisp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_lisp _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_lisp _tensor_rt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensor_rt_genlisp)
add_dependencies(tensor_rt_genlisp tensor_rt_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensor_rt_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
)
_generate_msg_nodejs(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
)

### Generating Services
_generate_srv_nodejs(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
)
_generate_srv_nodejs(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
)

### Generating Module File
_generate_module_nodejs(tensor_rt
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tensor_rt_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tensor_rt_generate_messages tensor_rt_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_nodejs _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_nodejs _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_nodejs _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_nodejs _tensor_rt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensor_rt_gennodejs)
add_dependencies(tensor_rt_gennodejs tensor_rt_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensor_rt_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
)
_generate_msg_py(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
)

### Generating Services
_generate_srv_py(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
)
_generate_srv_py(tensor_rt
  "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
)

### Generating Module File
_generate_module_py(tensor_rt
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tensor_rt_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tensor_rt_generate_messages tensor_rt_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_1.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_py _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/msg/msg_2.msg" NAME_WE)
add_dependencies(tensor_rt_generate_messages_py _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/Messages.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_py _tensor_rt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zythyra/gazebo_test_ws/src/tensor_rt/srv/SimCarComm.srv" NAME_WE)
add_dependencies(tensor_rt_generate_messages_py _tensor_rt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tensor_rt_genpy)
add_dependencies(tensor_rt_genpy tensor_rt_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tensor_rt_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tensor_rt
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tensor_rt_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(tensor_rt_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tensor_rt
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tensor_rt_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(tensor_rt_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tensor_rt
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tensor_rt_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(tensor_rt_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tensor_rt
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tensor_rt_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(tensor_rt_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tensor_rt
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tensor_rt_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(tensor_rt_generate_messages_py sensor_msgs_generate_messages_py)
endif()

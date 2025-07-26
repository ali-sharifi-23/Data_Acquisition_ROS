# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kamal_ocalib: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ikamal_ocalib:/home/ali/catkin_ws/src/kamal_ocalib/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kamal_ocalib_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_custom_target(_kamal_ocalib_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kamal_ocalib" "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" ""
)

get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_custom_target(_kamal_ocalib_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kamal_ocalib" "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" ""
)

get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_custom_target(_kamal_ocalib_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kamal_ocalib" "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_cpp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_cpp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib
)

### Generating Services

### Generating Module File
_generate_module_cpp(kamal_ocalib
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kamal_ocalib_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kamal_ocalib_generate_messages kamal_ocalib_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_cpp _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_cpp _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_cpp _kamal_ocalib_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kamal_ocalib_gencpp)
add_dependencies(kamal_ocalib_gencpp kamal_ocalib_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kamal_ocalib_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_eus(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_eus(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib
)

### Generating Services

### Generating Module File
_generate_module_eus(kamal_ocalib
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(kamal_ocalib_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(kamal_ocalib_generate_messages kamal_ocalib_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_eus _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_eus _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_eus _kamal_ocalib_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kamal_ocalib_geneus)
add_dependencies(kamal_ocalib_geneus kamal_ocalib_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kamal_ocalib_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_lisp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_lisp(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib
)

### Generating Services

### Generating Module File
_generate_module_lisp(kamal_ocalib
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kamal_ocalib_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kamal_ocalib_generate_messages kamal_ocalib_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_lisp _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_lisp _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_lisp _kamal_ocalib_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kamal_ocalib_genlisp)
add_dependencies(kamal_ocalib_genlisp kamal_ocalib_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kamal_ocalib_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_nodejs(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_nodejs(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib
)

### Generating Services

### Generating Module File
_generate_module_nodejs(kamal_ocalib
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(kamal_ocalib_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(kamal_ocalib_generate_messages kamal_ocalib_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_nodejs _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_nodejs _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_nodejs _kamal_ocalib_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kamal_ocalib_gennodejs)
add_dependencies(kamal_ocalib_gennodejs kamal_ocalib_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kamal_ocalib_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_py(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib
)
_generate_msg_py(kamal_ocalib
  "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib
)

### Generating Services

### Generating Module File
_generate_module_py(kamal_ocalib
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kamal_ocalib_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kamal_ocalib_generate_messages kamal_ocalib_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/encoderStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_py _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/stereoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_py _kamal_ocalib_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ali/catkin_ws/src/kamal_ocalib/msg/monoStream.msg" NAME_WE)
add_dependencies(kamal_ocalib_generate_messages_py _kamal_ocalib_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kamal_ocalib_genpy)
add_dependencies(kamal_ocalib_genpy kamal_ocalib_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kamal_ocalib_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kamal_ocalib
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(kamal_ocalib_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kamal_ocalib
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(kamal_ocalib_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kamal_ocalib
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(kamal_ocalib_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kamal_ocalib
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(kamal_ocalib_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib)
  install(CODE "execute_process(COMMAND \"/home/ali/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kamal_ocalib
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(kamal_ocalib_generate_messages_py std_msgs_generate_messages_py)
endif()

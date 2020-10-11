# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "espeleo_planner: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(espeleo_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_custom_target(_espeleo_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "espeleo_planner" "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" ""
)

get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_custom_target(_espeleo_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "espeleo_planner" "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/espeleo_planner
)
_generate_srv_cpp(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/espeleo_planner
)

### Generating Module File
_generate_module_cpp(espeleo_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/espeleo_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(espeleo_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(espeleo_planner_generate_messages espeleo_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_cpp _espeleo_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_cpp _espeleo_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(espeleo_planner_gencpp)
add_dependencies(espeleo_planner_gencpp espeleo_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS espeleo_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/espeleo_planner
)
_generate_srv_eus(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/espeleo_planner
)

### Generating Module File
_generate_module_eus(espeleo_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/espeleo_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(espeleo_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(espeleo_planner_generate_messages espeleo_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_eus _espeleo_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_eus _espeleo_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(espeleo_planner_geneus)
add_dependencies(espeleo_planner_geneus espeleo_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS espeleo_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/espeleo_planner
)
_generate_srv_lisp(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/espeleo_planner
)

### Generating Module File
_generate_module_lisp(espeleo_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/espeleo_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(espeleo_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(espeleo_planner_generate_messages espeleo_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_lisp _espeleo_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_lisp _espeleo_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(espeleo_planner_genlisp)
add_dependencies(espeleo_planner_genlisp espeleo_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS espeleo_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/espeleo_planner
)
_generate_srv_nodejs(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/espeleo_planner
)

### Generating Module File
_generate_module_nodejs(espeleo_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/espeleo_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(espeleo_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(espeleo_planner_generate_messages espeleo_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_nodejs _espeleo_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_nodejs _espeleo_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(espeleo_planner_gennodejs)
add_dependencies(espeleo_planner_gennodejs espeleo_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS espeleo_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner
)
_generate_srv_py(espeleo_planner
  "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner
)

### Generating Module File
_generate_module_py(espeleo_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(espeleo_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(espeleo_planner_generate_messages espeleo_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processAllFrontiers.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_py _espeleo_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/h3ct0r/catkin_ws_espeleo/src/espeleo_planner/espeleo_planner/srv/processFrontier.srv" NAME_WE)
add_dependencies(espeleo_planner_generate_messages_py _espeleo_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(espeleo_planner_genpy)
add_dependencies(espeleo_planner_genpy espeleo_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS espeleo_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/espeleo_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/espeleo_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(espeleo_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/espeleo_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/espeleo_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(espeleo_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/espeleo_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/espeleo_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(espeleo_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/espeleo_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/espeleo_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(espeleo_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/espeleo_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(espeleo_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dobot_magician_driver: 0 messages, 2 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dobot_magician_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_custom_target(_dobot_magician_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dobot_magician_driver" "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" ""
)

get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_custom_target(_dobot_magician_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dobot_magician_driver" "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dobot_magician_driver
)
_generate_srv_cpp(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dobot_magician_driver
)

### Generating Module File
_generate_module_cpp(dobot_magician_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dobot_magician_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dobot_magician_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dobot_magician_driver_generate_messages dobot_magician_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_cpp _dobot_magician_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_cpp _dobot_magician_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dobot_magician_driver_gencpp)
add_dependencies(dobot_magician_driver_gencpp dobot_magician_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dobot_magician_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dobot_magician_driver
)
_generate_srv_eus(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dobot_magician_driver
)

### Generating Module File
_generate_module_eus(dobot_magician_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dobot_magician_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dobot_magician_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dobot_magician_driver_generate_messages dobot_magician_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_eus _dobot_magician_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_eus _dobot_magician_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dobot_magician_driver_geneus)
add_dependencies(dobot_magician_driver_geneus dobot_magician_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dobot_magician_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dobot_magician_driver
)
_generate_srv_lisp(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dobot_magician_driver
)

### Generating Module File
_generate_module_lisp(dobot_magician_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dobot_magician_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dobot_magician_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dobot_magician_driver_generate_messages dobot_magician_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_lisp _dobot_magician_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_lisp _dobot_magician_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dobot_magician_driver_genlisp)
add_dependencies(dobot_magician_driver_genlisp dobot_magician_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dobot_magician_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dobot_magician_driver
)
_generate_srv_nodejs(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dobot_magician_driver
)

### Generating Module File
_generate_module_nodejs(dobot_magician_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dobot_magician_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dobot_magician_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dobot_magician_driver_generate_messages dobot_magician_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_nodejs _dobot_magician_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_nodejs _dobot_magician_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dobot_magician_driver_gennodejs)
add_dependencies(dobot_magician_driver_gennodejs dobot_magician_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dobot_magician_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver
)
_generate_srv_py(dobot_magician_driver
  "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver
)

### Generating Module File
_generate_module_py(dobot_magician_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dobot_magician_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dobot_magician_driver_generate_messages dobot_magician_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetTargetPoints.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_py _dobot_magician_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tim/catkin_ws/src/new dobot/dobot_magician_driver/srv/SetSuctionCup.srv" NAME_WE)
add_dependencies(dobot_magician_driver_generate_messages_py _dobot_magician_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dobot_magician_driver_genpy)
add_dependencies(dobot_magician_driver_genpy dobot_magician_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dobot_magician_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dobot_magician_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dobot_magician_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dobot_magician_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dobot_magician_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dobot_magician_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dobot_magician_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dobot_magician_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dobot_magician_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dobot_magician_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "prediction: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iprediction:/home/stier/catkin_ws/src/prediction/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(prediction_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_custom_target(_prediction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "prediction" "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" "prediction/TrackedPoint"
)

get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_custom_target(_prediction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "prediction" "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" "std_msgs/Header:prediction/TrackedPoint:prediction/PredictedObjectPath"
)

get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_custom_target(_prediction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "prediction" "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" "prediction/TrackedPoint"
)

get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_custom_target(_prediction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "prediction" "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" "std_msgs/Header:prediction/TrackedPoint:prediction/TrackedObjectPose"
)

get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_custom_target(_prediction_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "prediction" "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
)
_generate_msg_cpp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
)
_generate_msg_cpp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
)
_generate_msg_cpp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
)
_generate_msg_cpp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
)

### Generating Services

### Generating Module File
_generate_module_cpp(prediction
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(prediction_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(prediction_generate_messages prediction_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_dependencies(prediction_generate_messages_cpp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_cpp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_dependencies(prediction_generate_messages_cpp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_cpp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_dependencies(prediction_generate_messages_cpp _prediction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(prediction_gencpp)
add_dependencies(prediction_gencpp prediction_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS prediction_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
)
_generate_msg_eus(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
)
_generate_msg_eus(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
)
_generate_msg_eus(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
)
_generate_msg_eus(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
)

### Generating Services

### Generating Module File
_generate_module_eus(prediction
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(prediction_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(prediction_generate_messages prediction_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_dependencies(prediction_generate_messages_eus _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_eus _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_dependencies(prediction_generate_messages_eus _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_eus _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_dependencies(prediction_generate_messages_eus _prediction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(prediction_geneus)
add_dependencies(prediction_geneus prediction_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS prediction_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
)
_generate_msg_lisp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
)
_generate_msg_lisp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
)
_generate_msg_lisp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
)
_generate_msg_lisp(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
)

### Generating Services

### Generating Module File
_generate_module_lisp(prediction
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(prediction_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(prediction_generate_messages prediction_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_dependencies(prediction_generate_messages_lisp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_lisp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_dependencies(prediction_generate_messages_lisp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_lisp _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_dependencies(prediction_generate_messages_lisp _prediction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(prediction_genlisp)
add_dependencies(prediction_genlisp prediction_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS prediction_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
)
_generate_msg_nodejs(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
)
_generate_msg_nodejs(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
)
_generate_msg_nodejs(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
)
_generate_msg_nodejs(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
)

### Generating Services

### Generating Module File
_generate_module_nodejs(prediction
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(prediction_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(prediction_generate_messages prediction_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_dependencies(prediction_generate_messages_nodejs _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_nodejs _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_dependencies(prediction_generate_messages_nodejs _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_nodejs _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_dependencies(prediction_generate_messages_nodejs _prediction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(prediction_gennodejs)
add_dependencies(prediction_gennodejs prediction_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS prediction_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
)
_generate_msg_py(prediction
  "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
)
_generate_msg_py(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
)
_generate_msg_py(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg;/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
)
_generate_msg_py(prediction
  "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
)

### Generating Services

### Generating Module File
_generate_module_py(prediction
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(prediction_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(prediction_generate_messages prediction_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPath.msg" NAME_WE)
add_dependencies(prediction_generate_messages_py _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/PredictedObjectPathList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_py _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPose.msg" NAME_WE)
add_dependencies(prediction_generate_messages_py _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedObjectPoseList.msg" NAME_WE)
add_dependencies(prediction_generate_messages_py _prediction_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/stier/catkin_ws/src/prediction/msg/TrackedPoint.msg" NAME_WE)
add_dependencies(prediction_generate_messages_py _prediction_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(prediction_genpy)
add_dependencies(prediction_genpy prediction_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS prediction_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/prediction
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(prediction_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/prediction
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(prediction_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/prediction
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(prediction_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/prediction
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(prediction_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/prediction
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(prediction_generate_messages_py std_msgs_generate_messages_py)
endif()

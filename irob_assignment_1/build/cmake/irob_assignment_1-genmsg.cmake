# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "irob_assignment_1: 7 messages, 1 services")

set(MSG_I_FLAGS "-Iirob_assignment_1:/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(irob_assignment_1_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" "nav_msgs/Path:geometry_msgs/PointStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" "actionlib_msgs/GoalID:nav_msgs/Path:actionlib_msgs/GoalStatus:irob_assignment_1/GetNextGoalFeedback:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" ""
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" "actionlib_msgs/GoalID:nav_msgs/Path:irob_assignment_1/GetNextGoalResult:actionlib_msgs/GoalStatus:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" "nav_msgs/Path:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" "actionlib_msgs/GoalID:nav_msgs/Path:irob_assignment_1/GetNextGoalGoal:actionlib_msgs/GoalStatus:irob_assignment_1/GetNextGoalFeedback:irob_assignment_1/GetNextGoalActionGoal:geometry_msgs/Pose:irob_assignment_1/GetNextGoalActionFeedback:irob_assignment_1/GetNextGoalActionResult:std_msgs/Header:irob_assignment_1/GetNextGoalResult:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" "actionlib_msgs/GoalID:irob_assignment_1/GetNextGoalGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_custom_target(_irob_assignment_1_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "irob_assignment_1" "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" "nav_msgs/Path:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)

### Generating Services
_generate_srv_cpp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
)

### Generating Module File
_generate_module_cpp(irob_assignment_1
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(irob_assignment_1_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(irob_assignment_1_generate_messages irob_assignment_1_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_cpp _irob_assignment_1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(irob_assignment_1_gencpp)
add_dependencies(irob_assignment_1_gencpp irob_assignment_1_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS irob_assignment_1_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)

### Generating Services
_generate_srv_eus(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
)

### Generating Module File
_generate_module_eus(irob_assignment_1
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(irob_assignment_1_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(irob_assignment_1_generate_messages irob_assignment_1_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_eus _irob_assignment_1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(irob_assignment_1_geneus)
add_dependencies(irob_assignment_1_geneus irob_assignment_1_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS irob_assignment_1_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)

### Generating Services
_generate_srv_lisp(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
)

### Generating Module File
_generate_module_lisp(irob_assignment_1
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(irob_assignment_1_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(irob_assignment_1_generate_messages irob_assignment_1_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_lisp _irob_assignment_1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(irob_assignment_1_genlisp)
add_dependencies(irob_assignment_1_genlisp irob_assignment_1_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS irob_assignment_1_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)

### Generating Services
_generate_srv_nodejs(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
)

### Generating Module File
_generate_module_nodejs(irob_assignment_1
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(irob_assignment_1_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(irob_assignment_1_generate_messages irob_assignment_1_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_nodejs _irob_assignment_1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(irob_assignment_1_gennodejs)
add_dependencies(irob_assignment_1_gennodejs irob_assignment_1_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS irob_assignment_1_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)
_generate_msg_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)

### Generating Services
_generate_srv_py(irob_assignment_1
  "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PointStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
)

### Generating Module File
_generate_module_py(irob_assignment_1
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(irob_assignment_1_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(irob_assignment_1_generate_messages irob_assignment_1_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/srv/GetSetpoint.srv" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalResult.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalAction.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalActionGoal.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quini/catkin_ws/src/irob_assignment_1/build/devel/share/irob_assignment_1/msg/GetNextGoalFeedback.msg" NAME_WE)
add_dependencies(irob_assignment_1_generate_messages_py _irob_assignment_1_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(irob_assignment_1_genpy)
add_dependencies(irob_assignment_1_genpy irob_assignment_1_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS irob_assignment_1_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/irob_assignment_1
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(irob_assignment_1_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(irob_assignment_1_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(irob_assignment_1_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(irob_assignment_1_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/irob_assignment_1
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(irob_assignment_1_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(irob_assignment_1_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(irob_assignment_1_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(irob_assignment_1_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/irob_assignment_1
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(irob_assignment_1_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(irob_assignment_1_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(irob_assignment_1_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(irob_assignment_1_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/irob_assignment_1
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(irob_assignment_1_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(irob_assignment_1_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(irob_assignment_1_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(irob_assignment_1_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/irob_assignment_1
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(irob_assignment_1_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(irob_assignment_1_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(irob_assignment_1_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(irob_assignment_1_generate_messages_py std_msgs_generate_messages_py)
endif()

# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;nav_msgs;roscpp;rospy;std_msgs;tf;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldwa_planner_lib;-la_star_lib".split(';') if "-ldwa_planner_lib;-la_star_lib" != "" else []
PROJECT_NAME = "robot_navigation"
PROJECT_SPACE_DIR = "/home/duykhang0709/thesisAutonomous_ws/install"
PROJECT_VERSION = "0.0.1"

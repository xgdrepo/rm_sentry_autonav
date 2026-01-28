# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;geometry_msgs;std_msgs;serial".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lserial_com".split(';') if "-lserial_com" != "" else []
PROJECT_NAME = "serial_com"
PROJECT_SPACE_DIR = "/home/xu/ws1/install"
PROJECT_VERSION = "1.0.0"

cmake_minimum_required(VERSION 2.8.3)

# add servo and json libaries
link_directories($ENV{EXT_LIB_DIR})

# add ros include dir
include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(hello
  SRCS firmware_node.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  PORT "/dev/$ENV{CURRENT_ARDUINO_PORT}"
)
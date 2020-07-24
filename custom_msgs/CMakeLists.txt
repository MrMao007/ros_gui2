cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        rospy
                        std_msgs
                        message_generation
                        geometry_msgs
                        nav_msgs)

add_message_files(DIRECTORY msg/state_machine FILES
                  # MachineDescription.msg
                  # State.msg
                  # Transition.msg
                  )

add_message_files(DIRECTORY
                  msg/detection
                  FILES
                  FaceInfo.msg
                  DetectedFaces.msg
                  NormalizedFaces.msg)

add_message_files(DIRECTORY
                  msg/display
                  FILES
                  TextCmd.msg)

add_message_files(DIRECTORY
                  msg/virtual_layer
                  FILES
                  Door.msg
                  Form.msg
                  Obstacles.msg
                  Zone.msg)

add_message_files(DIRECTORY
                  msg/supervisor
                  FILES
                  AppCmd.msg
                  Dependency.msg
                  Service.msg
                  Process.msg)

add_message_files(DIRECTORY
                  msg/nav
                  FILES
                  Map64.msg)

add_message_files(DIRECTORY
                  msg/image
                  FILES
                  Image64.msg)

add_message_files(DIRECTORY
                  msg/control
                  FILES
                  HeadCmd.msg
                  HeadPose.msg
                  PhoneCmdVel.msg)

add_message_files(DIRECTORY
                  msg/alpha1s
                  FILES
                  BatteryState.msg)

add_message_files(DIRECTORY
                  msg/hardware
                  FILES
                  ColorRGB.msg
                  Leds.msg)

add_message_files(DIRECTORY
                  msg/create
                  FILES
                  Bumper.msg
                  ChargingState.msg
                  Mode.msg
                  DefineSong.msg
                  PlaySong.msg)

add_message_files(DIRECTORY
                  msg/rtc
                  FILES
                  MessageRTC.msg)

add_message_files(DIRECTORY
                  msg/console
                  FILES
                  Log.msg)
# For Arduino
add_message_files(DIRECTORY
                  msg
                  FILES
                  LedRGBColors.msg
                  Battery.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  nav_msgs)

catkin_package(CATKIN_DEPENDS
               roscpp
               rospy
               message_runtime
               std_msgs
               geometry_msgs
               nav_msgs)

include_directories(${catkin_INCLUDE_DIRS})

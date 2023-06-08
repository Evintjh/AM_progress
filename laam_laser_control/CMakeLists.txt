cmake_minimum_required(VERSION 2.8.3)
project(laam_laser_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
  acoustic_monitoring_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_python_setup()


generate_dynamic_reconfigure_options(
    config/Pid.cfg
)


add_message_files(
  FILES
  MsgPower.msg
  MsgStart.msg
  MsgControl.msg
  MsgInfo.msg
  MsgEmission.msg
  MsgSetpoint.msg
  MsgStatus.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime roscpp std_msgs dynamic_reconfigure
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


add_executable(main src/main.cpp src/CPP_pid.cpp )
#add_executable(CPP_plant_sim src/CPP_plant_sim.cpp)
add_executable(simu_time src/sim_time.cpp)

add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(CPP_plant_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simu_time ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

cmake_policy(SET CMP0079 NEW)
target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(simu_time ${catkin_LIBRARIES})
#target_link_libraries(CPP_plant_sim ${catkin_LIBRARIES})

#install(TARGETS main CPP_plant_sim 
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS main  sim_time
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


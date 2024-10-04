cmake_minimum_required(VERSION 3.0.2)
project(jackal_controller)

add_compile_options(-std=c++11 -O3)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  cv_bridge
  tf
  tf2
  
  std_msgs
  visualization_msgs
  message_generation
  sensor_msgs
  dynamic_reconfigure
)

#set(acados_include "${PROJECT_SOURCE_DIR}/../acados/include")
#set(acados_lib "${PROJECT_SOURCE_DIR}/../acados/lib")
set(acados_include "/home/pc2004/acados/include")
set(acados_lib "/home/pc2004/acados/lib")
#set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/jackal_full_model/c_generated_code)

add_message_files(
    FILES
    StampedModelStates.msg
    JackalState.msg
    JackalStateStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jackal_controller
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2 message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
include
/usr/local/include

${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}

${acados_include}
${acados_include}/blasfeo/include/
${acados_include}/hpipm/include/
${acados_include}/acados/
${acados_include}/qpOASES_e/

${PROJECT_SOURCE_DIR}/src/Eigen-3.3
${PROJECT_SOURCE_DIR}/scripts/c_generated_code
${PROJECT_SOURCE_DIR}/scripts/c_generated_code/jackal_model
)

link_directories(${PROJECT_SOURCE_DIR}/scripts/c_generated_code)
link_directories(/usr/local/lib)

#add_executable(acados_mpc src/main.cpp src/ACADOS_MPC.cpp src/MPC_ROS_Utils.cpp)
#target_link_libraries(acados_mpc
#	${PROJECT_SOURCE_DIR}/scripts/c_generated_code/libacados_ocp_solver_jackal.so
#	${acados_lib}/libacados.so
#	${catkin_LIBRARIES}
#)

add_executable(mpc_acados_node src/mpc_acados_node/mpc_acados_node.cpp)
target_link_libraries(mpc_acados_node
	${PROJECT_SOURCE_DIR}/scripts/c_generated_code/libacados_ocp_solver_jackal.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(test_solver src/mpc_acados_node/test_solver.cpp)
target_link_libraries(test_solver
	${PROJECT_SOURCE_DIR}/scripts/c_generated_code/libacados_ocp_solver_jackal.so
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(trajectory_publisher src/utils/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_publisher src/utils/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gt_publisher src/utils/gt_publisher.cpp)
target_link_libraries(gt_publisher ${catkin_LIBRARIES})
add_dependencies(gt_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(model_state_stamper src/utils/model_state_stamper.cpp)
target_link_libraries(model_state_stamper ${catkin_LIBRARIES})
add_dependencies(model_state_stamper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

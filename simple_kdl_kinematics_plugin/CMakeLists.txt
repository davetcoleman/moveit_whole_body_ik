set(MOVEIT_LIB_NAME moveit_simple_kdl_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} 
  src/simple_kdl_kinematics_plugin.cpp 
  src/kdl/ik_solver_vel_pinv_nso.cpp
  src/kdl/chain.cpp
  src/kdl/frames.cpp
  src/kdl/jacobian.cpp
  src/kdl/jntarray.cpp
  src/kdl/jnt_to_jac_solver.cpp
  src/kdl/utilities/svd_HH.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

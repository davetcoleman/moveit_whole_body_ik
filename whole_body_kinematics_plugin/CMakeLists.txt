set(MOVEIT_LIB_NAME moveit_whole_body_kinematics_plugin)

# SVD Solver
add_library(${MOVEIT_LIB_NAME}_solver
  src/MatrixSolvers.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME}_solver ${catkin_LIBRARIES} ${LAPACK_LIBRARIES})

# Main plugin
add_library(${MOVEIT_LIB_NAME} 
  src/whole_body_kinematics_plugin.cpp 
  src/jacobian_generator.cpp
  src/ik_solver_pinverse.cpp
  # KDL components
  src/kdl/chain.cpp
  src/kdl/frames.cpp
  src/kdl/jacobian.cpp
  src/kdl/jntarray.cpp
  src/kdl/utilities/svd_HH.cpp
)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_solver ${catkin_LIBRARIES})




install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_solver LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

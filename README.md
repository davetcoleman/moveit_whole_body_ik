MoveIt! Whole Body IK
====================

Whole body (non-chain) inverse kinematics solver for MoveIt! for any robot. 
Uses a weighted least-norm solution combined with the gradient projection method of the null space to avoid joint limits. 
Calculates the psuedo inverse using Lapack's singular value decomposition (SVD) implementation. 
Uses some componentes from the Kinematic Dynamics Library (KDL) from OROCOS. 
Build to work with MoveIt!

Developed by [Dave Coleman](dave@dav.ee) while at the University of Tokyo JSK lab in collaboration with Shunichi Nozawa and Kei Okada.

### Limitations

 - Does not yet support mimic joints
 - Does not yet support locking joints externally
 - Does not yet support forward kinematics (use MoveIt's robot state)

# Setup

## kinematics.yaml

Make your ``kinematics.yaml`` look something like this:

```
whole_body_fixed:
  kinematics_solver: whole_body_kinematics_plugin/WholeBodyKinematicsPlugin
  kinematics_solver_ik_links:
    - LARM_LINK6
    - RARM_LINK6
    - LLEG_LINK5
    - RLEG_LINK5
```

### Optional Parameters

```
kinematics_solver_max_solver_iterations:   # iterations for newton raphson method before trying a different seed state
kinematics_solver_timeout: 100             # amount of time allowed before giving up
kinematics_solver_verbose:                 # show lots of console output 
kinematics_solver_debug_mode:              # similar to verbose but only shows matrix-related math debug output
kinematics_solver_visualize_search:        # publish to rviz every step of the solver
kinematics_solver_epsilon:                 # error tolerance between desired pose and current pose for termination condition
kinematics_solver_null_space_epsilon:      # threshold for when ee pose and solved pose are close enough that we should reduce influence of the null space projection
kinematics_solver_null_space_vel_gain:     # k, the amount the null space calculation affects the overall velocity gain
kinematics_solver_ee_pos_vel_limit:        # maximum allowed input positional velocity of the end effector before limiting, in meters
kinematics_solver_ee_rot_vel_limit:        # maximum allowed input rotational velocity of the end effector before limiting, in meters
kinematics_solver_joint_velocity_max_ratio:# the fraction of a joint's total range that it is allowed to move per iteration
```

## SRDF

 - You must define planning groups for each *tip* listed in the ''ik_links'' section, above, that goes from ``BASE_LINK -> TIP_LINK``
 - You must also define planning **subgroups** for any overlapping links. For example:
   - Group 1: ``BASE -> TORSO -> ARM1 -> HAND1`` group (has overlapping links with ARM2)
   - Group 2: ``TORSO -> HAND1`` group (no overlapping links with ARM2)

# Assumptions

Note: this code assumes your jmg (joint model group) is in this order:

   TORSO
   ARM
   ARM
   LEG
   LEG

if its not, im not sure what will happen
   
It also assumes your left & right arms each share ``n`` number of torso joints, and those are the only shared joints on the whole robot
  
Assumes all planning groups have their root at the base_link (root link)


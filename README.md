moveit_whole_body_ik
====================

Non-chain inverse kinematics solver for MoveIt!

# Setup

## kinematics.yaml

Make your ``kinematics.yaml`` look something like this:

```
whole_body_fixed:
  kinematics_solver: whole_body_kinematics_plugin/WholeBodyKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 10
  kinematics_solver_attempts: 1
  kinematics_solver_ik_links:
    - LARM_LINK6
    - RARM_LINK6
    - LLEG_LINK5
    - RLEG_LINK5
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
   
It also assumes your l & r arms each share x number of torso joints, and those are the only shared joints on the whole robot
  
Assumes all planning groups have their root at the base_link (root link)

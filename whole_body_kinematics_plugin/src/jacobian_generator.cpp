// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <moveit/whole_body_kinematics_plugin/jacobian_generator.h>

// ROS
#include <ros/ros.h>

#include <iostream>

namespace whole_body_kinematics_plugin
{

JacobianGenerator::JacobianGenerator(bool verbose)
  : verbose_(verbose)
    //locked_joints_(num_joints,false),
    //nr_of_unlocked_joints_(num_joints),   
{
}

bool JacobianGenerator::initialize(const boost::shared_ptr<urdf::ModelInterface>& urdf_model, const robot_model::RobotModelPtr robot_model, const std::vector<std::string>& tip_frames)
{
  // Convert to KDL Tree
  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Could not initialize tree object");
    return false;
  }

  // Convert to multiple KDL chains
  static const std::string BASE_LINK  = "BODY";
  static const std::string CHEST_LINK  = "CHEST_LINK1";
  
  int number_of_chains = tip_frames.size();

  bool include_torso = true;
  if (include_torso)
  {
    number_of_chains += 2;
  }
  chains_.resize(number_of_chains);

  int chain_id = 0; // track what chain we are on
  for (std::size_t i = 0; i < tip_frames.size(); ++i)
  {
    // One chain per tip
    if (!kdl_tree.getChain(CHEST_LINK, tip_frames[i], chains_[chain_id]))
    {
      ROS_ERROR_STREAM_NAMED("kdl","Could not initialize chain object from " << CHEST_LINK << " to " << tip_frames[i]);
      return false;
    }
    // Debug
    if (verbose_)
      ROS_INFO_STREAM_NAMED("kdl","Created chain object from " << CHEST_LINK << " to " << tip_frames[i] 
        << " with " << chains_[chain_id].getNrOfJoints() << " joints");
    ++chain_id;

    // Check if we also need to include the torso jacobian
    if (!include_torso)
      continue;

    if (!kdl_tree.getChain(BASE_LINK, tip_frames[i], chains_[chain_id]))
    {
      ROS_ERROR_STREAM_NAMED("kdl","Could not initialize chain object from " << BASE_LINK << " to " << tip_frames[i]);
      return false;
    }
    // Debug
    if (verbose_)
      ROS_INFO_STREAM_NAMED("kdl","Created chain object from " << BASE_LINK << " to " << tip_frames[i] 
        << " with " << chains_[chain_id].getNrOfJoints() << " joints");
    ++chain_id;

  }

  // Decide which arm we are processing first
  std::string arm0;
  std::string arm1;
  if (tip_frames[0] == "LARM_LINK6")
  {
    arm0 = "left";
    arm1 = "right";
  }
  else
  {
    arm0 = "right";
    arm1 = "left";
  }
  ROS_WARN_STREAM_NAMED("temp","Arm 0 is " << arm0 << " and arm 1 is " << arm1);

  // TODO: make this not hard-coded
  int size_rows = NUM_DIM_EE;
  int size_arm_cols = 7; // todo not hard code
  int size_torso_cols = 2; // todo not hard code
  if (include_torso)
  {
    // Create a vector of locked joints for a torso
    LockedJoints torso_locked;
    torso_locked.push_back(false); // keep torso unlocked
    torso_locked.push_back(false);
    torso_locked.push_back(true); // lock the joints
    torso_locked.push_back(true);
    torso_locked.push_back(true);
    torso_locked.push_back(true);
    torso_locked.push_back(true);
    torso_locked.push_back(true);
    torso_locked.push_back(true);
    LockedJoints arm_locked(7, false);

    // 0 = arm 1
    jacobian_coords_.push_back( MatrixCoords( 0, size_torso_cols ) ); // top, second
    jacobian_groups_.push_back( robot_model->getJointModelGroup(arm0+"_arm") );
    jacobian_locked_joints_.push_back( arm_locked ); 

    // 0 = arm 1 torso
    jacobian_coords_.push_back( MatrixCoords( 0, 0 ) ); // top, first
    jacobian_groups_.push_back( robot_model->getJointModelGroup(arm0+"_arm_torso") );
    jacobian_locked_joints_.push_back( torso_locked ); 

    // 1 = arm 2
    jacobian_coords_.push_back( MatrixCoords( size_rows, size_torso_cols + size_arm_cols ) ); // bottom, third
    jacobian_groups_.push_back( robot_model->getJointModelGroup(arm1+"_arm") );
    jacobian_locked_joints_.push_back( arm_locked ); 

    // 1 = arm 2 torso
    jacobian_coords_.push_back( MatrixCoords( size_rows, 0 ) ); // bottom, first
    jacobian_groups_.push_back( robot_model->getJointModelGroup(arm1+"_arm_torso") );
    jacobian_locked_joints_.push_back( torso_locked ); 
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("temp","TODO");
    exit(-1);
    return false;
  }

  // Track how many joints are locked for each chain
  num_unlocked_joints_.resize(jacobian_locked_joints_.size());

  // Allocate memory
  for (std::size_t chain_id = 0; chain_id < chains_.size(); ++chain_id)
  {
    num_unlocked_joints_[chain_id] = 0;

    // Decide how big each sub jacobian needs to be by counting how many non-locked joints it has
    for (std::size_t i = 0; i < jacobian_locked_joints_[chain_id].size(); ++i)
    {
      if (jacobian_locked_joints_[chain_id][i] == false) // is not locked
        num_unlocked_joints_[chain_id]++; // increment
    }
    ROS_INFO_STREAM_NAMED("temp","Chain " << chain_id << " has " << num_unlocked_joints_[chain_id] << " unlocked joints");

    // Number that are NOT locked
    int sub_jac_cols = num_unlocked_joints_[chain_id];

    // Allocate sub jacobians
    sub_jacobians_.push_back(KDL::Jacobian2dPtr(new KDL::Jacobian2d(sub_jac_cols, NUM_DIM_EE)));

    // Allocate sub joint arrays
    sub_q_ins.push_back(KDL::JntArrayPtr(new KDL::JntArray(sub_jac_cols)));
  }


  return true;
}

JacobianGenerator::~JacobianGenerator()
{
}

/*
bool JacobianGenerator::setLockedJoints(const std::vector<bool> locked_joints)
{
  if(locked_joints.size()!=locked_joints_.size())
    return false;
  locked_joints_=locked_joints;
  nr_of_unlocked_joints_=0;
  for(unsigned int i=0;i<locked_joints_.size();i++){
    if(!locked_joints_[i])
      nr_of_unlocked_joints_++;
  }

  return true;
}
*/

bool JacobianGenerator::generateJacobian(const robot_state::RobotStatePtr state, KDL::Jacobian2d& jacobian, int seg_nr)
//q_in
{
  if (verbose_)
  {
    std::cout << "\n\n> Starting JntToJac with "<< chains_.size() << " chains --------------------------- " << std::endl;
  }

  // Reset target jacobian
  SetToZero(jacobian); // ORIGINAL

  //std::cout << "Zero jacobian: " << std::endl;
  //jacobian.print();

  int joint_start_index = 0; // track which joint vector we are using

  const KDL::Chain *this_chain;

  if (verbose_)
    std::cout << "\nWe have " << chains_.size() << " chains" << std::endl;

  for (std::size_t chain_id = 0; chain_id < chains_.size(); ++chain_id)
  {
    if (verbose_)
      std::cout << std::endl << "Processing chain " << chain_id << " ----------------------------" << std::endl;

    // Get a pointer to the current chain
    this_chain = &chains_[chain_id];

    SetToZero(*sub_jacobians_[chain_id].get());

    // Create subset q array ---------------
    std::cout << OMPL_CONSOLE_COLOR_CYAN;

    // Get joints in correct ordering from joint model group
    std::vector<double> joints(jacobian_groups_[chain_id]->getVariableCount()); // TODO don't copy to vector
    state->copyJointGroupPositions( jacobian_groups_[chain_id], sub_q_ins[chain_id]->data );

    if (verbose_)
    {
      std::cout << "Sub joints for chain " << chain_id << " is: ";
      for (std::size_t i = 0; i < sub_q_ins[chain_id]->rows(); ++i)
      {
        std::cout << (*sub_q_ins[chain_id])(i) <<  ", ";
      }
      std::cout << std::endl;
    }

    if (verbose_)
      std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
    // ------------------------------

    // Calculate this jacobian
    if (!generateChainJacobian((*sub_q_ins[chain_id]), *sub_jacobians_[chain_id].get(), seg_nr, chain_id))
    {
      std::cout << "Failed to calculate jacobian for chain " << chain_id << std::endl;
      return false;
    }

    // Debug
    if (verbose_)
    {
      std::cout << "Sub jacobian for chain #" << chain_id << ":" << std::endl;
      sub_jacobians_[chain_id]->print();
      std::cout << std::endl;
    }

    if (verbose_)
      std::cout << OMPL_CONSOLE_COLOR_GREEN << "*** Overlaying into main jacobian **** " << std::endl;

    // Top left location to start placing jacobian
    int target_row = jacobian_coords_[chain_id].first;
    int target_col = jacobian_coords_[chain_id].second;

    if (verbose_)
      std::cout << "Placing subjacobian in row " << target_row << " and col " << target_col << " for chain " << chain_id << std::endl;

    // Overlay into main jacobian
    for (std::size_t i = 0; i < sub_jacobians_[chain_id]->rows(); ++i)
    {
      for (std::size_t j = 0; j < sub_jacobians_[chain_id]->columns(); ++j)
      {
        // Copy single value at a time
        jacobian(target_row + i, target_col + j) = (*sub_jacobians_[chain_id])(i,j);
        //std::cout << "  Location " << jac_output_row + i << " x " << jac_output_col + j
        //          << " equals " << jacobian(jac_output_row + i, jac_output_col + j) << std::endl;
      }
    }

    // Show status
    if (verbose_)
    {
      std::cout << "Modified Main jacobian: " << std::endl;
      jacobian.print();
      std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
      //std::cout << "about to loop chain id: " << chain_id << " chain size: " << chains.size() << std::endl;
    }
  }

  if (verbose_)
    std::cout << "FINISHED CALCULTING JACOBIAN -------------------------- " << std::endl << std::endl;

  return true;
}

bool JacobianGenerator::generateChainJacobian(const KDL::JntArray& q_in, KDL::Jacobian2d& jacobian, const int seg_nr, const int chain_id)
{
  if (verbose_)
    std::cout << "generateChainJacobian for chain # "  << chain_id << std::endl;

  // Optionally do not proccess whole chain
  if(seg_nr<0) // default value
    segment_nr_ = chains_[chain_id].getNrOfSegments(); // process the entire chain
  else
    segment_nr_ = seg_nr; // stop early

  //Initialize Jacobian2d to zero since only segment_nr_ colunns are computed
  SetToZero(jacobian) ;

  // Error check
  if (q_in.rows() != chains_[chain_id].getNrOfJoints())
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Number of rows " << q_in.rows() << " does not equal number of joints in chain "
      << chains_[chain_id].getNrOfJoints());
    return false;
  }
  else if (num_unlocked_joints_[chain_id] != jacobian.columns())
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Number of unlocked joints " << num_unlocked_joints_[chain_id] << " does not equal number of columns "
      << jacobian.columns() << " and rows is " << jacobian.rows());
    return false;
  }
  else if (segment_nr_ > chains_[chain_id].getNrOfSegments())
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Segment number is greater than the number of segments " << segment_nr_);
    return false;
  }

  // Reset the frame
  T_frame_tmp_ = KDL::Frame::Identity();

  // Reset the twist
  SetToZero(t_twist_tmp_);

  int j=0; // tracks which input joint value q(j) we are using
  int k=0; // tracks which column in jacobian we are editing

  // Loop through every segment in chain (until the stop index)
  for (unsigned int joint_id=0; joint_id < segment_nr_; joint_id++)
  {
    //Calculate new Frame_base_ee
    if (chains_[chain_id].getSegment(joint_id).getJoint().getType() != KDL::Joint::None) // is a regular joint
    {
      if (verbose_)
        std::cout << "Chain " << chain_id << " joint " << joint_id  << " is a regular joint" <<  std::endl;

      //pose of the new end-point expressed in the base
      total_frame_ = T_frame_tmp_ * chains_[chain_id].getSegment(joint_id).pose(q_in(j));

      //changing base of new segment's twist to base frame if it is not locked
      if(!jacobian_locked_joints_[chain_id][j])
      {
        if (verbose_)
          std::cout << " --> is NOT locked " << std::endl;

        t_twist_tmp_ = T_frame_tmp_.M * chains_[chain_id].getSegment(joint_id).twist( q_in(j), 1.0 );
      }
    }
    else
    {
      if (verbose_)
        std::cout << "Chain " << chain_id << " joint " << joint_id  << " is a fixed joint" <<  std::endl;

      // Is a fixed joint, so skip it (just calculate transpose)
      total_frame_ = T_frame_tmp_ * chains_[chain_id].getSegment(joint_id).pose(0.0);
    }

    //Changing Refpoint of all columns to new ee
    changeRefPoint(jacobian, total_frame_.p - T_frame_tmp_.p, jacobian);

    //Only increase jointnr if the segment has a joint
    if(chains_[chain_id].getSegment(joint_id).getJoint().getType() != KDL::Joint::None)
    {
      //Only put the twist inside if it is not locked
      if(!jacobian_locked_joints_[chain_id][j])
      {
        if (verbose_)
          std::cout << " --> is NOT locked " << std::endl;
        jacobian.setColumn(k++, t_twist_tmp_);
      }
      j++;
    }

    T_frame_tmp_ = total_frame_;
  }

  return true;
}
} // namespace


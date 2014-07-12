/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Creates a jacobian for non-chain robot kinematics
*/


#include <moveit/whole_body_kinematics_plugin/jacobian_generator.h>

// ROS
#include <ros/ros.h>

#include <iostream>

namespace whole_body_kinematics_plugin
{

JacobianGenerator::JacobianGenerator(bool verbose)
  : verbose_(verbose)
{
}

JacobianGenerator::~JacobianGenerator()
{
}

bool JacobianGenerator::initialize(const boost::shared_ptr<urdf::ModelInterface>& urdf_model, const robot_model::RobotModelPtr robot_model,
  const std::vector<std::string>& tip_frames, const robot_model::JointModelGroup *jmg)
{
  // Note: this code assumes your jmg (joint model group) is in this order:
  // TORSO
  // ARM
  // ARM
  // LEG
  // LEG
  // if its not, im not sure what will happen
  //
  // It also assumes your l & r arms each share x number of torso joints,
  // and those are the only shared joints on the whole robot
  //
  // Assumes all planning groups have their root at the base_link (root link)

  // Convert to KDL Tree
  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Could not initialize tree object");
    return false;
  }

  int expected_dimensions = 0; // book keeping for error checking

  // Check if this is a chain or a tree and split modes
  if (jmg->isChain())
  {
    // This is the easy version

    // HACK TO COMPARE WITH JSK EUSLISP VERSION
    if (false)
    {
      const robot_model::JointModelGroup *torso_and_arm = robot_model->getJointModelGroup("left_arm_torso");

      // Create the pure chain group
      IKChainGroup ik_group(torso_and_arm);

      // Set the number of unlocked joints
      ik_group.num_unlocked_joints_ = 7;

      // Choose where the coordinates will go
      ik_group.jacobian_coords_.first = 0;
      ik_group.jacobian_coords_.second = 0;

      // Lock the torso
      ik_group.locked_joints_[0] = true;
      ik_group.locked_joints_[1] = true;

      // This new_group is a pure, regular kinematic chain (no shared joints)
      ROS_DEBUG_STREAM_NAMED("jacobian_generator","Adding jacobian subgroup " << ik_group.jmg_->getName());
      chains_.push_back( ik_group );
    }
    else
    {
      // Add chains that do not share any common links (i.e. legs)
      int temp1 = 0, temp2 = 0;
      addChain(jmg, temp1, temp2);
    }

    // Manually set this value, its real purpose is for non-chain kinematic structures
    expected_dimensions = jmg->getActiveJointModels().size();
  }
  else
  {
    matchTipsToSubgroups(robot_model, tip_frames, jmg, expected_dimensions);
  }

  // Debug
  if (verbose_ || true)
  {
    std::cout << OMPL_CONSOLE_COLOR_CYAN << std::endl  << "Overview of created IK chain groups: " << std::endl;
    for (std::size_t i = 0; i < chains_.size(); ++i)
    {
      std::cout << "Chain " << i << ": " << chains_[i].jmg_->getName() << std::endl;
    }
    std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
  }

  // Error check that our joint model group has the same number of active joints as we expect
  if (jmg->getActiveJointModels().size() != expected_dimensions)
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator","The main joint model group does not have the same number of active joints as our sub groups in total.");
    ROS_ERROR_STREAM_NAMED("jacobian_generator","Main JM joints: " << jmg->getActiveJointModels().size() << " Expected: " << expected_dimensions);
    return false;
  }
  else if( verbose_ )
    ROS_DEBUG_STREAM_NAMED("jacobian_generator","Main JM joints: " << jmg->getActiveJointModels().size() << " Expected: " << expected_dimensions);

  // Fill in rest of info ---------------------------------------------------------------------

  // Convert to multiple KDL chains
  for (std::size_t chain_id = 0; chain_id < chains_.size(); ++chain_id)
  {
    IKChainGroup *group = &chains_[chain_id];
    const robot_model::LinkModel *from_link = group->jmg_->getJointModels()[0]->getParentLinkModel();
    const robot_model::LinkModel *to_link = group->jmg_->getLinkModels().back();

    // Load chain kinematic structure
    if (!kdl_tree.getChain(from_link->getName(), to_link->getName(), group->kdl_chain_))
    {
      ROS_ERROR_STREAM_NAMED("whole_body_ik","Could not initialize chain object from " << from_link->getName() << " to " << to_link->getName());
      return false;
    }
    if (verbose_)
      ROS_DEBUG_STREAM_NAMED("whole_body_ik","Created chain object from " << from_link->getName() << " to " << to_link->getName());

    // Allocate sub jacobians
    group->sub_jacobian_ = KDL::Jacobian2dPtr(new KDL::Jacobian2d(group->num_unlocked_joints_, NUM_DIM_EE));

    // Allocate sub joint arrays
    group->sub_q_in_ = KDL::JntArrayPtr(new KDL::JntArray(group->num_unlocked_joints_));
  }

  return true;
}

bool JacobianGenerator::matchTipsToSubgroups(const robot_model::RobotModelPtr robot_model, const std::vector<std::string>& tip_frames,
  const robot_model::JointModelGroup *jmg, int &expected_dimensions)
{

  // Get base link of robot
  const robot_model::LinkModel *base_link = robot_model->getRootLink();

  // Get containing subgroups
  std::vector<const robot_model::JointModelGroup*> subgroups;
  jmg->getSubgroups(subgroups);

  if (verbose_)
  {
    for (std::size_t i = 0 ; i < subgroups.size() ; ++i)
    {
      std::cout << "Subgroup found: " << subgroups[i]->getName() << "\n  from: "
                << subgroups[i]->getLinkModelNames().front() << "\n  to: " <<
        subgroups[i]->getLinkModelNames().back() << "\n  is chain: " << subgroups[i]->isChain() << "\n  base: " <<
        subgroups[i]->getCommonRoot()->getParentLinkModel()->getName();
    }
  }

  std::vector<const robot_model::JointModelGroup*> tip_to_jmg(tip_frames.size());

  // Find a MoveIt! planning group for each tip frame
  for (std::size_t tip_id = 0; tip_id < tip_frames.size(); ++tip_id)
  {
    if (verbose_)
      std::cout << "Looking for planning group for tip frame: " << tip_id << " named " << tip_frames[tip_id] << std::endl;

    const robot_model::LinkModel *tip_link = robot_model->getLinkModel(tip_frames[tip_id]);

    // Check that a jmg was found to match this tip
    if (!linksToJointGroup(subgroups, base_link, tip_link, tip_to_jmg[tip_id] ))
    {
      // Check if the
      ROS_ERROR_STREAM_NAMED("jacobian_generator","Unable to find joint model group corresponding to tip " << tip_frames[tip_id]);
      return false;
    }
    else
    {
      if (verbose_)
        std::cout << " --> Found joint model group: " << tip_to_jmg[tip_id]->getName()
                  << " of size " << tip_to_jmg[tip_id]->getActiveJointModels().size() << std::endl;
    }

    // Make sure at the end we have the expected number of joints
    expected_dimensions += tip_to_jmg[tip_id]->getActiveJointModels().size();

  } // for each tip

  if (verbose_)
    std::cout << std::endl << "DONE MATCHING, now find common joints --------------------------" << std::endl;

  // Use this to place the sub jacobians
  int full_jac_row_location = 0;
  int full_jac_col_location = 0;

  // Track which tips we have added to our final structure
  std::vector<bool> processed_tips(tip_frames.size(), false);

  // Check for common links in each found joint model group, such as torsos
  for (std::size_t tip_id = 0; tip_id < tip_frames.size(); ++tip_id)
  {
    const robot_model::JointModelGroup *this_group = tip_to_jmg[tip_id];

    if (verbose_)
      ROS_INFO_STREAM_NAMED("jacobian_generator","Tip " << tip_frames[tip_id] << " has group " << this_group->getName());

    // Compare to all the other groups to see if any overlaps exist
    for (std::size_t tip2_id = tip_id + 1; tip2_id < tip_frames.size(); ++tip2_id)
    {
      const robot_model::JointModelGroup *that_group = tip_to_jmg[tip2_id];

      // Compare each joint, starting from base
      bool shares_joints = false;
      std::size_t joint_id;
      for (joint_id = 0; joint_id < std::min(this_group->getJointModels().size(),that_group->getJointModels().size()); ++joint_id)
      {
        std::cout << "Joint of group " << this_group->getName() << " id " << joint_id
                  << " is " << this_group->getJointModels()[joint_id]->getName() << std::endl;
        std::cout << "   Compared to " << that_group->getName() << " id " << joint_id
                  << " is " << that_group->getJointModels()[joint_id]->getName() << std::endl;

        // Note: the we are moving backwards from base, so we can terminate as soon as a non-match is found
        if ( this_group->getJointModels()[joint_id] != that_group->getJointModels()[joint_id] )
        {
          // Not matching break;
          break;
        }
        shares_joints = true;
      }

      // Create new kinematic chains for these subgroups
      if (shares_joints)
      {
        std::cout << "Groups " << this_group->getName() << " and " << that_group->getName() << " have common joints" << std::endl;
        std::cout << " --> They share joints up to joint " << joint_id << std::endl;

        if (processed_tips[tip_id] || processed_tips[tip2_id])
        {
          ROS_ERROR_STREAM_NAMED("jacobian_generator","These tips have already been processed, unknown error.");
          return false;
        }

        // Create the overlapping torso groups
        int num_shared_joints = joint_id;
        expected_dimensions -= num_shared_joints; // book keeping

        // ASSUMPTION: shared joints are first in the input overall joint model group.
        // Check this and throw error if not (TODO?)
        for (std::size_t shared_id = 0; shared_id < num_shared_joints; ++shared_id)
        {
          if (jmg->getJointModels()[shared_id] != this_group->getJointModels()[shared_id])
          {
            ROS_ERROR_STREAM_NAMED("jacobian_generator","An implementation assumption that the troso joints are first in the planning group has failed");
            return false;
          }
        }

        // Add both 'this' and 'that' planning groups that have shared joints (not pure ik chains)
        for (std::size_t group_id = 0; group_id < 2; ++group_id)
        {
          // Choose which group of the two
          const robot_model::JointModelGroup *current_group = group_id == 0 ? this_group : that_group;

          // Create new group
          IKChainGroup ik_group(current_group);

          // Lock joints after joint_id
          lockJointsAfter(ik_group, joint_id - 1);

          // Set the number of unlocked joints
          ik_group.num_unlocked_joints_ = num_shared_joints;

          // Choose where the coordinates will go
          ik_group.jacobian_coords_.first = full_jac_row_location;
          ik_group.jacobian_coords_.second = full_jac_col_location;

          // Because we know there are two shared groups, we will move the next jacobian under it
          if (group_id == 0) // TODO: make this much more durable for non-traditional humanoid cases
          {
            full_jac_row_location += NUM_DIM_EE;
          }
          else // we are finished with the shared joints, move back up and to the right
          {
            full_jac_row_location = 0;
            full_jac_col_location = num_shared_joints;
          }

          if (verbose_)
          {
            std::cout << "Group " << current_group->getName() << " has locks on: " << std::endl;
            std::copy(ik_group.locked_joints_.begin(), ik_group.locked_joints_.end(), std::ostream_iterator<bool>(std::cout, "\n"));
            std::cout << std::endl;
          }

          // Save this new ik group
          if (verbose_)
            ROS_DEBUG_STREAM_NAMED("jacobian_generator","Adding jacobian subgroup " << ik_group.jmg_->getName());
          chains_.push_back( ik_group );
        }
        // Set as saved
        processed_tips[tip_id] = true;
        processed_tips[tip2_id] = true;

        // Find a planning group for each of them that only includes non-shared joints
        // i.e. this will create left_arm and right_arm, not including the torso
        for (std::size_t group_id = 0; group_id < 2; ++group_id)
        {
          // Choose which group of the two
          const robot_model::JointModelGroup *outer_group = group_id == 0 ? this_group : that_group;

          // Get its new base
          const robot_model::LinkModel *new_base = outer_group->getJointModels()[joint_id]->getParentLinkModel();

          // Find the joint model group that has the same base->tip links
          const robot_model::JointModelGroup* new_group;
          if (!linksToJointGroup(subgroups, new_base, outer_group->getLinkModels().back(), new_group))
          {
            ROS_ERROR_STREAM_NAMED("jacobian_generator","Unable to find planning group from links " << new_base->getName()
              << " to " << outer_group->getLinkModels().back()->getName());
            return false;
          }

          std::cout << "Adding pure chain joint model group " << new_group->getName() << std::endl;

          // Create the pure chain group
          IKChainGroup ik_group(new_group);
          // All joints should be *unlocked*, which is true by default
          // Number of unlocked joints is set by default

          // Choose where the coordinates will go
          ik_group.jacobian_coords_.first = full_jac_row_location;
          ik_group.jacobian_coords_.second = full_jac_col_location;

          // Move to next location
          full_jac_row_location += NUM_DIM_EE;
          full_jac_col_location += new_group->getJointModels().size();

          // This new_group is a pure, regular kinematic chain (no shared joints)
          ROS_DEBUG_STREAM_NAMED("jacobian_generator","Adding jacobian subgroup " << ik_group.jmg_->getName());
          chains_.push_back( ik_group );

        }
      }
    }
  }

  // Add any chains that do not share any common links (i.e. legs)
  for (std::size_t tip_id = 0; tip_id < processed_tips.size(); ++tip_id)
  {
    // Skip the tips we've already processed
    if (processed_tips[tip_id])
      continue;

    ROS_INFO_STREAM_NAMED("jacobian_generator","Adding tip " << tip_id << " as regular chain (no shared joints)");

    addChain( tip_to_jmg[tip_id], full_jac_row_location, full_jac_col_location );
  }

}

void JacobianGenerator::addChain(const robot_model::JointModelGroup *current_group, int &full_jac_row_location, int &full_jac_col_location)
{
  // Create the pure chain group
  IKChainGroup ik_group(current_group);
  // All joints should be *unlocked*, which is true by default
  // Number of unlocked joints is set by default

  // Choose where the coordinates will go
  ik_group.jacobian_coords_.first = full_jac_row_location;
  ik_group.jacobian_coords_.second = full_jac_col_location;

  // Move to next location
  full_jac_row_location += NUM_DIM_EE;
  full_jac_col_location += current_group->getJointModels().size();

  // This new_group is a pure, regular kinematic chain (no shared joints)
  ROS_DEBUG_STREAM_NAMED("jacobian_generator","Adding jacobian subgroup " << ik_group.jmg_->getName());
  chains_.push_back( ik_group );
}

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
      std::cout << std::endl << "Processing chain " << chain_id << " - " << chains_[chain_id].jmg_->getName() << " ----------------------" << std::endl;

    // Get a pointer to the current chain
    this_chain = &chains_[chain_id].kdl_chain_;

    SetToZero(*chains_[chain_id].sub_jacobian_.get());

    // Create subset q array ---------------
    // Get joints in correct ordering from joint model group
    std::vector<double> joints(chains_[chain_id].jmg_->getVariableCount()); // TODO don't copy to vector
    state->copyJointGroupPositions( chains_[chain_id].jmg_, chains_[chain_id].sub_q_in_->data );

    if (verbose_)
    {
      std::cout << OMPL_CONSOLE_COLOR_CYAN;
      std::cout << "Sub joints for chain " << chain_id << " is: ";
      for (std::size_t i = 0; i < chains_[chain_id].sub_q_in_->rows(); ++i)
      {
        std::cout << (*chains_[chain_id].sub_q_in_)(i) <<  ", ";
      }
      std::cout << std::endl;
    }

    if (verbose_)
      std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
    // ------------------------------

    // Calculate this jacobian
    if (!generateChainJacobian((*chains_[chain_id].sub_q_in_), *chains_[chain_id].sub_jacobian_.get(), seg_nr, chain_id))
    {
      std::cout << "Failed to calculate jacobian for chain " << chain_id << std::endl;
      return false;
    }

    // Debug
    if (verbose_)
    {
      std::cout << "Sub jacobian for chain #" << chain_id << ":" << std::endl;
      chains_[chain_id].sub_jacobian_->print();
      std::cout << std::endl;
    }

    if (verbose_)
      std::cout << OMPL_CONSOLE_COLOR_GREEN << "*** Overlaying into main jacobian **** " << std::endl;

    // Top left location to start placing jacobian
    int target_row = chains_[chain_id].jacobian_coords_.first;
    int target_col = chains_[chain_id].jacobian_coords_.second;

    if (verbose_)
      std::cout << "Placing subjacobian in row " << target_row << " and col " << target_col << " for chain " << chain_id << std::endl;

    // Overlay into main jacobian
    for (std::size_t i = 0; i < chains_[chain_id].sub_jacobian_->rows(); ++i)
    {
      for (std::size_t j = 0; j < chains_[chain_id].sub_jacobian_->columns(); ++j)
      {
        // Copy single value at a time
        jacobian(target_row + i, target_col + j) = (*chains_[chain_id].sub_jacobian_)(i,j);
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
    segment_nr_ = chains_[chain_id].kdl_chain_.getNrOfSegments(); // process the entire chain
  else
    segment_nr_ = seg_nr; // stop early

  //Initialize Jacobian2d to zero since only segment_nr_ colunns are computed
  SetToZero(jacobian) ;

  // Error check
  if (q_in.rows() != chains_[chain_id].kdl_chain_.getNrOfJoints())
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Number of rows " << q_in.rows() << " does not equal number of joints in chain "
      << chains_[chain_id].kdl_chain_.getNrOfJoints());
    return false;
  }
  else if (chains_[chain_id].num_unlocked_joints_ != jacobian.columns())
  {
    ROS_ERROR_STREAM_NAMED("jacobian_generator", "Number of unlocked joints " << chains_[chain_id].num_unlocked_joints_
      << " does not equal number of columns " << jacobian.columns() << " and rows is " << jacobian.rows());
    return false;
  }
  else if (segment_nr_ > chains_[chain_id].kdl_chain_.getNrOfSegments())
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
    if (chains_[chain_id].kdl_chain_.getSegment(joint_id).getJoint().getType() != KDL::Joint::None) // is a regular joint
    {
      if (verbose_)
        std::cout << "Chain " << chain_id << " joint " << joint_id  << " is a regular joint" <<  std::endl;

      //pose of the new end-point expressed in the base
      total_frame_ = T_frame_tmp_ * chains_[chain_id].kdl_chain_.getSegment(joint_id).pose(q_in(j));

      //changing base of new segment's twist to base frame if it is not locked
      if(!chains_[chain_id].locked_joints_[j])
      {
        t_twist_tmp_ = T_frame_tmp_.M * chains_[chain_id].kdl_chain_.getSegment(joint_id).twist( q_in(j), 1.0 );
      }
    }
    else
    {
      if (verbose_)
        std::cout << "Chain " << chain_id << " joint " << joint_id  << " is a fixed joint" <<  std::endl;

      // Is a fixed joint, so skip it (just calculate transpose)
      total_frame_ = T_frame_tmp_ * chains_[chain_id].kdl_chain_.getSegment(joint_id).pose(0.0);
    }

    //Changing Refpoint of all columns to new ee
    changeRefPoint(jacobian, total_frame_.p - T_frame_tmp_.p, jacobian);

    //Only increase jointnr if the segment has a joint
    if(chains_[chain_id].kdl_chain_.getSegment(joint_id).getJoint().getType() != KDL::Joint::None)
    {
      //Only put the twist inside if it is not locked
      if(!chains_[chain_id].locked_joints_[j])
      {
        jacobian.setColumn(k++, t_twist_tmp_);
      }
      else if (verbose_)
        std::cout << " --> is locked " << std::endl;

      j++;
    }

    T_frame_tmp_ = total_frame_;
  }

  return true;
}

bool JacobianGenerator::linksToJointGroup( std::vector<const robot_model::JointModelGroup*> subgroups,
  const robot_model::LinkModel* from, const robot_model::LinkModel* to, const robot_model::JointModelGroup* &found_group)
{
  if (verbose_)
    ROS_INFO_STREAM_NAMED("temp","linksToJointGroup from: " << from->getName() << " to: " << to->getName());

  for (std::size_t subgroup_id = 0; subgroup_id < subgroups.size(); ++subgroup_id)
  {
    const robot_model::JointModelGroup *subgroup = subgroups[subgroup_id];

    // Must be chain to be used with a tip frame
    if (!subgroup->isChain())
      continue;

    // Check first link
    if (subgroup->getCommonRoot()->getParentLinkModel() == from &&
      subgroup->getLinkModels().back() == to)
    {
      found_group = subgroup;
      return true;
    }


    if (verbose_)
    {
      std::cout << " -- DID NOT FIND joint model group that has same first and last tips: on group " <<
        subgroup->getName() <<
        " from: " <<
        subgroup->getCommonRoot()->getParentLinkModel()->getName() <<
        " to " << subgroup->getLinkModelNames().back() << std::endl;
    }

  }
  return false;
}

void JacobianGenerator::lockJointsAfter(IKChainGroup &ik_group, int lock_after_id)
{
  // -1 means everything is unlocked
  if (lock_after_id == -1)
    return;

  for (std::size_t i = lock_after_id + 1; i < ik_group.jmg_->getJointModels().size(); ++i)
  {
    ik_group.locked_joints_[i] = true;
  }
}


} // namespace


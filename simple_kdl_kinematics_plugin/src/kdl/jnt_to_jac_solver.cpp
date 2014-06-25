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

#include <moveit/simple_kdl_kinematics_plugin/kdl/jnt_to_jac_solver.hpp>

#include <iostream> // TODO remove

namespace KDL
{
JntToJacSolver::JntToJacSolver(const std::vector<Chain>& _chains, int num_joints):
  chains(_chains),locked_joints_(num_joints,false),
  nr_of_unlocked_joints_(num_joints)
{
}

JntToJacSolver::~JntToJacSolver()
{
}

void JntToJacSolver::printJacobian(const Jacobian &jac)
{
  std::cout << "--------------------------- " << std::endl;
  for (std::size_t i = 0; i < jac.rows(); ++i)
  {
    for (std::size_t j = 0; j < jac.columns(); ++j)
    {
      std::cout << jac(i,j) << ", ";
    }
    std::cout << std::endl;
  }
}

int JntToJacSolver::setLockedJoints(const std::vector<bool> locked_joints)
{
  if(locked_joints.size()!=locked_joints_.size())
    return -1;
  locked_joints_=locked_joints;
  nr_of_unlocked_joints_=0;
  for(unsigned int i=0;i<locked_joints_.size();i++){
    if(!locked_joints_[i])
      nr_of_unlocked_joints_++;
  }

  return 0;
}

int JntToJacSolver::JntToJac(const JntArray& q_in, Jacobian& jac, int seg_nr)
{
  const Chain *this_chain;

  std::cout << "Starting jnt to jac " << std::endl;
  // Reset target jacobian
  SetToZero(jac);
  printJacobian(jac);

  // Store all the jacobian components
  std::vector<Jacobian> jacs(chains.size());

  int joint_start_index = 0; // track which joint vector we are using
  int jac_output_row = 0; // track where to place the output jacobian
  int jac_output_col = 0; // track where to place the output jacobian

  for (std::size_t chain_id = 0; chain_id < chains.size(); ++chain_id)
  {
    // Get a pointer to the current chain
    this_chain = &chains[chain_id];

    // Get subcomponent of q_in via copy
    int length_q = this_chain->getNrOfJoints();
    JntArray this_q_in(length_q);
    for (std::size_t j = joint_start_index; j < joint_start_index + length_q; ++j)
    {
      this_q_in(j) = q_in(joint_start_index + j);
    }
    joint_start_index += length_q; // move forward

    // Calculate this jacobian
    JntToJacSingle(this_q_in, jacs[chain_id], seg_nr, chain_id); // TODO check for failure

    // Debug
    printJacobian(jacs[chain_id]);

    // Overlay into main jacobina
    for (std::size_t i = 0; i < jacs[chain_id].rows(); ++i)
    {
      for (std::size_t j = 0; j < jacs[chain_id].columns(); ++j)
      {
        // Copy single value at a time
        jac(jac_output_row + i, jac_output_col + j) = jacs[i](i,j);
      }
    }
    // Assume we are just putting them down diagonally for now
    jac_output_row += jacs[chain_id].rows();
    jac_output_col += jacs[chain_id].columns();
  }
  std::cout << "DONE, result: " << std::endl;
  printJacobian(jac);
}

int JntToJacSolver::JntToJacSingle(const JntArray& q_in, Jacobian& jac, int seg_nr, int chain_id)
{
  // Optionally do not proccess whole chain
  unsigned int segmentNr;
  if(seg_nr<0) // default value
    segmentNr = chains[chain_id].getNrOfSegments(); // process the entire chain
  else
    segmentNr = seg_nr; // stop early

  //Initialize Jacobian to zero since only segmentNr colunns are computed
  SetToZero(jac) ;

  // Error check
  if (q_in.rows() != chains[chain_id].getNrOfJoints() || nr_of_unlocked_joints_ != jac.columns())
    return -1;
  else if (segmentNr > chains[chain_id].getNrOfSegments())
    return -1;

  // Reset the frame
  T_frame_tmp = Frame::Identity();

  // Reset the twist
  SetToZero(t_twist_tmp);

  int j=0; // tracks which input joint value q(j) we are using
  int k=0; // tracks which column in jacobian we are editing

  // Loop through every segment in chain (until the stop index)
  for (unsigned int i=0; i < segmentNr; i++)
  {
    //Calculate new Frame_base_ee
    if (chains[chain_id].getSegment(i).getJoint().getType() != Joint::None) // is a regular joint
    {
      //pose of the new end-point expressed in the base
      total_frame = T_frame_tmp * chains[chain_id].getSegment(i).pose(q_in(j));

      //changing base of new segment's twist to base frame if it is not locked
      if(!locked_joints_[j])
      {
        t_twist_tmp = T_frame_tmp.M * chains[chain_id].getSegment(i).twist( q_in(j), 1.0 );
      }
    }
    else
    {
      // Is a fixed joint, so skip it (just calculate transpose)
      total_frame = T_frame_tmp * chains[chain_id].getSegment(i).pose(0.0);
    }

    //Changing Refpoint of all columns to new ee
    changeRefPoint(jac, total_frame.p - T_frame_tmp.p, jac);

    //Only increase jointnr if the segment has a joint
    if(chains[chain_id].getSegment(i).getJoint().getType() != Joint::None)
    {
      //Only put the twist inside if it is not locked
      if(!locked_joints_[j])
        jac.setColumn(k++, t_twist_tmp);
      j++;
    }

    T_frame_tmp = total_frame;
  }

  return 0;
}
} // namespace


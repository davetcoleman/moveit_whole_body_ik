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
JntToJacSolver::JntToJacSolver(const std::vector<Chain>& _chains, int _num_joints, bool _verbose):
  chains(_chains),locked_joints_(_num_joints,false),
  nr_of_unlocked_joints_(_num_joints),
  verbose(_verbose)
{
}

JntToJacSolver::~JntToJacSolver()
{
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

int JntToJacSolver::JntToJac(const JntArray& q_in, Jacobian2d& jac, int seg_nr)
{
  if (verbose)
  {
    std::cout << "\n\n> Starting JntToJac with "<< chains.size() << " chains and input joint array of size "
              << q_in.rows() << " rows by " << q_in.columns() << " columns ------------------------ " << std::endl;
  }

  // Reset target jacobian
  SetToZero(jac); // ORIGINAL

  //std::cout << "Zero jacobian: " << std::endl;
  //jac.print();

  int joint_start_index = 0; // track which joint vector we are using
  int jac_output_row = 0; // track where to place the output jacobian
  int jac_output_col = 0; // track where to place the output jacobian

  const Chain *this_chain;

  if (verbose)
    std::cout << "\nWe have " << chains.size() << " chains" << std::endl;

  for (std::size_t chain_id = 0; chain_id < chains.size(); ++chain_id)
  {
    if (verbose)
      std::cout << std::endl << "Processing chain " << chain_id << " ----------------------------" << std::endl;

    // Get a pointer to the current chain
    this_chain = &chains[chain_id];

    // Create a sub-jacobian for this chain
    // 6 (pose) x n (number of joints)
    Jacobian2d sub_jacobian(this_chain->getNrOfJoints(), 6);
    SetToZero(sub_jacobian);

    // Get subcomponent of q_in via copy
    int length_q = this_chain->getNrOfJoints();
    JntArray this_q_in(length_q);

    // Create q array ---------------
    if (verbose)
      std::cout << OMPL_CONSOLE_COLOR_CYAN << "\nCreating jntArry of length " << length_q <<
        " starting at index " << joint_start_index << std::endl;

    for (std::size_t j = joint_start_index; j < joint_start_index + length_q; ++j)
    {
      if (verbose)
        std::cout << "   Value " << j << ": ";

      // this_q_in starts from zero, while q_in starts from 'joint_start_index'
      this_q_in(j - joint_start_index) = q_in(j);

      if (verbose)
        std::cout << this_q_in(j) << std::endl;
    }
    if (verbose)
      std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
    // ------------------------------

    joint_start_index += length_q; // move forward

    // Calculate this jacobian
    if (JntToJacSingle(this_q_in, sub_jacobian, seg_nr, chain_id) == -1)
    {
      std::cout << "Failed to calculate jacobian for chain " << chain_id << std::endl;
      exit(-1);
    }

    // Debug
    if (verbose)
    {
      std::cout << "Sub jacobian for chain #" << chain_id << ":" << std::endl;
      sub_jacobian.print();
      std::cout << std::endl;
    }

    if (verbose)
      std::cout << OMPL_CONSOLE_COLOR_GREEN << "*** Overlaying into main jacobian **** " << std::endl;

    // Overlay into main jacobian
    for (std::size_t i = 0; i < sub_jacobian.rows(); ++i)
    {
      for (std::size_t j = 0; j < sub_jacobian.columns(); ++j)
      {
        // Copy single value at a time
        jac(jac_output_row + i, jac_output_col + j) = sub_jacobian(i,j);
        //std::cout << "  Location " << jac_output_row + i << " x " << jac_output_col + j
        //          << " equals " << jac(jac_output_row + i, jac_output_col + j) << std::endl;
      }
    }
    // Assume we are just putting them down diagonally for now
    jac_output_row += sub_jacobian.rows();
    jac_output_col += sub_jacobian.columns();

    // Show status
    if (verbose)
    {
      std::cout << "Modified Main jacobian: " << std::endl;
      jac.print();
      std::cout << OMPL_CONSOLE_COLOR_RESET << std::endl;
      //std::cout << "about to loop chain id: " << chain_id << " chain size: " << chains.size() << std::endl;
    }
  }

  if (verbose)
    std::cout << "FINISHED CALCULTING JACOBIAN -------------------------- " << std::endl << std::endl;
}

int JntToJacSolver::JntToJacSingle(const JntArray& q_in, Jacobian2d& jac, const int seg_nr, const int chain_id)
{
  if (verbose)
    std::cout << "JntToJacSingle for chain # "  << chain_id << std::endl;

  // Optionally do not proccess whole chain
  unsigned int segmentNr;
  if(seg_nr<0) // default value
    segmentNr = chains[chain_id].getNrOfSegments(); // process the entire chain
  else
    segmentNr = seg_nr; // stop early

  //Initialize Jacobian2d to zero since only segmentNr colunns are computed
  SetToZero(jac) ;

  // Error check
  if (q_in.rows() != chains[chain_id].getNrOfJoints())
  {
    std::cout << "Number of rows " << q_in.rows() << " does not equal number of joints in chain "
              << chains[chain_id].getNrOfJoints() << std::endl;
    return -1;
  }
  /*
    TODO address locked joints
  else if (nr_of_unlocked_joints_ != jac.columns())
  {
    std::cout << "Number of unlocked joints " << nr_of_unlocked_joints_ << " does not equal number of columns "
              << jac.columns() << std::endl;
    return -1;
  }
  */
  else if (segmentNr > chains[chain_id].getNrOfSegments())
  {
    std::cout << "Segment number is greater than the number of segments " << segmentNr << std::endl;
    return -1;
  }

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


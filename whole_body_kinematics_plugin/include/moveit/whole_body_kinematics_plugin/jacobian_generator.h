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

#ifndef MOVEIT_WHOLE_BODY_IK__JACOBIAN_GENERATOR_H
#define MOVEIT_WHOLE_BODY_IK__JACOBIAN_GENERATOR_H

// Custom KDL
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"

// Binary KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// System
#include <boost/shared_ptr.hpp>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/robot_model.h>

// Change text color on console output
#define OMPL_CONSOLE_COLOR_RESET "\033[0m"
#define OMPL_CONSOLE_COLOR_GREEN "\033[92m"
#define OMPL_CONSOLE_COLOR_BLUE "\033[94m"
#define OMPL_CONSOLE_COLOR_CYAN "\033[96m"
#define OMPL_CONSOLE_COLOR_BROWN "\033[93m"
#define OMPL_CONSOLE_COLOR_RED "\033[91m"

namespace whole_body_kinematics_plugin
{

typedef std::pair< std::size_t, std::size_t > MatrixCoords; //row, col
typedef std::vector<bool> LockedJoints; // disable joints in a kin chain

static const int NUM_DIM_EE = 6; // end effector pose dimensions

/**
 * @brief  Class to calculate the jacobian of a set of KDL::Chains
 */
class JacobianGenerator
{
public:
  /**
   * \brief Constructor
   */
  JacobianGenerator(bool verbose);

  /**
   * \brief Destructor
   */
  virtual ~JacobianGenerator();

  /**
   * \brief Read a URDF and convert into structures for Jacobian generation
   * \param urdf_model
   * \param tip_frames
   * \return true on success
   */
  bool initialize(const boost::shared_ptr<urdf::ModelInterface>& urdf_model, const robot_model::RobotModelPtr robot_model, 
    const std::vector<std::string>& tip_frames);

  /**
   * Calculate the jacobian expressed in the base frame of the
   * chain, with reference point at the end effector of the
   * *chain. The alghoritm is similar to the one used in
   * KDL::ChainFkSolverVel_recursive
   *
   * @param q_in input joint positions
   * @param jac output jacobian
   *
   * @return always returns 0
   */
  bool generateJacobian(const robot_state::RobotStatePtr state, KDL::Jacobian2d& jacobian, int seg_nr = -1);

  /**
   * \brief Find jacobian for single chain
   */
  bool generateChainJacobian(const KDL::JntArray& q_in, KDL::Jacobian2d& jacobian, const int seg_nr, const int chain_id);

  //  int setLockedJoints(const std::vector<bool> locked_joints);
private:

  KDL::Twist t_twist_tmp_; // base of new segment's twist
  KDL::Frame T_frame_tmp_;
  KDL::Frame total_frame_;

  unsigned int nr_of_unlocked_joints_;
  bool verbose_;

  // All the chains that are combined to make a jacobian
  std::vector<KDL::Chain> chains_;

  // Mapping from subjacobians to their location in the combined jacobian
  std::vector<MatrixCoords> jacobian_coords_;

  // Mapping from subjacobians to their planning groups
  std::vector<robot_model::JointModelGroup*> jacobian_groups_;

  // Track which joints are locked
  std::vector<LockedJoints> jacobian_locked_joints_;

  // Track how many unlocked joints per chain
  std::vector<int> num_unlocked_joints_;

  // Store allocated memory for every chainsub_jacobians
  std::vector<KDL::Jacobian2dPtr> sub_jacobians_;

  // Sub joint index for every chain
  std::vector<KDL::JntArrayPtr> sub_q_ins;

  // Used in generateChainJacobian
  unsigned int segment_nr_;

};

typedef boost::shared_ptr<JacobianGenerator> JacobianGeneratorPtr;

}
#endif


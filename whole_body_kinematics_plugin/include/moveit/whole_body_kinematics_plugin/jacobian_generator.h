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

#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include <boost/shared_ptr.hpp>

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

/**
 * @brief  Class to calculate the jacobian of a set of KDL::Chains
 */
class JacobianGenerator
{
public:
  explicit JacobianGenerator(const std::vector<KDL::Chain>& chains, const std::vector<MatrixCoords>& jacobian_coords, int num_joints, bool verbose);
  virtual ~JacobianGenerator();

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
  virtual int JntToJac(const KDL::JntArray& q_in, KDL::Jacobian2d& jac, int segmentNR=-1);

  /**
   * \brief Find jacobian for single chain
   */
  int JntToJacSingle(const KDL::JntArray& q_in, KDL::Jacobian2d& jac, const int seg_nr, const int chain_id);

  int setLockedJoints(const std::vector<bool> locked_joints);
private:

  KDL::Twist t_twist_tmp; // base of new segment's twist
  KDL::Frame T_frame_tmp;
  KDL::Frame total_frame;

  std::vector<bool> locked_joints_;
  unsigned int nr_of_unlocked_joints_;
  bool verbose;

  // All the chains that are combined to make a jacobian
  const std::vector<KDL::Chain> chains_;

  // Mapping from subjacobians to their location in the combined jacobian
  const std::vector<MatrixCoords> jacobian_coords_;

  // Store allocated memory for every chain
  std::vector<KDL::Jacobian2dPtr> sub_jacobians;

  // Sub joint index for every chain
  std::vector<KDL::JntArrayPtr> sub_q_ins;

  // Used in JntToJacSingle
  unsigned int segmentNr;

};

typedef boost::shared_ptr<JacobianGenerator> JacobianGeneratorPtr;

}
#endif


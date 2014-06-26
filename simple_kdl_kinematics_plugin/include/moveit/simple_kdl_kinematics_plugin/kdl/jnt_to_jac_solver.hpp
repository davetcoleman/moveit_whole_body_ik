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

#ifndef KDL_JNTTOJACSOLVER_HPP
#define KDL_JNTTOJACSOLVER_HPP

#include "frames.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"
#include "chain.hpp"

// Change text color on console output
#define OMPL_CONSOLE_COLOR_RESET "\033[0m"
#define OMPL_CONSOLE_COLOR_GREEN "\033[92m"
#define OMPL_CONSOLE_COLOR_BLUE "\033[94m"
#define OMPL_CONSOLE_COLOR_CYAN "\033[96m"
#define OMPL_CONSOLE_COLOR_BROWN "\033[93m"
#define OMPL_CONSOLE_COLOR_RED "\033[91m"

namespace KDL
{
/**
 * @brief  Class to calculate the jacobian of a general
 * KDL::Chain, it is used by other solvers. It should not be used
 * outside of KDL.
 *
 *
 */

class JntToJacSolver
{
public:
  explicit JntToJacSolver(const std::vector<Chain>& chains, int num_joints);
  virtual ~JntToJacSolver();

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
  virtual int JntToJac(const JntArray& q_in, Jacobian2d& jac, int segmentNR=-1);

  /**
   * \brief Find jacobian for single chain
   */
  int JntToJacSingle(const JntArray& q_in, Jacobian2d& jac, const int seg_nr, const int chain_id);

  int setLockedJoints(const std::vector<bool> locked_joints);
private:
  const std::vector<Chain> chains;

  Twist t_twist_tmp; // base of new segment's twist
  Frame T_frame_tmp;
  Frame total_frame;

  std::vector<bool> locked_joints_;
  unsigned int nr_of_unlocked_joints_;
};
}
#endif


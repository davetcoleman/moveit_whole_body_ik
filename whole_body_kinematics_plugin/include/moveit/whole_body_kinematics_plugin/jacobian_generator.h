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

// Sub components of our overall jacobian
struct IKChainGroup
{
  IKChainGroup (const robot_model::JointModelGroup* jmg)
    : jmg_(jmg),
      locked_joints_(jmg->getJointModels().size(), false),
      num_unlocked_joints_(jmg->getJointModels().size())
  {}
  // Mapping from subjacobians to their location in the combined jacobian
  MatrixCoords jacobian_coords_;  // row, col

  // All the chains that are combined to make a jacobian
  KDL::Chain kdl_chain_;

  // Mapping from subjacobians to their planning groups
  const robot_model::JointModelGroup* jmg_; //jacobian_groups_;

  // Track which joints are locked
  LockedJoints locked_joints_;

  // Track how many unlocked joints per chain
  int num_unlocked_joints_;

  // Store allocated memory for every chainsub_jacobians
  KDL::Jacobian2dPtr sub_jacobian_;

  // Sub joint index for every chain
  KDL::JntArrayPtr sub_q_in_;
};

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
    const std::vector<std::string>& tip_frames, const robot_model::JointModelGroup *jmg);

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

  bool linksToJointGroup( std::vector<const robot_model::JointModelGroup*> subgroups,
    const robot_model::LinkModel* from, const robot_model::LinkModel* to, const robot_model::JointModelGroup* &found_group);

  void lockJointsAfter(IKChainGroup &ik_group, int lock_after_id);

private:

  KDL::Twist t_twist_tmp_; // base of new segment's twist
  KDL::Frame T_frame_tmp_;
  KDL::Frame total_frame_;

  unsigned int nr_of_unlocked_joints_;
  bool verbose_;

  /*
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
  */

  // Used in generateChainJacobian
  unsigned int segment_nr_;

  // Store all sub jacobian data
  std::vector<IKChainGroup> chains_;

};

typedef boost::shared_ptr<JacobianGenerator> JacobianGeneratorPtr;

}
#endif


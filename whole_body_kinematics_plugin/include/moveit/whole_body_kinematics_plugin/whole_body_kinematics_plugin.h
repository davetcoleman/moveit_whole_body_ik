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
   Desc:   Non-chain IK solver
*/

#ifndef MOVEIT_WHOLE_BODY_IK__PLANNING_KDL_KINEMATICS_PLUGIN_
#define MOVEIT_WHOLE_BODY_IK__PLANNING_KDL_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// KDL
#include "kdl/jacobian.hpp" // load this here so that it overrides the version from kdl_urdf_parser
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

// This pkg
#include <moveit/whole_body_kinematics_plugin/jacobian_generator.h>
#include <moveit/whole_body_kinematics_plugin/ik_solver_pinverse.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Helper for Rviz
#include <moveit_visual_tools/visual_tools.h>

// Random numbers
#include <random_numbers/random_numbers.h>

namespace whole_body_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using KDL. This version can be used with any robot.
 */
class WholeBodyKinematicsPlugin : public kinematics::KinematicsBase
{
public:

  /**
   *  @brief Default constructor
   */
  WholeBodyKinematicsPlugin();

  virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
      ik_seed_state,
      default_timeout_,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
  }

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
  }

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    const IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
  }

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
  }

  virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    return searchPositionIK(ik_pose,
      ik_seed_state,
      timeout,
      solution,
      solution_callback,
      error_code,
      consistency_limits,
      options);
  }

  virtual bool getPositionFK(const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::Pose> &poses) const;

  virtual bool initialize(const std::string &robot_description,
    const std::string &group_name,
    const std::string &base_name,
    const std::string &tip_frame,
    double search_discretization)
  {
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    initialize(robot_description, group_name, base_name, tip_frames, search_discretization);
  }
 
  virtual bool initialize(const std::string &robot_description,
    const std::string &group_name,
    const std::string &base_name,
    const std::vector<std::string>& tip_frames,
    double search_discretization);

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const
  {
    return ik_group_info_.joint_names;
  }

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const
  {
    return ik_group_info_.link_names;
  }
  /**
   * \brief Check if this solver supports a given JointModelGroup.
   *
   * Override this function to check if your kinematics solver
   * implementation supports the given group.
   *
   * The default implementation just returns jmg->isChain(), since
   * solvers written before this function was added all supported only
   * chain groups.
   *
   * \param jmg the planning group being proposed to be solved by this IK solver
   * \param error_text_out If this pointer is non-null and the group is
   *          not supported, this is filled with a description of why it's not
   *          supported.
   * \return True if the group is supported, false if not.
   */
  const bool supportsGroup(const moveit::core::JointModelGroup *jmg, std::string* error_text_out = NULL) const;

protected:

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param check_consistency Set to true if consistency check needs to be performed
   * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const std::vector<double> &consistency_limits,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
  {
    // Convert single pose into a vector of one pose
    std::vector<geometry_msgs::Pose> ik_poses;
    ik_poses.push_back(ik_pose);

    return searchPositionIK(ik_poses,
      ik_seed_state,
      timeout,
      consistency_limits,
      solution,
      solution_callback,
      error_code,
      options);
  }

  /**
   * @brief Given a set of desired poses for a planning group with multiple end-effectors, search for the joint angles
   * required to reach them. This is useful for e.g. biped robots that need to perform whole-body IK.
   * Not necessary for most robots that have kinematic chains.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_poses the desired pose of each tip link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param options container for other IK options
   * @param context_state (optional) the context in which this request
   *        is being made.  The position values corresponding to
   *        joints in the current group may not match those in
   *        ik_seed_state.  The values in ik_seed_state are the ones
   *        to use.  This is passed just to provide the \em other
   *        joint values, in case they are needed for context, like
   *        with an IK solver that computes a balanced result for a
   *        biped.
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
    const moveit::core::RobotState* context_state = NULL) const;

private:

  bool timedOut(const ros::WallTime &start_time, double duration) const;

  /**
   * \brief Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the
   *  position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account.
   */
  int newtonRaphsonIterator(const KDL::JntArray& q_init, const std::vector<KDL::Frame>& kdl_poses, KDL::JntArray& q_out, std::size_t &total_loops) const;

  /** @brief Check whether the solution lies within the consistency limit of the seed state
   *  @param seed_state Seed state
   *  @param consistency_limit The returned state for redundant joint should be in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param solution solution configuration
   *  @return true if check succeeds
   */
  bool checkConsistency(const KDL::JntArray& seed_state,
    const std::vector<double> &consistency_limit,
    const KDL::JntArray& solution) const;

  int getJointIndex(const std::string &name) const;

  int getKDLSegmentIndex(const std::string &name) const;


  /**
   * \brief A wrapper for robot_state's setToRandomPositions function, for use with KDL
   */
  void getRandomConfiguration(KDL::JntArray &jnt_array) const;

  /** @brief Get a random configuration within joint limits close to the seed state
   *  @param seed_state Seed state
   *  @param consistency_limit The returned state will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   *  @param jnt_array Returned random configuration
   */
  void getRandomConfiguration(const KDL::JntArray& seed_state,
    const std::vector<double> &consistency_limits,
    KDL::JntArray &jnt_array) const;


  // Member Variables ----------------------------------------------------------------------------------------

  static const int TWIST_SIZE = 6;

  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  moveit_msgs::KinematicSolverInfo fk_group_info_; /** Store information for the forward kinematics solver */

  unsigned int dimension_; /** Dimension of the group */

  KDL::JntArray joint_min_, joint_max_; /** Joint limits */

  robot_model::RobotModelPtr robot_model_;

  robot_state::RobotStatePtr robot_state_;

  robot_model::JointModelGroup* joint_model_group_;
  int max_solver_iterations_;

  // Parameters
  double ee_pos_vel_limit_; // maximum allowed input positional velocity of the end effector before limiting
  double ee_rot_vel_limit_; // maximum allowed input rotational velocity of the end effector before limiting
  double epsilon_; // threshold of similiarity of desired ee pose and solved ee pose
  double joint_limit_offset_; // amount to move the joint away from the limit when the limit is hit. setting to zero will cause nan to occur in calculations
  double null_space_vel_gain_; // k, the amount the null space calculation affects the overall velocity gain

  bool verbose_; // show debug info
  bool debug_mode_; // math debug info, similar to euslisp's version
  bool visualize_search_; // publish to rviz every step of the solver

  // For visualizing things in rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // Velocity Pseudo Inverse Solver
  IkSolverPinversePtr ik_solver_vel_;
  JacobianGeneratorPtr jacobian_generator_;

  // Variables for use in newtonRaphsonIterator() ------------------------
  struct CartesionToJointData
  {
    CartesionToJointData(int dimension, int num_poses) 
      : qdot_(dimension),
        qdot_cache_(dimension),
        prev_H_(dimension),
        delta_twists_( num_poses * 6 ),
        jacobian_(dimension, num_poses * 6) // TODO fix
    {}
    KDL::Twist delta_twist_; // velocity and rotational velocity
    KDL::JntArray delta_twists_; // multiple twists from different end effectors, each of which have 6 dof
    KDL::JntArray qdot_;
    KDL::JntArray qdot_cache_;
    KDL::JntArray prev_H_; // track the change in performance criterion
    KDL::Frame current_pose_;
    KDL::Jacobian2d jacobian_;
  };
  typedef boost::shared_ptr<CartesionToJointData> CartesionToJointDataPtr;

  CartesionToJointDataPtr ctj_data_;
  // ----------------------------------------------------------------

  random_numbers::RandomNumberGenerator *rng_;

  ros::NodeHandle nh_;
};
}

#endif

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
*/


#include <moveit/simple_kdl_kinematics_plugin/simple_kdl_kinematics_plugin.h>
#include <class_loader/class_loader.h>

#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

//register SimpleKDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(simple_kdl_kinematics_plugin::SimpleKDLKinematicsPlugin, kinematics::KinematicsBase)

namespace simple_kdl_kinematics_plugin
{

SimpleKDLKinematicsPlugin::SimpleKDLKinematicsPlugin() {}

void SimpleKDLKinematicsPlugin::getRandomConfiguration(KDL::JntArray &jnt_array) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  robot_state_->setToRandomPositions(joint_model_group_);
  robot_state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array(i) = jnt_array_vector[i];
  }
}

void SimpleKDLKinematicsPlugin::getRandomConfiguration(const KDL::JntArray &seed_state,
  const std::vector<double> &consistency_limits,
  KDL::JntArray &jnt_array) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0 ; i < dimension_; ++i)
    near[i] = seed_state(i);

  joint_model_group_->getVariableRandomPositionsNearBy(robot_state_->getRandomNumberGenerator(), values, near, consistency_limits);

  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array(i) = values[i];
  }
}

bool SimpleKDLKinematicsPlugin::checkConsistency(const KDL::JntArray& seed_state,
  const std::vector<double> &consistency_limits,
  const KDL::JntArray& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  return true;
}

bool SimpleKDLKinematicsPlugin::initialize(const std::string &robot_description,
  const std::string& group_name,
  const std::string& base_frame,
  const std::vector<std::string>& tip_frames,
  double search_discretization)
{
  bool debug = true;
  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  // Load URDF and SRDF --------------------------------------------------------------------
  ros::NodeHandle private_handle("~");
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("kdl","URDF and SRDF must be loaded for KDL kinematics solver to work.");
    return false;
  }
  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  // Load the joint model group ------------------------------------------------------------------------
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if(joint_model_group_->isChain())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' is a chain, which this plugin does not support", group_name.c_str());
    return false;
  }
  if(!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  // Debug
  if (debug)
  {
    std::cout << std::endl << "Joint Model Variable Names: ------------------------------------------- " << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Convert to KDL Tree
  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize tree object");
    return false;
  }

  // Convert to multiple KDL chains
  // Hard coded for now, just for HRP2

  static const std::string BASE_LINK  = "BODY";
  static const std::string CHEST_LINK  = "CHEST_LINK1";
  static const std::string RIGHT_ARM_LINK = "RARM_LINK6";
  static const std::string LEFT_ARM_LINK = "LARM_LINK6";

  kdl_chains_.resize(2);

  // Left Chain
  if (!kdl_tree.getChain(CHEST_LINK, LEFT_ARM_LINK, kdl_chains_[0]))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize chain object");
    return false;
  }

  // Right Chain
  if (!kdl_tree.getChain(CHEST_LINK, RIGHT_ARM_LINK, kdl_chains_[1]))
  {
    ROS_ERROR_NAMED("kdl","Could not initialize chain object");
    return false;
  }

  /*
  // Torso Chain
  if (!kdl_tree.getChain(BASE_LINK, CHEST_LINK, kdl_chains_[2]))
  {
  ROS_ERROR_NAMED("kdl","Could not initialize chain object");
  return false;
  }
  */

  // Get the num of dimensions
  ROS_INFO_STREAM_NAMED("temp","Found " << joint_model_group_->getActiveJointModels().size() << " active joints and "
    << joint_model_group_->getMimicJointModels().size() << " mimic joints");

  dimension_ = joint_model_group_->getActiveJointModels().size();

  // Copy joint names and limits
  for (std::size_t i=0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    // Filter out non revolute or prismatic joint
    if(joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
      joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
      ik_group_info_.limits.insert(ik_group_info_.limits.end(), jvec.begin(), jvec.end());
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("kdl","This is the Simple KDL kinematics plugin, and it does not support mimic/redundant/non-revolute or prismatic joints");
    }
  }

  // Copy data to FK version
  fk_group_info_.joint_names = ik_group_info_.joint_names;
  fk_group_info_.limits = ik_group_info_.limits;

  if (debug)
  {
    std::cout << std::endl << "Tip Link Names: ------------------------------------------- " << std::endl;
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if(!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("srv","Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(), group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }
  fk_group_info_.link_names = joint_model_group_->getLinkModelNames();

  // Populate the joint limits
  joint_min_.resize(ik_group_info_.limits.size());
  joint_max_.resize(ik_group_info_.limits.size());
  for(unsigned int i=0; i < ik_group_info_.limits.size(); i++)
  {
    joint_min_(i) = ik_group_info_.limits[i].min_position;
    joint_max_(i) = ik_group_info_.limits[i].max_position;
  }

  // Get Solver Parameters
  private_handle.param("max_solver_iterations", max_solver_iterations_, 500);
  private_handle.param("epsilon", epsilon_, 1e-5);
  ROS_DEBUG_NAMED("kdl","Looking in private handle: %s for param name: %s",
    private_handle.getNamespace().c_str(), (group_name+"/position_only_ik").c_str());

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  ROS_DEBUG_NAMED("kdl","KDL solver initialized");
  return true;
}

int SimpleKDLKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_group_info_.joint_names.size(); i++) {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

int SimpleKDLKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  /*
    int i=0;
    while (i < (int)kdl_chain_.getNrOfSegments()) {
    if (kdl_chain_.getSegment(i).getName() == name) {
    return i+1;
    }
    i++;
    }
    return -1;
  */
}

bool SimpleKDLKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool SimpleKDLKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
  const std::vector<double> &ik_seed_state,
  double timeout,
  const std::vector<double> &consistency_limits,
  std::vector<double> &solution,
  const IKCallbackFn &solution_callback,
  moveit_msgs::MoveItErrorCodes &error_code,
  const kinematics::KinematicsQueryOptions &options,
  const moveit::core::RobotState* context_state) const
{
  ros::WallTime n1 = ros::WallTime::now();

  // Check if seed state correct
  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("srv","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check consistency limits
  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("kdl","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("srv","Mismatched number of pose requests (" << ik_poses.size()
      << ") to tip frames (" << tip_frames_.size() << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Joint arrays
  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);

  // Setup Inv Kin Solver -------------------------------------
  // the desired positions of the chain used by to resolve the redundancy
  KDL::JntArray opt_positions(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    opt_positions(i) = ik_seed_state[i]; // This could also be half of joint range value, or minimum torque value

  // the weights applied in the joint space
  KDL::JntArray weights(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    weights(i) = 1;

  // if a singular value is below this value, its inverse is set to zero, default: 0.00001
  double eps=0.00001;
  // maximum iterations for the svd calculation, default: 150
  int maxiter=150;
  // alpha the null-space velocity gain
  double alpha = 0.25;

  ROS_INFO_STREAM_NAMED("temp","number of poses: " << ik_poses.size());

  // inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity
  KDL::IkSolverVel_pinv_nso ik_solver_vel(kdl_chains_, opt_positions, weights, eps, maxiter, alpha, ik_poses.size());

  // Create new instances of the other solvers -------------------
  KDL::ChainFkSolverPos_recursive fk_solver(kdl_chains_[0]);
  //KDL::ChainIkSolverPos_NR_JL ik_solver_pos(kdl_chains_, _, fk_solver, ik_solver_vel, max_solver_iterations_, epsilon_);

  // Setup solution struct
  solution.resize(dimension_);

  // Convert format of pose
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_poses[0], pose_desired); // TODO

  // Copy the seed state
  for(unsigned int i=0; i < dimension_; i++)
    jnt_seed_state(i) = ik_seed_state[i];
  jnt_pos_in = jnt_seed_state;

  // Loop until solution is within epsilon
  unsigned int counter(0);
  while(true)
  {
    ROS_DEBUG_NAMED("kdl","Iteration: %d, time: %f, Timeout: %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
    counter++;

    // Check if timed out
    if(timedOut(n1,timeout))
    {
      ROS_DEBUG_NAMED("kdl","IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;
    }

    // Update random values unless we are on the first iteration
    if (counter > 1)
    {
      if(!consistency_limits.empty())
      {
        // Look for random joint values within consistency limits of the seed state
        getRandomConfiguration(jnt_seed_state, consistency_limits, jnt_pos_in);
      }
      else
      {
        // Look for any random joint values
        getRandomConfiguration(jnt_pos_in);
      }
      ROS_DEBUG_NAMED("kdl","New random configuration");
      for(unsigned int j=0; j < dimension_; j++)
        ROS_DEBUG_NAMED("kdl","%d %f", j, jnt_pos_in(j));
    }

    // Solve
    //int ik_valid = ik_solver_pos.CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);
    int ik_valid = cartesionToJoint(jnt_pos_in, pose_desired, jnt_pos_out, fk_solver, ik_solver_vel);
    ROS_DEBUG_NAMED("kdl","IK valid: %d", ik_valid);

    // Check solution
    if( !consistency_limits.empty() &&
      ( (ik_valid < 0 && !options.return_approximate_solution) || !checkConsistency(jnt_seed_state, consistency_limits, jnt_pos_out)) )
    {
      ROS_DEBUG_NAMED("kdl","Could not find IK solution: does not match consistency limits");
      continue;
    }
    else if(ik_valid < 0 && !options.return_approximate_solution)
    {
      ROS_DEBUG_NAMED("kdl","Could not find IK solution");
      continue;
    }

    // Convert solution to MoveIt format
    for(unsigned int j=0; j < dimension_; j++)
      solution[j] = jnt_pos_out(j);

    // Optionally check again with custom callback
    /*
      if(!solution_callback.empty())
      solution_callback(ik_poses,solution,error_code);
      else
      error_code.val = error_code.SUCCESS;
    */
    // Check if callback was sucessful
    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("kdl","Solved after " << counter << " iterations");
      return true;
    }
  }

  // Should actually never get here
  /*
    ROS_DEBUG_NAMED("kdl","An IK that satisifes the constraints and is collision free could not be found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  */
  throw; // just testing
}

int SimpleKDLKinematicsPlugin::cartesionToJoint(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out,
  KDL::ChainFkSolverPos& fksolver, KDL::IkSolverVel_pinv_nso& iksolver) const
{
  // First solution guess is the seed state
  q_out = q_init;

  KDL::JntArray delta_q(dimension_);
  KDL::Frame p_current;
  KDL::Twist delta_twist;

  unsigned int i;
  
  // Iterate on approximate guess of joint values 'q_out'
  for ( i=0; i < max_solver_iterations_; i++ )
  {
    // Forward kinematics from current guess to new frame
    fksolver.JntToCart(q_out, p_current);

    // Calculate the difference between our desired pose and current pose
    delta_twist = diff(p_current, p_in);

    // Check if the difference between our desired pose and current
    // pose is within epsilon tolerance. if it is,  we are done
    if (Equal(delta_twist, KDL::Twist::Zero(), epsilon_))
      break;

    // Run velocity solver - delta_q is returned as the joint velocities
    // (change in joint value guess)
    iksolver.CartToJnt(q_out, delta_twist, delta_q);

    // Add current guess 'q_out' with our new change in guess
    Add(q_out, delta_q, q_out);

    // Enforce low joint limits
    for (unsigned int j = 0; j < joint_min_.rows(); j++)
    {
      if (q_out(j) < joint_min_(j))
        q_out(j) = joint_min_(j);
    }

    // Enforce high joint limits
    for (unsigned int j = 0; j<joint_max_.rows(); j++)
    {
      if (q_out(j) > joint_max_(j))
        q_out(j) = joint_max_(j);
    }
  }

  // Check if we succeeded in finding a close enough solution
  if (i != max_solver_iterations_)
    return 0;
  else
    return -3;
}

bool SimpleKDLKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
  const std::vector<double> &joint_angles,
  std::vector<geometry_msgs::Pose> &poses) const
{
  /*
    ros::WallTime n1 = ros::WallTime::now();

    poses.resize(link_names.size());
    if(joint_angles.size() != dimension_)
    {
    ROS_ERROR_NAMED("kdl","Joint angles vector must have size: %d",dimension_);
    return false;
    }

    KDL::Frame p_out;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    KDL::JntArray jnt_pos_in(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
    {
    jnt_pos_in(i) = joint_angles[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chains_);

    bool valid = true;
    for(unsigned int i=0; i < poses.size(); i++)
    {
    ROS_DEBUG_NAMED("kdl","End effector index: %d",getKDLSegmentIndex(link_names[i]));
    if(fk_solver.JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
    tf::poseKDLToMsg(p_out,poses[i]);
    }
    else
    {
    ROS_ERROR_NAMED("kdl","Could not compute FK for %s",link_names[i].c_str());
    valid = false;
    }
    }
    return valid;
  */
  ROS_ERROR_STREAM_NAMED("temp","Currently not implemented");
  return false;
}

const bool SimpleKDLKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
  std::string* error_text_out) const
{
  if (jmg->isChain())
  {
    if(error_text_out)
    {
      *error_text_out = "This plugin only supports joint groups which are not chains";
    }
    return false;
  }

  return true;
}


} // namespace

//PLUGINLIB_EXPORT_CLASS(simple_kdl_kinematics_plugin::SimpleKDLKinematicsPlugin,
//  kinematics::KinematicsBase);

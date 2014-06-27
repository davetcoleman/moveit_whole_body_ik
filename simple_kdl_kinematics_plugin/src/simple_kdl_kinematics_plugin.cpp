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
#include <tf_conversions/tf_eigen.h>
#include <kdl_parser/kdl_parser.hpp>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

#include <boost/format.hpp>

#include <moveit/simple_kdl_kinematics_plugin/kdl/utilities/svd_HH.hpp> // temp

//register SimpleKDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(simple_kdl_kinematics_plugin::SimpleKDLKinematicsPlugin, kinematics::KinematicsBase)

namespace simple_kdl_kinematics_plugin
{

SimpleKDLKinematicsPlugin::SimpleKDLKinematicsPlugin()
  : verbose_(false)
{}

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
    ROS_ERROR_NAMED("kdl","Group '%s' is a chain, which this plugin might not support. I've only tested it with non-chains", group_name.c_str());
    return false;
  }
  if(!joint_model_group_->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("kdl","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  // Verbose
  if (verbose_)
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
  static const std::string LEFT_ARM_LINK = "LARM_LINK6";
  static const std::string RIGHT_ARM_LINK = "RARM_LINK6";

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

  if (verbose_)
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
  robot_state_->setToDefaultValues();

  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new moveit_visual_tools::VisualTools("/odom","/hrp2_visual_markers", robot_model_));
  visual_tools_->loadRobotStatePub("/moveit_whole_body_ik");

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
  // the desired positions of the chain used to resolve the redundancy
  KDL::JntArray opt_positions(dimension_);
  //for(unsigned int i=0; i < dimension_; i++)
  //  opt_positions(i) = ik_seed_state[i]; // This could also be half of joint range value, or minimum torque value

  // Set to midpoint of each joint
  //std::cout << "Setting desired positions: " << std::endl;
  for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i)
  {
    moveit::core::VariableBounds bounds = joint_model_group_->getActiveJointModels()[i]->getVariableBounds()[0];
    opt_positions(i) = (bounds.min_position_ + bounds.max_position_) / 2.0;
    //std::cout << "pos " << i << ": " << opt_positions(i) << std::endl;
  }



  // the weights applied in the joint space
  // i think this decides how much power the 'opt_positions' (above) has on the overall velocity.
  KDL::JntArray weights(dimension_);
  for(unsigned int i=0; i < dimension_; i++)
    weights(i) = 0.5;

  // if a singular value is below this value, its inverse is set to zero, default: 0.00001
  double eps=0.00001;
  // maximum iterations for the svd calculation, default: 150
  int maxiter=150;
  // alpha the null-space velocity gain
  double alpha = 0.25;

  if (verbose_)
    ROS_INFO_STREAM_NAMED("searchPositionIK","Number of poses: " << ik_poses.size());

  // inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity
  KDL::IkSolverVel_pinv_nso ik_solver_vel(ik_poses.size(), dimension_, kdl_chains_, opt_positions, weights, eps, maxiter, alpha, verbose_);

  // Create fk solvers for each chain
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos> > fk_solvers;
  for (std::size_t chain_id = 0; chain_id < kdl_chains_.size(); ++chain_id)
  {
    boost::shared_ptr<KDL::ChainFkSolverPos> temp (new KDL::ChainFkSolverPos_recursive(kdl_chains_[chain_id]));
    fk_solvers.push_back(temp);
  }

  /*
  // Test Jnt2Jac Jacobian Generator
  KDL::JntArray q_in(dimension_);
  KDL::JntArray delta_twists( ik_poses.size() * 6 ); // multiple twists from different end effectors, each of which have 6 dof
  KDL::JntArray qdot(dimension_);
  KDL::Jacobian2d jacobian(dimension_, ik_poses.size() * 6);

  // Copy the seed state
  for(unsigned int i=0; i < dimension_; i++)
  {
  q_in(i) = 0.5; // ik_seed_state[i];
  //std::cout << "q_in " << q_in(i) << std::endl;
  }

  // Find pseudo inverse
  for (std::size_t i = 0; i < delta_twists.rows(); ++i)
  {
  KDL::SetToZero(delta_twists);
  delta_twists(i) = 1;
  ik_solver_vel.CartToJnt(q_in, delta_twists, qdot);
  //qdot.print();

  for (std::size_t j = 0; j < qdot.rows(); ++j)
  {
  jacobian(i,j) = qdot(j);
  }
  }

  // Combine to one jacobian
  jacobian.print();

  // Compare to regular jacobian
  //KDL::SetToZero(jacobian);
  KDL::JntToJacSolver jnt2jac(kdl_chains_, dimension_, verbose_);

  jnt2jac.JntToJac(q_in,jacobian);
  jacobian.print();
  */

  // PSEUDO INVERSE ----------------
  // Using the svd decomposition (jac_pinv=V*S_pinv*Ut):

  /*
    KDL::Jacobian2d jacobian_pinv(dimension_, ik_poses.size() * 6);
    KDL::SVD_HH svd(jacobian);
    int ret = svd.calculate(jacobian,U,S,V,maxiter); // Pseudo inverse

    double sum;
    unsigned int i,j;
    for (i=0;i<jacobian.columns();i++)
    {
    sum = 0.0;
    for (j=0;j<jacobian.rows();j++)
    {
    sum += U[j](i) * v_in(j);
    }
    //If the singular value is too small (<eps), don't invert it but
    //set the inverted singular value to zero (truncated svd)
    tmp(i) = sum * ( fabs(S(i)) < eps ? 0.0 : 1.0/S(i) );
    }
  */



  // Setup solution struct
  solution.resize(dimension_);

  // Convert format of pose
  std::vector<KDL::Frame> kdl_poses;
  for (std::size_t i = 0; i < ik_poses.size(); ++i)
  {
    KDL::Frame temp;
    tf::poseMsgToKDL(ik_poses[i], temp);
    kdl_poses.push_back(temp);
  }

  // Copy the seed state
  for(unsigned int i=0; i < dimension_; i++)
  {
    jnt_seed_state(i) = ik_seed_state[i];
    if (verbose_)
      std::cout << "Seed state " << i << ": " << ik_seed_state[i] << std::endl;
  }
  jnt_pos_in = jnt_seed_state;

  // Loop until solution is within epsilon
  unsigned int counter(0);
  while(true)
  {
    ROS_DEBUG_NAMED("kdl","Outer most iteration: %d, time: %f, Timeout: %f",counter,(ros::WallTime::now()-n1).toSec(),timeout);
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
      if (verbose_)
      {
        ROS_DEBUG_NAMED("kdl","New random configuration");
        for(unsigned int j=0; j < dimension_; j++)
          ROS_DEBUG_NAMED("kdl","%d %f", j, jnt_pos_in(j));
      }
    }

    // Solve
    int ik_valid = cartesionToJoint(jnt_pos_in, kdl_poses, jnt_pos_out, fk_solvers, ik_solver_vel);
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
    if(!solution_callback.empty())
    {
      //solution_callback(ik_poses,solution,error_code); // TODO
      ROS_ERROR_STREAM_NAMED("temp","Implement callbacks");
      exit(-1);
    }
    else
      error_code.val = error_code.SUCCESS;

    // Check if callback was sucessful
    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("kdl","Solved after " << counter << " iterations");
      return true;
    }
  }
  // should never get here
}

// Convert Eigen::Affine3d to KDL::Frame
void poseEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  // Set position
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e.matrix()(i,3);

  // Set orientation
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e.matrix()(i/3,i%3);

  //k.M.data[i] = t.getBasis()[i/3][i%3];
}

/* \brief Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the
 *  position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account. */
int SimpleKDLKinematicsPlugin::cartesionToJoint(const KDL::JntArray& q_init, const std::vector<KDL::Frame>& kdl_poses, KDL::JntArray& q_out,
  std::vector<boost::shared_ptr<KDL::ChainFkSolverPos> > &fk_solvers, KDL::IkSolverVel_pinv_nso& ik_solver_vel) const
{
  // First solution guess is the seed state
  q_out = q_init;

  int dimension_of_subgroup = dimension_/2; // TODO remove this hack and replace with robot_state/robot_model maybe
  KDL::JntArray q_out_subgroup(dimension_of_subgroup); // TODO this is two-arm specific and is a terrible hack
  KDL::JntArray qdot(dimension_);
  KDL::Frame current_pose;
  KDL::Twist delta_twist; // velocity and rotational velocity
  KDL::JntArray delta_twists( kdl_poses.size() * 6 ); // multiple twists from different end effectors, each of which have 6 dof
  bool all_poses_valid; // track if any pose is still not within epsilon distance to its goal
  static const int TWIST_SIZE = 6;
  // Iterate on approximate guess of joint values 'q_out'
  std::size_t solver_iteration;
  std::vector<double> current_joint_values(dimension_); // for visualizing

  bool use_robot_state = false;


  for (  solver_iteration = 0; solver_iteration < max_solver_iterations_; solver_iteration++ )
  {
    if (verbose_)
      ROS_WARN_STREAM_NAMED("cartesionToJoint","Starting solver iteration " << solver_iteration << " of " << max_solver_iterations_ << " =====================");

    if (!ros::ok())
    {
      ROS_ERROR_STREAM_NAMED("cartesianToJoint","ROS requested shutdown");
      return -1;
    }

    all_poses_valid = true;

    if (use_robot_state)
    {
      // Convert to vector of doubles
      for (std::size_t i = 0; i < q_out.rows(); ++i)
        current_joint_values[i] = q_out(i);

      if (verbose_)
      {
        std::cout << "Input robot state joints: " << std::endl;
        std::copy(current_joint_values.begin(), current_joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));
      }
      robot_state_->setJointGroupPositions(joint_model_group_, current_joint_values);

      // Visualize progress
      if (verbose_ && solver_iteration % 10 == 0)
      {
        // Publish
        visual_tools_->publishRobotState(robot_state_);
        ros::Duration(0.15).sleep();
      }
    }

    // For each end effector
    for (std::size_t pose_id = 0; pose_id < kdl_poses.size(); ++pose_id)
    {
      if (!use_robot_state)
      {
        /*/ Error check my hack
          if (dimension_of_subgroup != kdl_chains_[pose_id].getNrOfJoints())
          {
          std::cout << "Hack is bad, chain size assumption does not hold " << std::endl;
          exit(-1);
          }*/

        // Convert q_out to its subgroup
        for (std::size_t i = 0; i < dimension_of_subgroup; ++i)
        {
          q_out_subgroup(i) = q_out(i + pose_id * dimension_of_subgroup);
          if (verbose_)
            std::cout << "q_out_subgroup(" <<  i << ") = " << q_out(i + pose_id * dimension_of_subgroup) << std::endl;
        }

        // Forward kinematics from current guess to new frame
        fk_solvers[pose_id]->JntToCart(q_out_subgroup, current_pose);

        // Convert subgroup to q_out
        for (std::size_t i = 0; i < dimension_of_subgroup; ++i)
        {
          q_out(i + pose_id * dimension_of_subgroup) = q_out_subgroup(i);
          if (verbose_)
            std::cout << "q_out(" <<  (i + pose_id * dimension_of_subgroup) << ") = " << q_out_subgroup(i) << std::endl;
        }
      }
      else
      {

        // Do forward kinematics to get new EE pose location
        Eigen::Affine3d eef_pose;
        if (pose_id == 0)
          eef_pose = robot_state_->getGlobalLinkTransform("LARM_LINK6");
        else
          eef_pose = robot_state_->getGlobalLinkTransform("RARM_LINK6");

        // Bring the pose to the frame of the IK solver
        robot_state_->setToIKSolverFrame( eef_pose, getBaseFrame() );

        // Convert Eigen::Affine3d to KDL::Frame
        poseEigenToKDL(eef_pose, current_pose);
      }

      // Calculate the difference between our desired pose and current pose
      delta_twist = diff(current_pose, kdl_poses[pose_id]);   // v_in = actual - target

      // Check if the difference between our desired pose and current pose is within epsilon tolerance
      if (!Equal(delta_twist, KDL::Twist::Zero(), epsilon_))
        all_poses_valid = false;

      // Add this twist to our large twist vector from multiple end effectors
      for (std::size_t twist_index = 0; twist_index < TWIST_SIZE; ++twist_index)
      {
        //std::cout << " >>> Twist value is: " << delta_twist( twist_index ) << " being placed in index " << TWIST_SIZE * pose_id + twist_index << std::endl;
        delta_twists( TWIST_SIZE * pose_id + twist_index ) = delta_twist( twist_index );
      }
    }

    // See twist delta
    if (verbose_)
    {
      for (std::size_t i = 0; i < delta_twists.rows(); ++i)
      {
        std::cout << boost::format("%12.5f") % delta_twists(i);
      }
      std::cout << std::endl;
    }

    // Check if we are done
    if (all_poses_valid)
    {
      ROS_DEBUG_STREAM_NAMED("cartesionToJoint","All of our end effectors are withing epsilon tolerance of goal location");
      break;
    }

    // Run velocity solver - qdot is returned as the joint velocities (delta q)
    // (change in joint value guess)
    ik_solver_vel.CartToJnt(q_out, delta_twists, qdot);


    // See velocities
    if (verbose_)
    {
      for (std::size_t i = 0; i < q_out.rows(); ++i)
      {
        std::cout << boost::format("%12.5f") % q_out(i);
      }
      std::cout << std::endl;
    }

    // Add current guess 'q_out' with our new change in guess qdot (delta q)
    Add(q_out, qdot, q_out); // q_out = q_out + q_delta

    // Enforce low joint limits
    for (unsigned int j = 0; j < joint_min_.rows(); j++)
    {
      if (q_out(j) < joint_min_(j))
      {
        if (verbose_)
          ROS_ERROR_STREAM_NAMED("cartesionToJoint","Min joint limit hit for joint " << j);
        q_out(j) = joint_min_(j);
      }
    }

    // Enforce high joint limits
    for (unsigned int j = 0; j<joint_max_.rows(); j++)
    {
      if (q_out(j) > joint_max_(j))
      {
        if (verbose_)
          ROS_ERROR_STREAM_NAMED("cartesionToJoint","Max joint limit hit for joint " << j);
        q_out(j) = joint_max_(j);
      }
    }
  }

  // Check if we succeeded in finding a close enough solution
  if (solver_iteration != max_solver_iterations_)
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

    KDL::Frame f_out;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    KDL::JntArray jnt_pos_in(dimension_);
    for(unsigned int i=0; i < dimension_; i++)
    {
    jnt_pos_in(i) = joint_angles[i];
    }

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

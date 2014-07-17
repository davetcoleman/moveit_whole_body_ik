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
   Based on ik solver code from Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
*/

#include <moveit/whole_body_kinematics_plugin/ik_solver_pinverse.h>
#include <moveit/whole_body_kinematics_plugin/MatrixSolvers.h> // SVD solver
#include <iostream> // TODO remove
#include <boost/format.hpp>
#include <moveit/macros/console_colors.h>

namespace whole_body_kinematics_plugin
{

IkSolverPinverse::IkSolverPinverse(int _num_tips, int _num_joints, JntArray _joint_min, JntArray _joint_max,
                                   JntArray weights, const Jacobian2d& jacobian, double _eps, int _maxiter, bool verbose)
  :
  // Load the jacobian to have #joint ROWS x #tips COLS
  original_jacobian(_num_joints, _num_tips*6), // TODO remove this? this is temp testing for using non-weighted jacobian in null space
  svd_(jacobian),
  U(_num_tips*6,JntArray(_num_joints)),
  S(_num_joints),
  V(_num_joints, JntArray(_num_joints)),
  H_(_num_joints),
  tmp(_num_joints),
  tmp2(_num_joints-6*_num_tips),
  eps(_eps),
  maxiter(_maxiter),
  num_tips(_num_tips),
  weights_(weights),
  W_(_num_joints),
  // Properties of joint
  joint_min(_joint_min),
  joint_max(_joint_max),
  joint_mid(_joint_min.rows()),
  joint_constant1(_joint_min.rows()),
  joint_constant2(_joint_min.rows()),
  joint_constant3(_joint_min.rows()),
  joint_constant4(_joint_min.rows()),
  // Debugging
  verbose_(verbose)
{
  for (std::size_t i = 0; i < joint_min.rows(); ++i)
  {
    // Calculate midpoint of all joints
    joint_mid(i) = (joint_max(i) + joint_min(i)) / 2.0;

    // Calculate a component of the Zghal performance criterion
    joint_constant1(i) = (joint_max(i) - joint_min(i)) * (joint_max(i) - joint_min(i));
    // Calculate components of the other performance criterion (from JSK - name unknown)
    joint_constant2(i) = (joint_max(i) + joint_min(i)) / 2.0;
    joint_constant3(i) = (joint_max(i) - joint_min(i)) / 2.0;
    joint_constant4(i) = pow(joint_max(i) - joint_min(i), 2.0);
  }

  // Initialize the matrix
  pinverse_.resize(_num_joints, _num_tips*6);
  tmp3_.resize    (_num_joints, _num_joints);
  tmp4_.resize    (_num_joints, _num_tips*6);
  identity_.resize(_num_joints, _num_joints);

  identity_.setIdentity();
}

/**
 * \param q_in - current joint location, (?? = this allows us to linearize the jacobian around its current state)
 * \param xdot_in - the difference between desired pose and current pose
 * \param jacobian
 * \param qdot_out - velocity (delta q) - change in joint values
 * \param prev_H - contains the previous performance criterion
 7 */
int IkSolverPinverse::cartesianToJoint(const JntArray& q_in, const JntArray& xdot_in, Jacobian2d& jacobian,
                                       JntArray& qdot_out, JntArray& prev_H, bool debug_mode, bool is_first_iteration, double &null_space_vel_gain)
{
  // weights:
  bool use_wln = false;
  // inverse methods:
  bool use_psm = true; // HRP
  bool use_kdl = false;
  // null space:
  bool use_gpm = false;

  // Copy original jacobian for later calculations TODO remove this?
  //original_jacobian = jacobian;

  // Automatically stop calculating gpm if null_space_vel_gain is below threshold to save calculations
  if (null_space_vel_gain < 0.0001)
  {
    use_gpm = false;
  }

  unsigned int i,j;
  double sum;

  // TODO: choose best ratio
  double sv_ratio=1e-300; // default: 1.0e-3

  if (debug_mode && false)
  {
    // Show original non-weighted jacobian
    std::cout << "Jacobian : ";
    for (i = 0; i < jacobian.rows(); ++i)
    {
      for (j = 0; j < jacobian.columns(); ++j)
      {
        std::cout << boost::format("%10.4f") % jacobian(i,j);
      }
      std::cout << std::endl;

      // close the whole matrix unless we are on last value
      if ( !(j == jacobian.columns() && i == jacobian.rows() - 1) )
        std::cout << "           ";
    }
  }

  // Find the Weighted Least Norm Jacobian
  if (use_wln)
  {
    weightedLeastNorm(q_in, jacobian, prev_H, debug_mode, is_first_iteration);

    if (debug_mode)
    {
      // Show jacobian with added weights
      std::cout << "Jw       : ";
      for (i = 0; i < jacobian.rows(); ++i)
      {
        for (j = 0; j < jacobian.columns(); ++j)
        {
          std::cout << boost::format("%10.4f") % jacobian(i,j);
        }
        std::cout << std::endl;

        // close the whole matrix unless we are on last value
        if ( !(j == jacobian.columns() && i == jacobian.rows() - 1) )
          std::cout << "           ";
      }
    }
  }

  // Method 1: Calculate the entire pseudo inverse as found in HRP --------------------------
  if (use_psm)
  {
    // Calculate pseudo inverse
    // pinv(A) = V*S^(-1)*U^(T)
    hrp::calcPseudoInverse(jacobian.data, pinverse_, sv_ratio);

    //print(pinverse_);

    // Apply pinverse to the velocity vector
    for (i = 0; i < jacobian.columns(); ++i) // row of pinverse,
    {
      sum = 0.0;
      for (j = 0; j < jacobian.rows(); ++j) // column of pinverse
      {
        sum += pinverse_(i,j) * xdot_in(j);
      }

      if (use_wln && !is_first_iteration)
        qdot_out(i) = W_(i) *  sum; // use weight to slow down joint velocities
      else
        qdot_out(i) = sum;
    }
  }

  // Method 2: Do SVD faster using the KDL method but directly calc pseudo invers---------------------------------------------
  if (use_kdl)
  {
    //Do a singular value decomposition of "jacobian" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jacobian = U*S*Vt
    svd_.calculate(jacobian,U,S,V,maxiter);

    if (verbose_)
    {
      std::cout << "U ------------------- " << std::endl;
      std::cout << "rows " << U.size() << std::endl;
      std::cout << "col " << U[0].rows() << std::endl;
      for (i = 0; i < U.size(); ++i)
      {
        for (j = 0; j < U[i].rows(); ++j)
        {
          std::cout << U[i](j) << ", ";
        }
        std::cout << std::endl;
      }

      std::cout << "S " << std::endl;
      S.print();
    }

    //J^(+) = V*S^(-1)*U^(T)

    int m = jacobian.columns(); // joints
    int n = jacobian.rows(); // eefs

    // We have to calculate qdot_out = jac_pinv*xdot_in
    // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    // qdot_out = V*S_pinv*Ut*xdot_in

    //first we calculate S_pinv*Ut
    for (i = 0;i < n; i++) // eefs / rows
    {
      for (j = 0;j < m; j++) // joints / columns
      {
        //If the singular value is too small (<eps), don't invert it but
        //set the inverted singular value to zero (truncated svd)
        if ( fabs(S(j)) < eps )
        {
          tmp4_(j,i) = 0.0 ;
        }
        else
        {
          tmp4_(j,i) = U[i](j) / S(j) ;
        }
      }
    }

    if (verbose_)
    {
      std::cout << "tmp4 " << std::endl;
      print(tmp4_);
    }

    // tmp4 = S_pinv*Ut
    // we still have to premultiply it with V
    for (i = 0; i < n; i++) // eefs / rows
    {
      for (j = 0; j < m; j++) // joints / columns
      {
        sum = 0.0;
        for (std::size_t k = 0; k < m; ++k)
        {
          sum += V[j](k) * tmp4_(k,i);
        }
        pinverse_(j, i) = sum;
      }
    }

    if (verbose_)
    {
      std::cout << "pinverse " << std::endl;
      print(pinverse_);
    }

    // Apply pinverse to the velocity vector
    for (i = 0; i < jacobian.columns(); ++i) // row of pinverse,
    {
      sum = 0.0;
      for (j = 0; j < jacobian.rows(); ++j) // column of pinverse
      {
        sum += pinverse_(i,j) * xdot_in(j);
      }

      if (use_wln && !is_first_iteration)
        qdot_out(i) = W_(i) *  sum;
      else
        qdot_out(i) = sum;
    }
  }

  if (debug_mode)
  {
    // Show pseudo-inverse, transposed
    std::cout << "J#t      : ";
    for (j = 0; j < pinverse_.cols(); ++j)
    {
      for (i = 0; i < pinverse_.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % pinverse_(i,j);
      }
      std::cout << std::endl;

      // close the whole matrix unless we are on last value
      if ( !(j == pinverse_.cols() - 1 && i == pinverse_.rows()) )
        std::cout << "           ";
    }

    // Show current xdot location
    /*
      std::cout << "xdot_in  : ";
      for (std::size_t i = 0; i < xdot_in.rows(); ++i)
      {
      std::cout << boost::format("%10.4f") % xdot_in(i);
      }
      std::cout << std::endl;
    */


  }

  // Gradient Projection Method using Null Space ----------------------------------------------

  if (use_gpm)
  {
    if (debug_mode)
    {
      // Pseudo inverse times xdot
      std::cout << "qdot_orig: ";

      for (std::size_t i = 0; i < qdot_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % qdot_out(i);
      }
      std::cout << std::endl;
    }

    // H
    for(i = 0; i < jacobian.columns(); i++) // joints
    {
      // Performance criterion used by EUSLISP implementation at JSK
      //   H will range from 1 at joint_min to  -1 at joint_max
      H_(i) = ( joint_constant2(i) - q_in(i) ) / joint_constant3(i);

      // Calculate the change in joint location relative to its limits
      /*
        H_(i) =
        pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) )
        /
        ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) );
      */
    }

    if (debug_mode)
    {
      // Scalar of null space component
      std::cout << "null_gain: ";
      std::cout << boost::format("%10.4f") % null_space_vel_gain;
      std::cout << std::endl;

      // Performance criterion
      std::cout << "H        : ";
      for (std::size_t i = 0; i < H_.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % H_(i);
      }
      std::cout << std::endl;
    }

    // qdot += k(I - J^(+)*J)
    tmp3_ = null_space_vel_gain * (identity_ - pinverse_ * jacobian.data) * H_.data;  // TODO is this jacobian already weighted?

    // Apply the null space
    qdot_out.data += tmp3_;

    if (debug_mode)
    {
      // Null space component
      std::cout << "gpm compt: " << MOVEIT_CONSOLE_COLOR_CYAN;

      for (std::size_t i = 0; i < tmp3_.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % tmp3_(i);
      }
      std::cout << MOVEIT_CONSOLE_COLOR_RESET << std::endl;

      // Pseudo inverse times xdot
      std::cout << "qdot_out : ";

      for (std::size_t i = 0; i < qdot_out.rows(); ++i)
      {
        std::cout << boost::format("%10.4f") % qdot_out(i);
      }
      std::cout << std::endl;
    }

  }

  return 1;
}

bool IkSolverPinverse::weightedLeastNorm(const JntArray& q_in, Jacobian2d& jacobian, JntArray& prev_H, bool debug_mode, bool is_first_iteration)
{
  double gradientH;
  std::vector<double> deltaH(W_.rows()); // TODO remove this, only for debugging
  std::vector<double> gradientHs(W_.rows()); // TODO remove this, only for debugging

  if (debug_mode)
  {
    std::cout << "prevH    : ";
    for (std::size_t i = 0; i < W_.rows(); ++i)
    {
      formatNum(prev_H(i));
    }
    std::cout << std::endl;
  }

  // Find the Weighted Least Norm Jacobian
  for (std::size_t i = 0; i < W_.rows(); ++i)
  {
    // Calculate the change in joint location relative to its limits
    // | gradient H(theta) |
    gradientH =
      joint_constant4(i) * (2*q_in(i) - joint_max(i) - joint_min(i) )
      /
      ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) );  // TODO absolute value calculated here

    gradientHs[i] = gradientH; // TODO remove, only for debugging

    // If joint is moving away from limit do not change its weight (leave as 1)
    deltaH[i] = fabs(gradientH) - fabs(prev_H(i)); // TODO remove, only for debugging

    if (fabs(gradientH) - fabs(prev_H(i)) >= 0 && !is_first_iteration) // change in performance criterion is positive
    {
      W_(i) = 1 + fabs(gradientH);
    }
    else
    {
      W_(i) = 1;
    }

    // Prepare the weight for multiplication by the jacobian
    // W = W^(-1/2)
    W_(i) = 1/sqrt(W_(i));

    // Save gradient for next iteration to find delta
    prev_H(i) = gradientH;
  }

  // Add the torso weights TODO make this not hard coded
  double torso_weight = 0.01;
  W_(0) *= torso_weight;
  W_(1) *= torso_weight;

  if (debug_mode)
  {
    std::cout << "gradH    : ";
    for (std::size_t i = 0; i < W_.rows(); ++i)
    {
      formatNum(gradientHs[i]);
    }
    std::cout << std::endl;

    std::cout << "deltaH   : ";
    for (std::size_t i = 0; i < W_.rows(); ++i)
    {
      formatNum(deltaH[i]);
    }
    std::cout << std::endl;
    //std::cout << "   if deltaH < 0 then weight = 1" << std::endl;

    std::cout << "weight   : ";
    double tmp;
    for (std::size_t i = 0; i < W_.rows(); ++i)
    {
      tmp = pow(1/W_(i),2);
      formatNum(tmp);
    }
    std::cout << std::endl;

    std::cout << "W^(-1)   : " << MOVEIT_CONSOLE_COLOR_BLUE;
    for (std::size_t i = 0; i < W_.rows(); ++i)
    {
      std::cout << boost::format("%10.4f") % W_(i);
    }
    std::cout << std::endl << MOVEIT_CONSOLE_COLOR_RESET;
  }


  // Apply weighting matrix to jacobian
  // J_w = J*W^(-1/2)
  if (!is_first_iteration)
  {
    for (std::size_t i = 0; i < jacobian.rows(); ++i)
    {
      for (std::size_t j = 0; j < jacobian.columns(); ++j)
      {
        jacobian(i,j) *= W_(j);
      }
    }
  }

  return true;
}

int IkSolverPinverse::setWeights(const JntArray & weights)
{
  weights_ = weights;
  return 0;
}

int IkSolverPinverse::setAllWeights(const double &weight)
{
  for(unsigned int i=0; i < weights_.rows(); i++)
    weights_(i) = weight;

  return 0;
}

void IkSolverPinverse::print(Eigen::MatrixXd &data) const
{
  std::cout << "------------ " << data.rows() << " rows by " << data.cols() << " cols --------------- " << std::endl;
  std::cout << "[" << std::endl;
  for (std::size_t i = 0; i < data.rows(); ++i)
  {
    std::cout << "[";
    for (std::size_t j = 0; j < data.cols(); ++j)
    {
      // Hide zeros
      /*
        if ( data(i,j) <= std::numeric_limits<double>::epsilon() )
        std::cout << boost::format("%6s") % "-";
        else
      */
      std::cout << boost::format("%6.10f") % data(i,j);

      if (j < data.cols() - 1)
        std::cout << ",";
    }
    std::cout << "]";

    // close the whole matrix
    if (i == data.rows() - 1)
      std::cout << "]";

    std::cout << std::endl;
  }
}


void IkSolverPinverse::print(Eigen::VectorXd &data) const
{
  std::cout << "========= " << data.rows() << "x Joint Array =======" << std::endl;
  for (std::size_t i = 0; i < data.rows(); ++i)
  {
    std::cout << data(i) << std::endl;
  }
  std::cout << "======================= " << std::endl;
}

void IkSolverPinverse::formatNum(double num) const
{
  if (num >= 1000)
    std::cout << boost::format("%+10s") % "inf";
  else if (num <= -1000)
    std::cout << boost::format("%+10s") % "-inf";
  else
    std::cout << boost::format("%10.4f") % num;
}

} // namespace

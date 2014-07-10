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

#include <moveit/whole_body_kinematics_plugin/kdl/ik_solver_vel_pinv_nso.hpp> // customized ik generalize pseudo inverse
#include <moveit/whole_body_kinematics_plugin/MatrixSolvers.h> // SVD solver
#include <iostream> // TODO remove
#include <boost/format.hpp>

namespace KDL
{

IkSolverVel_pinv_nso::IkSolverVel_pinv_nso(int _num_tips, int _num_joints, JntArray _joint_min, JntArray _joint_max,
  JntArray _weights, const Jacobian2d& jacobian, double _eps, int _maxiter, double _alpha, bool _verbose)
  :
  // Load the jacobian to have #joint ROWS x #tips COLS
  //  jacobian(_num_joints, _num_tips*6),
  svd(jacobian),
  U(_num_tips*6,JntArray(_num_joints)),
  S(_num_joints),
  V(_num_joints, JntArray(_num_joints)),
  H(_num_joints),
  tmp(_num_joints),
  tmp2(_num_joints-6*_num_tips),
  eps(_eps),
  maxiter(_maxiter),
  alpha(_alpha),
  num_tips(_num_tips),
  weights(_weights),
  W(_num_joints),
  // Properties of joint
  joint_min(_joint_min),
  joint_max(_joint_max),
  joint_mid(_joint_min.rows()),
  joint_constant1(_joint_min.rows()),
  joint_constant2(_joint_min.rows()),
  joint_constant3(_joint_min.rows()),
  // Debugging
  verbose(_verbose)
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
int IkSolverVel_pinv_nso::CartToJnt(const JntArray& q_in, const JntArray& xdot_in, Jacobian2d& jacobian, JntArray& qdot_out, JntArray& prev_H)
{
  // weights:
  bool use_wln = false;
  // inverse methods:
  bool use_psm = false;
  bool use_kdl = false;
  bool use_kdl2 = true;
  // null space:
  bool use_gpm = false;

  unsigned int i,j;
  double sum;

  // TODO: choose best ratio
  double sv_ratio=1e-300; // default: 1.0e-3

  // Find the Weighted Least Norm Jacobian
  if (use_wln)
    weightedLeastNorm(q_in, jacobian, prev_H);

  // Method 1: Calculate the entire pseudo inverse as found in HRP --------------------------
  if (use_psm)
  {
    // Calculate pseudo inverse
    // pinv(A) = V*S^(-1)*U^(T)
    hrp::calcPseudoInverse(jacobian.data, pinverse_, sv_ratio);

    print(pinverse_);

    // Apply pinverse to the velocity vector
    for (i = 0; i < jacobian.columns(); ++i) // row of pinverse,
    {
      sum = 0.0;
      for (j = 0; j < jacobian.rows(); ++j) // column of pinverse
      {
        sum += pinverse_(i,j) * xdot_in(j);
      }

      if (use_wln)
        qdot_out(i) = W(i) *  sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Method 2: Do SVD faster using the KDL method ---------------------------------------------
  if (use_kdl)
  {
    //Do a singular value decomposition of "jacobian" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jacobian = U*S*Vt
    svd.calculate(jacobian,U,S,V,maxiter);

    // We have to calculate qdot_out = jac_pinv*xdot_in
    // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    // qdot_out = V*S_pinv*Ut*xdot_in

    //first we calculate S_pinv*Ut*xdot_in
    for (i = 0;i < jacobian.columns();i++)
    {
      sum = 0.0;
      for (j = 0;j < jacobian.rows();j++)
      {
        sum += U[j](i) * xdot_in(j);
      }
      //If the singular value is too small (<eps), don't invert it but
      //set the inverted singular value to zero (truncated svd)
      if ( fabs(S(i)) < eps )
      {
        tmp(i) = 0.0 ;
      }
      else
      {
        tmp(i) = sum/S(i) ;
      }
    }

    //tmp is now: tmp=S_pinv*Ut*xdot_in, we still have to premultiply
    //it with V to get qdot_out
    for (i = 0;i < jacobian.columns();i++)
    {
      sum = 0.0;
      for (j = 0;j < jacobian.columns();j++)
      {
        sum += V[i](j) * tmp(j);
      }

      //Put the result in qdot_out
      if (use_wln)
        qdot_out(i) = W(i) * sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Method 3: Do SVD faster using the KDL method but directly calc pseudo invers---------------------------------------------
  if (use_kdl2)
  {
    //Do a singular value decomposition of "jacobian" with maximum
    //iterations "maxiter", put the results in "U", "S" and "V"
    //jacobian = U*S*Vt
    svd.calculate(jacobian,U,S,V,maxiter);

    if (verbose)
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

    if (verbose)
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

    if (verbose)
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

      if (use_wln)
        qdot_out(i) = W(i) *  sum;
      else
        qdot_out(i) = sum;
    }
  }

  // Gradient Projection Method using Null Space ----------------------------------------------

  if (use_gpm)
  {
    // H
    for(i = 0; i < jacobian.columns(); i++) // joints
    {
      //H(i) = 0.25 * joint_constant1(i) / (  (joint_max(i) - q_in(i)) * (q_in(i) - joint_min(i)) );
      //H(i) = ( joint_constant2(i) - q_in(i) ) / joint_constant3(i);

      // Calculate the change in joint location relative to its limits
      H(i) =
        pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) )
        /
        ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) );
    }

    bool this_verbose = false;
    if (this_verbose)
    {
      std::cout << "Identity " << std::endl;
      print(identity_);
      std::cout << "pinverse: " << std::endl;
      print(pinverse_);
      std::cout << "Jacobian: " << std::endl;
      jacobian.print();
      std::cout << "H criterion: " << std::endl;
      H.print();
      std::cout << "Alpha: " << alpha << std::endl;
    }

    // qdot += k(I - J^(+)*J)
    tmp3_ = -1 * alpha * (identity_ - pinverse_ * jacobian.data) * H.data;  // TODO is this jacobian already weighted?

    if (this_verbose)
    {
      std::cout << "QDot before null space componet:" << std::endl;
      qdot_out.print();

      std::cout << "Null space component: " << std::endl;
      print(tmp3_);
    }

    qdot_out.data += tmp3_;

  }


  return 1;
}

bool IkSolverVel_pinv_nso::weightedLeastNorm(const JntArray& q_in, Jacobian2d& jacobian, JntArray& prev_H)
{
  double gradientH;

  // Find the Weighted Least Norm Jacobian
  for (std::size_t i = 0; i < W.rows(); ++i)
  {
    // Calculate the change in joint location relative to its limits
    // | gradient H(theta) |
    gradientH = abs(
      pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) )
      /
      ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) )
    );

    // If joint is moveing away from limit do not change its weight (leave as 1)
    if (gradientH - prev_H(i) >= 0) // change in performance criterion is positive
    {
      W(i) = 1 + gradientH;
    }
    else
    {
      W(i) = 1;
    }

    // TODO: remove this safety check
    /*
      if ( isnan(gradientH) || isnan(W(i)) || isnan(-1/sqrt(W(i))))
      {
      std::cout << "Is nan! " << std::endl;
      std::cout << "  gradH:  " << gradientH << std::endl;
      std::cout << "  first:  " << pow(joint_max(i) - q_in(i), 2) * (2*q_in(i) - joint_max(i) - joint_min(i) ) << std::endl;
      std::cout << "  second: " << ( 4 * pow(joint_max(i)-q_in(i),2) * pow(q_in(i) - joint_min(i),2) ) << std::endl;
      std::cout << "  max:    " << joint_max(i) << "\n";
      std::cout << "  min:    " << joint_min(i) << std::endl;
      std::cout << "  q_in:   " << q_in(i) << "\n";
      std::cout << "  i: " << i << std::endl;
      std::cout << "  W(i): " << W(i) << std::endl;
      std::cout << "  W(i)^(-1/2): " << 1/sqrt(W(i)) << std::endl;
      q_in.print();
      std::cout << "---------------------------- " << std::endl;
      }
    */

    // Prepare the weight for multiplication by the jacobian
    // W = W^(-1/2)
    W(i) = 1/sqrt(W(i));

    // Save gradient for next iteration to find delta
    prev_H(i) = gradientH;
  }

  if (verbose || false)
  {
    std::cout << "Weighing Matrix: " << std::endl;
    W.print();
  }

  // Apply weighting matrix to jacobian
  // J_w = J*W^(-1/2)
  for (std::size_t i = 0; i < jacobian.rows(); ++i)
  {
    for (std::size_t j = 0; j < jacobian.columns(); ++j)
    {
      jacobian(i,j) *= W(j);
    }
  }

  return true;
}

int IkSolverVel_pinv_nso::setWeights(const JntArray & _weights)
{
  weights = _weights;
  return 0;
}

int IkSolverVel_pinv_nso::setAllWeights(const double &weight)
{
  for(unsigned int i=0; i < weights.rows(); i++)
    weights(i) = weight;

  return 0;
}

int IkSolverVel_pinv_nso::setAlpha(const double _alpha)
{
  alpha = _alpha;
  return 0;
}

void IkSolverVel_pinv_nso::print(Eigen::MatrixXd &data) const
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
        std::cout << boost::format("%6.3f") % data(i,j);

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


void IkSolverVel_pinv_nso::print(Eigen::VectorXd &data) const
{
  std::cout << "========= " << data.rows() << "x Joint Array =======" << std::endl;
  for (std::size_t i = 0; i < data.rows(); ++i)
  {
    std::cout << data(i) << std::endl;
  }
  std::cout << "======================= " << std::endl;
}


} // namespace




  // Now onto NULL space ==========================================================
  /*
  // Create weighted position error vector
  for(i = 0; i < jacobian.columns(); i++)
  {
  // Original Criterion:
  // A. Liegeois. Automatic supervisory control of the configuration and behavior of multibody mechnisms
  if (false)
  {
  H(i) = (1/jacobian.columns()) * weights(i)*(joint_mid(i) - q_in(i));
  }
  // Liegeois paper on GPM
  // the first one from A Weighted Least-Norm Solution Based Scheme for Avoiding Joint Limits for Redundant Joint Manipilators
  else if (false)
  {
  // Calculate H(q)
  component = ( q_in(i) - joint_mid(i) ) / ( joint_mid(i) - joint_max(i) );
  H(i) = (component*component) / jacobian.columns();
  }
  // Zghal et. all performance criterion
  // the better one from A Weighted Least-Norm Solution Based Scheme for Avoiding Joint Limits for Redundant Joint Manipilators
  else if (true)
  {
  // H(q) = 1/4 * (maxaa - min)^2 / [ (max - theta)(theta-min) ]
  H(i) = 0.25 * joint_constant(i) / (  (joint_max(i) - q_in(i)) * (q_in(i) - joint_min(i)) );
  }
  else
  {


  }
  }

  //Vtn*H
  // temp2 is a vector the length of our redudant dofs (length = num_joints - 6*num_eefs)
  // temp2 = V * H
  for (i = jacobian.rows() + 1; i < jacobian.columns(); i++)
  {
  tmp2(i-(jacobian.rows()+1)) = 0.0;
  for (j = 0; j < jacobian.columns(); j++)
  {
  tmp2(i-(jacobian.rows()+1)) += V[j](i) * H(j);
  }
  }

  // Add the velocity of the null space redudancy to our qdot_out
  // qdot_out = qdot_out + alpha*temp2
  bool show_null_space = false || verbose;
  if (show_null_space)
  std::cout << "Null space velocity: ";

  for (i = 0; i < jacobian.columns(); i++)
  {
  sum = 0.0;
  for (j = jacobian.rows() + 1; j < jacobian.columns(); j++)
  {
  sum += V[i](j)*tmp2(j);
  }

  if (show_null_space)
  std::cout << alpha*sum << "  ";

  // Apply to velocity output
  qdot_out(i) += alpha*sum;
  }
  if (show_null_space)
  std::cout << std::endl;


  // Debug
  if (verbose)
  {
  std::cout << "Joint Velocity: " << std::endl;
  for (std::size_t i = 0; i < qdot_out.rows(); ++i)
  {
  std::cout << "Joint " << i << ": " << qdot_out(i) << std::endl;
  }
  }
  */

  //return the return value of the svd decomposition
  //  return ret;

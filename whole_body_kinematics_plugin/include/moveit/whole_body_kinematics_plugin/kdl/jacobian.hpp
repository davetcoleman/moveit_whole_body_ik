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

#ifndef KDL_JACOBIAN2D_HPP
#define KDL_JACOBIAN2D_HPP

#include "frames.hpp"
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace KDL
{
// Equal is friend function, but default arguments for friends are forbidden (§8.3.6.4)
class Jacobian2d;
bool Equal2d(const Jacobian2d& a,const Jacobian2d& b,double eps=epsilon);


class Jacobian2d
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> data;
  Jacobian2d();

  explicit Jacobian2d(unsigned int nr_of_columns, unsigned int nr_of_rows = 6);
  Jacobian2d(const Jacobian2d& arg);

  ///Allocates memory for new size (can break realtime behavior)
  void resize(unsigned int newNrOfColumns, unsigned int newNrOfRows = 6);

  ///Allocates memory if size of this and argument is different
  Jacobian2d& operator=(const Jacobian2d& arg);

  bool operator ==(const Jacobian2d& arg)const;
  bool operator !=(const Jacobian2d& arg)const;

  friend bool Equal(const Jacobian2d& a,const Jacobian2d& b,double eps);


  ~Jacobian2d();

  double operator()(unsigned int i,unsigned int j)const;
  double& operator()(unsigned int i,unsigned int j);
  unsigned int rows()const;
  unsigned int columns()const;

  friend void SetToZero(Jacobian2d& jac);

  friend bool changeRefPoint(const Jacobian2d& src1, const Vector& base_AB, Jacobian2d& dest);
  friend bool changeBase(const Jacobian2d& src1, const Rotation& rot, Jacobian2d& dest);
  friend bool changeRefFrame(const Jacobian2d& src1,const Frame& frame, Jacobian2d& dest);

  Twist getColumn(unsigned int i) const;
  void setColumn(unsigned int i,const Twist& t);

  void changeRefPoint(const Vector& base_AB);
  void changeBase(const Rotation& rot);
  void changeRefFrame(const Frame& frame);

  void print() const;
}; // end class

bool changeRefPoint2d(const Jacobian2d& src1, const Vector& base_AB, Jacobian2d& dest);
bool changeBase2d(const Jacobian2d& src1, const Rotation& rot, Jacobian2d& dest);
bool changeRefFrame2d(const Jacobian2d& src1,const Frame& frame, Jacobian2d& dest);


typedef boost::shared_ptr<Jacobian2d> Jacobian2dPtr;


} // end namespace

#endif

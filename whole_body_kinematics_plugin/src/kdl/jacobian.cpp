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

#include <moveit/whole_body_kinematics_plugin/kdl/jacobian.hpp>

#include <iostream> // TODO remove
#include <boost/format.hpp>

namespace KDL
{
using namespace Eigen;

Jacobian2d::Jacobian2d()
{
}


Jacobian2d::Jacobian2d(unsigned int nr_of_columns, unsigned int nr_of_rows):
  data(nr_of_rows,nr_of_columns)
{
  //std::cout << " > 2D Jacobian created of " << nr_of_rows << " rows by " << nr_of_columns << " cols " << std::endl;
}

Jacobian2d::Jacobian2d(const Jacobian2d& arg):
  data(arg.data)
{
}

Jacobian2d& Jacobian2d::operator = (const Jacobian2d& arg)
{
  this->data=arg.data;
  return *this;
}


Jacobian2d::~Jacobian2d()
{

}

void Jacobian2d::resize(unsigned int new_nr_of_columns, unsigned int new_nr_of_rows)
{
  data.resize(new_nr_of_rows,new_nr_of_columns);
}

double Jacobian2d::operator()(unsigned int i,unsigned int j)const
{
  return data(i,j);
}

double& Jacobian2d::operator()(unsigned int i,unsigned int j)
{
  //std::cout << " > setting jacobian value at  " << i << "," << j << std::endl;
  return data(i,j);
}

unsigned int Jacobian2d::rows()const
{
  return data.rows();
}

unsigned int Jacobian2d::columns()const
{
  return data.cols();
}

void SetToZero(Jacobian2d& jac)
{
  jac.data.setZero();
}

void Jacobian2d::changeRefPoint(const Vector& base_AB){
  for(unsigned int i=0;i<data.cols();i++)
    this->setColumn(i,this->getColumn(i).RefPoint(base_AB));
}

bool changeRefPoint(const Jacobian2d& src1, const Vector& base_AB, Jacobian2d& dest)
{
  if(src1.columns()!=dest.columns())
    return false;
  for(unsigned int i=0;i<src1.columns();i++)
    dest.setColumn(i,src1.getColumn(i).RefPoint(base_AB));
  return true;
}

void Jacobian2d::changeBase(const Rotation& rot){
  for(unsigned int i=0;i<data.cols();i++)
    this->setColumn(i,rot*this->getColumn(i));;
}

bool changeBase(const Jacobian2d& src1, const Rotation& rot, Jacobian2d& dest)
{
  if(src1.columns()!=dest.columns())
    return false;
  for(unsigned int i=0;i<src1.columns();i++)
    dest.setColumn(i,rot*src1.getColumn(i));;
  return true;
}

void Jacobian2d::changeRefFrame(const Frame& frame){
  for(unsigned int i=0;i<data.cols();i++)
    this->setColumn(i,frame*this->getColumn(i));
}

bool changeRefFrame(const Jacobian2d& src1,const Frame& frame, Jacobian2d& dest)
{
  if(src1.columns()!=dest.columns())
    return false;
  for(unsigned int i=0;i<src1.columns();i++)
    dest.setColumn(i,frame*src1.getColumn(i));
  return true;
}

bool Jacobian2d::operator ==(const Jacobian2d& arg)const
{
  return Equal2d((*this),arg);
}

bool Jacobian2d::operator!=(const Jacobian2d& arg)const
{
  return !Equal2d((*this),arg);
}

bool Equal2d(const Jacobian2d& a,const Jacobian2d& b,double eps)
{
  if(a.rows()==b.rows()&&a.columns()==b.columns()){
    return a.data.isApprox(b.data,eps);
  }else
    return false;
}

Twist Jacobian2d::getColumn(unsigned int i) const{
  return Twist(Vector(data(0,i),data(1,i),data(2,i)),Vector(data(3,i),data(4,i),data(5,i)));
}

void Jacobian2d::setColumn(unsigned int i,const Twist& t){
  data.col(i).head<3>()=Eigen::Map<const Vector3d>(t.vel.data);
  data.col(i).tail<3>()=Eigen::Map<const Vector3d>(t.rot.data);
}

void Jacobian2d::print() const
{
  std::cout << "------------ " << rows() << " rows by " << columns() << " cols --------------- " << std::endl;
  std::cout << "[" << std::endl;
  for (std::size_t i = 0; i < rows(); ++i)
  {
    std::cout << "[";
    for (std::size_t j = 0; j < columns(); ++j)
    {
      // Hide zeros
      /*
      if ( data(i,j) <= std::numeric_limits<double>::epsilon() )
        std::cout << boost::format("%6s") % "-";
      else
      */
        std::cout << boost::format("%6.3f") % data(i,j);

      if (j < columns() - 1)
        std::cout << ",";
    }
    std::cout << "]";

    // close the whole matrix
    if (i == rows() - 1)
      std::cout << "]";

    std::cout << std::endl;
  }
}

}

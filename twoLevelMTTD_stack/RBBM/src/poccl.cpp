// $Id: Poccl.cpp 29890 2009-02-02 10:22:01Z tdelaet $
// Copyright (C) 2009 Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "poccl.h"
#include <cmath>
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries

#include <fstream>
#include <iostream>
#define NUMCONDARGUMENTS_RBBM 1
#define TOLERANCE 0.01 //tolerance to determine z=z*

namespace BFL
{
  using namespace MatrixWrapper;

  Poccl::Poccl(double p, double zmax)
    : ConditionalPdf<ColumnVector,ColumnVector>(2,NUMCONDARGUMENTS_RBBM)
    , _p(p)
    , _zmax(zmax)
    , _tol(TOLERANCE)
  {
    this->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM);
  }

  /// Destructor
  Poccl::~Poccl(){}

  //Clone function
  Poccl* Poccl::Clone() const
  {
      return new Poccl(*this);
  }

  void Poccl::ZstarSet(double zstar) 
  {
      _zstar = zstar;
      double u = _zstar/_zmax;
      // set _pacc = dependent on z*
      _pacc = u * _p / (1-(1-u)*_p); 
  }

  Probability
  Poccl::ProbabilityGet(const ColumnVector& meas) const
  {
    Probability prob(1.0);
    // if z is 'equal' to z* (meas(1) == _zstar)
    //cout << "z*= " << _zstar << endl;
    //cout << "z= " << meas(1) << endl;
    if ( meas(1) < _zstar + _tol && meas(1) > _zstar - _tol)
    {
        if(_zstar==0)
            prob = (double)prob ;
        else
            prob = (double)prob * (1-_pacc)/_zstar;
        //cout << "z=z* " << prob << endl;
    }
    else if(meas(1)<= _zstar ) // if z<=z*
    {
        prob = (double)prob * (1-_pacc)/_zstar/ pow( 1 - (_pacc* (1 - meas(1)/_zstar )  ),2.0);
        //cout << "z<z* " << prob << endl;
    }
    else {
        prob = (double)prob * 0.0; 
        //cout << "z>z* " << prob << endl;
        return prob; 
    } // if prob=0 the loop can be stopped immediately
    

    //cout << "(Poccl::ProbabilityGet) finished: " << prob << endl;
    return prob;
  }

  bool
  Poccl::SampleFrom (vector<Sample<ColumnVector> >& samples, const int num_samples, int method, void * args) const
  {
	cerr << "Poccl: Sampling " << method
	     << "not implemented yet!" << endl;
	return false;
  }

  bool
  Poccl::SampleFrom (Sample<ColumnVector>& sample, int method, void * args) const
  {
	cerr << "Poccl: Sampling " << method
	     << "not implemented yet!" << endl;
	return false;
  }

} // End namespace

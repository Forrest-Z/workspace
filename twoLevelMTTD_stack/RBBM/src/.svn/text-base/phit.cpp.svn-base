// $Id: phit.cpp $
// Copyright (C) 2009  Tinne De Laet <first dot last at mech dot kuleuven dot be>
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

#include "phit.h"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#include <fstream>
#include <iostream>
#define NUMCONDARGUMENTS_RBBM 1

namespace BFL
{
  using namespace MatrixWrapper;


  Phit::Phit(Gaussian& additiveNoise )
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_RBBM)
    , _zstar(0.0)
    , _measurement(1)
  {
  }

  Phit::~Phit(){}

  //Clone function
  Phit* Phit::Clone() const
  {     
      return new Phit(*this);
  }

  void Phit::ZstarSet(double zstar) 
  {
      _zstar = zstar;
  }
    
  ColumnVector Phit::ExpectedValueGet() const
  {
    _measurement = _zstar;
    return _measurement + AdditiveNoiseMuGet();
  }

  Matrix Phit::dfGet(unsigned int i) const
  {
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
  }

}//namespace BFL


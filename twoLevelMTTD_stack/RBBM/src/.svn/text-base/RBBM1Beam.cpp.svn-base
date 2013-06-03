// $Id: RBBM1Beam.cpp 29890 2009-02-02 10:22:01Z tdelaet $
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

#include "RBBM1Beam.h"
#include <cmath>
#include <bfl/pdf/gaussian.h>
#include <bfl/pdf/uniform.h>
#include "phit.h"
#include "poccl.h"
#include <fstream>
#include <iostream>

//#include <ocl/ComponentLoader.hpp>

//ORO_CREATE_COMPONENT(BFL::RBBM1Beam)

#define NUMCONDARGUMENTS_RBBM1Beam 1

namespace BFL
{
  using namespace MatrixWrapper;

  RBBM1Beam::RBBM1Beam(double p, double zmax, double sigma_m, Probability pi3, Probability pi4 )
    : ConditionalPdf<ColumnVector,ColumnVector>()
    , _mixtureRBBM1Beam()
    , _p(p)
    , _zmax(zmax)
    , _sigma_m(sigma_m)
    , _pi3(pi3)
    , _pi4(pi4)
    , _weights(4)
  {
    //cout << "RBBM1Beam constructor started " << endl;
    // has ONE conditional argument
    this->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM1Beam);
    ((ConditionalPdf<ColumnVector,ColumnVector>*)this)->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM1Beam);

    //Create mixture of Phit, Poccl, Prand and Pmax
    
    //cout << "create mixture " << endl;

    ColumnVector MeasNoise_Mu(1);
    SymmetricMatrix MeasNoise_Cov(1);
    MeasNoise_Mu = 0.0;
    MeasNoise_Cov = 0.0;
    MeasNoise_Cov(1,1) = pow(sigma_m,2.0);
    Gaussian additiveNoise(MeasNoise_Mu,MeasNoise_Cov);

    ColumnVector center(1);
    center = _zmax/2.0;
    //cout <<"center " << center << endl;
    ColumnVector width(1);
    width = _zmax;
    //cout <<"width " << width << endl;

    Phit *phit = new Phit(additiveNoise);
    //cout << "phit created " << endl;
    Poccl *poccl = new Poccl(_p,_zmax);
    //cout << "poccl created " << endl;
    Uniform *prand = new Uniform(center,width);
    //cout << "prand created " << endl;
    center = _zmax;
    width = 0.15;
    //cout << "center pmax " << center << endl;
    //cout << "width pmax " << width << endl;
    Uniform *pmax = new Uniform(center,width); // TODO: add property for 0.05?
    //cout << "pmax created " << endl;

    //cout << "mixture components created " << endl;

    _mixtureRBBM1Beam.AddComponent(*phit);
    _mixtureRBBM1Beam.AddComponent(*poccl);
    _mixtureRBBM1Beam.AddComponent(*prand);
    _mixtureRBBM1Beam.AddComponent(*pmax);
    //cout << " mixture components added " << endl;

    // to intialize set zstar to maximum range
    ZstarSet(_zmax);

    //cout << "RBBM1Beam constructor finished " << endl;
  }

  /// Destructor
  RBBM1Beam::~RBBM1Beam()
  {
    //delete _phit, poccl, prand, pmax;
  }

  //Clone function
  RBBM1Beam* RBBM1Beam::Clone() const
  {
      return new RBBM1Beam(*this);
  }

  Probability
  RBBM1Beam::ProbabilityGet(const ColumnVector& meas) const
  {
    //cout << "(RBBM1Beam::ProbabilityGet)  entered " << endl;
    //cout << "prob phit" << _mixtureRBBM1Beam.ComponentGet(0)->ProbabilityGet(meas) << endl;
    //cout << "prob poccl " << _mixtureRBBM1Beam.ComponentGet(1)->ProbabilityGet(meas) << endl;
    //cout << "prob prand " << _mixtureRBBM1Beam.ComponentGet(2)->ProbabilityGet(meas) << endl;
    //cout << "prob pmax " << _mixtureRBBM1Beam.ComponentGet(3)->ProbabilityGet(meas) << endl;
    //cout << "(RBBM1Beam:ProbabilityGet ) _mixtureRBBM1Beam.ProbabilityGet " << _mixtureRBBM1Beam.ProbabilityGet(meas) << endl;
    //cout << "(RBBM1Beam::ProbabilityGet)  finished " << endl;
    return _mixtureRBBM1Beam.ProbabilityGet(meas);
  }
    
  void
  RBBM1Beam::ZstarSet(double zstar) 
  {
      // weights depend on zstar
      double u = zstar/_zmax;
      double _pacc = u * _p / (1-(1-u)*_p);
      _weights[0] = (1.0-_pacc)*(1.0-_pi3.getValue()-_pi4.getValue());
      _weights[1] = _pacc*(1.0-_pi3.getValue()-_pi4.getValue());
      _weights[2] = _pi3; 
      _weights[3] = _pi4; 
      _mixtureRBBM1Beam.WeightsSet(_weights);
      // cout << "(RBBM1Beam::ZstarSet)  entered " << endl;
      // set Ideal measurement in Phit
      (dynamic_cast<Phit*>(_mixtureRBBM1Beam.ComponentGet(0)))->ZstarSet(zstar);
      //cout << "(RBBM1Beam::ProbabilityGet)  zstar set for component 0" << zstar << endl;
      // set Ideal measurement in Poccl
      (dynamic_cast<Poccl*>(_mixtureRBBM1Beam.ComponentGet(1)))->ZstarSet(zstar);
      //cout << "(RBBM1Beam::ProbabilityGet)  zstar set for component 1" << zstar << endl;
  }
/*
  double RBBM1Beam::ZstarGet() const
  {
    //cout << "(RBBM1Beam::ZstarGet)  entered " << endl;
    // get state under consideration
    _state = this->ConditionalPdf<ColumnVector,ColumnVector>::ConditionalArgumentGet(0);
    //cout << "state examined " << _state<< endl;
    // calculate relative position of state wrt scanner
    _relPos.resize(_state.sub(1,2).size());
    _relPos = _state.sub(1,2) - _scannerPos;
    //cout << "relative Position " << _relPos << endl;
    // calculate pole coordinates of relative position

    // calculate which beam(s) are affected by the _state
    double angle = atan2(_relPos(2),_relPos(1));
    //cout << "angle " << angle << endl;
    // TODO: add 0.5 before (int)
    int beam_affected = (int)(angle / M_PI *(_nbeams-1)) + 1 ;
    if (beam_affected == _beamNumber)
    {
        //cout << "beam_affected " << beam_affected << endl;
        double exp_meas = sqrt( pow(_relPos(1),2.0) + pow(_relPos(2),2.0) );
        if (exp_meas < _environment(_beamNumber))
            return exp_meas;
        else
            return _environment(_beamNumber);
    }
    else
    {
        return _environment(_beamNumber);
    }
    return _environment(_beamNumber);

  }
*/

  bool
  RBBM1Beam::SampleFrom (vector<Sample<ColumnVector> >& samples, const int num_samples, int method, void * args) const
  {
    return _mixtureRBBM1Beam.SampleFrom(samples, num_samples, method, args) ;
  }

  bool
  RBBM1Beam::SampleFrom (Sample<ColumnVector>& sample, int method, void * args) const
  {
    return _mixtureRBBM1Beam.SampleFrom(sample, method, args) ;
  }

  void
  RBBM1Beam::ConditionalArgumentSet(unsigned int n_argument, const ColumnVector& argument)
  {
    //cout << "(RBBM1Beam) ConditionalArgumentSet " << endl;
    assert ( n_argument < this->NumConditionalArgumentsGet());
    ConditionalPdf<ColumnVector,ColumnVector>::ConditionalArgumentSet(n_argument,argument);
    // index of conditional arguments of ConditionalPdf out of range
    // set conditional argument of phit and poccl
    ((ConditionalPdf<ColumnVector,ColumnVector>*)_mixtureRBBM1Beam.ComponentGet(0))->ConditionalArgumentSet(n_argument,argument);
    ((ConditionalPdf<ColumnVector,ColumnVector>*)_mixtureRBBM1Beam.ComponentGet(1))->ConditionalArgumentSet(n_argument,argument);
  }

  void
  RBBM1Beam::ConditionalArgumentsSet(std::vector<ColumnVector> condargs)
  {
    //cout << "(RBBM1Beam) ConditionalArgumentsSet " << endl;
    assert (condargs.size() == this->NumConditionalArgumentsGet());
    ConditionalPdf<ColumnVector,ColumnVector>::ConditionalArgumentsSet(condargs);
    // set conditional arguments of phit and poccl
    ((ConditionalPdf<ColumnVector,ColumnVector>*)_mixtureRBBM1Beam.ComponentGet(0))->ConditionalArgumentsSet(condargs);
    ((ConditionalPdf<ColumnVector,ColumnVector>*)_mixtureRBBM1Beam.ComponentGet(1))->ConditionalArgumentsSet(condargs);
  }
} // End namespace

// $Id: RBBM.cpp 29890 2009-02-02 10:22:01Z tdelaet $
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

#include "RBBM.h"
#include <cmath>
#include <fstream>
#include <iostream>

//#include <ocl/ComponentLoader.hpp>

//ORO_CREATE_COMPONENT(BFL::RBBM)

#define NUMCONDARGUMENTS_RBBM 1

namespace BFL
{
  using namespace MatrixWrapper;
  using namespace std;

  RBBM::RBBM( double p, double zmax, double sigma_m, Probability pi3, Probability pi4, ColumnVector scannerPos, string environment_file, int nbeams, double radiusState)
    : ConditionalPdf<vector<ColumnVector>,ColumnVector>()
    , _scannerPos(scannerPos)
    , _environment_file(environment_file)
    , _nbeams(nbeams)
    , _environment(_nbeams)
    , _zstars(nbeams)
    , _radiusState(radiusState)
    , _zmax(zmax)
  {
#ifndef NDEBUG
    cout << "RBBM constructor started " << endl;
#endif
    this->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM);
    ConditionalPdf<vector<ColumnVector>,ColumnVector>::NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM);

    _beamModels.resize(_nbeams);

    // create _nbeams RBBM1Beam models
    for(int i=0; i<_nbeams;i++) 
    {
        _beamModels[i] = new RBBM1Beam(p, zmax, sigma_m, pi3, pi4);
        // has ONE conditional argument
        _beamModels[i]->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM);
        ((ConditionalPdf<ColumnVector,ColumnVector>*)_beamModels[i])->NumConditionalArgumentsSet(NUMCONDARGUMENTS_RBBM);
    }   

    //Read environment from file
    ifstream indata; // indata is like cin
    double num; // variable for input value

    indata.open(_environment_file.c_str()); // opens the file
    if(!indata)  // file couldn't be opened
        cerr << "(RBBM) Error: file " << _environment_file << " could not be opened" << endl;

    indata >> num; // first one is timestamp
    indata >> num;
    int counter = 0;
    while ( !indata.eof()  && counter < _nbeams) //keep reading until end-of-file
    { 
        _environment[counter] = num;
        indata >> num; // sets EOF flag if no value found
        counter++;
    }
     indata.close();
#ifndef NDEBUG
    cout << "RBBM constructor finished " << endl;
#endif
  }

  /// Destructor
  RBBM::~RBBM()
  {
    //delete _phit, poccl, prand, pmax;
  }

  //Clone function
  RBBM* RBBM::Clone() const
  {
      return new RBBM(*this);
  }

  void RBBM::CalculateZstars() const
  {
#ifndef NDEBUG
    cout << "(RBBM::CalculateZstars)  entered " << endl;
#endif
    // set ideal measurements equal to environment
    _zstars = _environment ;

    // Calculate which beams state affects
    // get state under consideration
    _state = this->ConditionalPdf<vector<ColumnVector>,ColumnVector>::ConditionalArgumentGet(0);
    // calculate relative position of state wrt scanner
    _relPos.resize(_state.sub(1,2).size());
    _relPos = _state.sub(1,2) - _scannerPos;
    // calculate pole coordinates of relative position

    /*
    double angle = atan2(_relPos(2),_relPos(1));
    // TODO: add 0.5 before (int)
    int beam_affected = (int)(angle / M_PI *(_nbeams-1)) + 1 ;
    double exp_meas = sqrt( pow(_relPos(1),2.0) + pow(_relPos(2),2.0) );
    _zstars(beam_affected) = exp_meas;
    */

    // Calculate intersection between beams and circle around state (x,y)
    double x,y; 
    double m,m2,a,b,c,D;
    double x1,x2,y1,y2;
    double exp_meas,distsq ;
    int solution;
    m2=0.0;
    for(int i=1; i<=_nbeams;i++) 
    {
        if(_nbeams==1)
            m = 0.0;
        else
            m = tan( (i-1)*M_PI/(_nbeams-1));
        if (fabs(m)<100)
        { 
            solution = 1; // y=m x is equation of beam
        }
        else
        {
            solution = 2; // x=m2 y is equation of beam
            m2 = tan(M_PI/2- (i-1)*M_PI/(_nbeams-1) ) ;
        }
        switch (solution){
            case 1 :
                a = pow(m,2.0)+1;
                b = -2*(_relPos(1)+m*_relPos(2));
                c = pow(_relPos(1),2.0) + pow(_relPos(2),2.0) - pow(_radiusState,2.0);
                D = pow(b,2.0) - 4 * a * c;
                distsq = pow(_relPos(1),2.0) + pow(_relPos(2),2.0);
                if( (distsq>pow(_radiusState,2.0)) && (D>=0) &&  ( ( ( (m!=0) && (_relPos(1)*m)>=0)) || (i==1 && _relPos(1)>=0) || (i==_nbeams && _relPos(1)<=0)  ) ) 
//                if( (distsq>pow(_radiusState,2.0)) && (D>=0) &&  ( ( (_relPos(1)*m)>=0) || (i==1 && _relPos(1)>=0) || (i==_nbeams && _relPos(1)<=0)  ) ) 
                // there is an intersection between stateCircle and beam, and
                // intersection point is on positive beam 
                {
                    if (a==0)
                    {
                       x = -c/b; 
                       y = m*x;
                    }
                    else
                    {
                        x1 = (-b + sqrt(D))/(2*a);
                        x2 = (-b - sqrt(D))/(2*a);
                        // solution is x with smalles absvalue
                        if(fabs(x1) <= fabs(x2) )
                        {
                            x = x1;
                            y = m*x;
                        }
                        else
                        {
                            x = x2;
                            y = m*x;
                        }
                        exp_meas = sqrt( pow(x,2.0) + pow(y,2.0) );
                        if(exp_meas < _zstars(i)) // only adapt expected measurement if object is located in front of environment
                        {
                            if (exp_meas > _zmax) // only measurement if distance < measurement range
                            {
                                exp_meas=_zmax;
                            }
                            _zstars(i) = exp_meas;
                        }
                    }
                }
                break;
            case 2 :
                a = pow(m2,2.0)+1;
                b = -2*(_relPos(2)+m2*_relPos(1));
                c = pow(_relPos(1),2.0) + pow(_relPos(2),2.0) - pow(_radiusState,2.0);
                D = pow(b,2.0) - 4 * a * c;
                distsq = pow(_relPos(1),2.0) + pow(_relPos(2),2.0);
                if( (distsq>pow(_radiusState,2.0)) && (D>=0) )
                // there is an intersection between stateCircle and beam, and
                // intersection point is on positive beam 
                {
                    if (a==0)
                    {
                       y = -c/b; 
                       x = m2*y;
                    }
                    else
                    {
                        y1 = (-b + sqrt(D))/(2*a);
                        y2 = (-b - sqrt(D))/(2*a);
                        // solution is y with smalles absvalue
                        if(fabs(y1) <= fabs(y2) )
                        {
                            y = y1;
                            x = m2*y;
                        }
                        else
                        {
                            y = y2;
                            x = m2*y;
                        }
                        exp_meas = sqrt( pow(x,2.0) + pow(y,2.0) );
                        if(exp_meas < _zstars(i)) // only adapt expected measurement if object is located in front of environment
                        {
                            if (exp_meas > _zmax) // only measurement if distance < measurement range
                            {
                                exp_meas=_zmax;
                            }
                            _zstars(i) = exp_meas;
                        }
                    }
                }
                break;
            default:
                cerr << "(RBBM::CalculateZstars) Error no solution found: case wrong implemented " << endl;
        }
        
    }
  }

  Probability
  RBBM::ProbabilityGet(const vector<ColumnVector>& meas) const
  {
#ifndef NDEBUG
    cout << "(RBBM::ProbabilityGet)  entered " << endl;
#endif
    Probability prob = 1.0;
    Probability probOne = 1.0;
    Probability probLoc = 1.0;
    ColumnVector meas1(1);
    int beam;
    int beamMiddle;
    // Calculate the ideal measurements
    this->CalculateZstars();
    // independent beams assumed
    for(int i=0; i<meas[1].size();i++) 
    {
        meas1 = meas[0](i+1);
        // middle beam
        beamMiddle = (int)(meas[1](i+1) / M_PI *(double)(_nbeams-1)) ;
        
        double weight = 0.0;
        probOne = 0.0;

        beam = beamMiddle;
        _beamModels[beam]->ZstarSet(_zstars(beam+1));
        probLoc=_beamModels[beam]->ProbabilityGet(meas1);
        weight +=  0.6;
        probOne=probOne+ 0.6*probLoc;

        beam = beamMiddle-3;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.01;
            probOne=probOne+ 0.01*probLoc;
        }
        beam = beamMiddle-2;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.04;
            probOne=probOne+ 0.04*probLoc;
        }
        beam = beamMiddle-1;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.15;
            probOne=probOne+  0.15*probLoc;
        }
        beam = beamMiddle+1;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.15;
            probOne=probOne+ 0.15*probLoc;
        }
        beam = beamMiddle+2;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.04;
            probOne=probOne+  0.04*probLoc;
        }
        beam = beamMiddle+3;
        if(beam> -1 && beam<_nbeams)
        {
            _beamModels[beam]->ZstarSet(_zstars(beam+1));
            probLoc=_beamModels[beam]->ProbabilityGet(meas1);
            weight +=  0.01;
            probOne=probOne+  0.01*probLoc;
        }

        prob = prob * Probability((double)probOne / weight);
//        cout<< "prob  measurement " << i << " " <<  (double)probOne / weight<< endl;
    }   
#ifndef NDEBUG
    cout<< "prob " << (double)prob << endl;
    cout << "(RBBM::ProbabilityGet)  finished " << endl;
#endif
    return prob;
  }


  bool
  RBBM::SampleFrom (vector<Sample<vector<ColumnVector> > >& samples, const int num_samples, int method, void * args) const
  {
        cout << "ERROR: not implemented yet" << endl;
        return false;
  }

  bool
  RBBM::SampleFrom (Sample<vector<ColumnVector> >& sample, int method, void * args) const
  {
        cout << "ERROR: not implemented yet" << endl;
        return false;
  }

  void
  RBBM::ConditionalArgumentSet(unsigned int n_argument, const ColumnVector& argument)
  {
#ifndef NDEBUG
    cout << "(RBBM::ConditionalArgumentSet)  entered " << endl;
#endif
    assert ( n_argument < this->NumConditionalArgumentsGet());
    // index of conditional arguments of ConditionalPdf out of range
    ConditionalPdf<vector<ColumnVector>,ColumnVector>::ConditionalArgumentSet(n_argument, argument);
    for(int i=0; i<_nbeams;i++) 
    {
        // independent beams assumed
        _beamModels[i]->ConditionalArgumentSet(n_argument,argument);
    }   
#ifndef NDEBUG
    cout << "(RBBM::ConditionalArgumentSet)  finished " << endl;
#endif
  }

  void
  RBBM::ConditionalArgumentsSet(std::vector<ColumnVector> condargs)
  {
#ifndef NDEBUG
    cout << "(RBBM::ConditionalArgumentsSet)  entered " << endl;
#endif
    assert (condargs.size() == this->NumConditionalArgumentsGet());
    ConditionalPdf<vector<ColumnVector>,ColumnVector>::ConditionalArgumentsSet(condargs);
    for(int i=0; i<_nbeams;i++) 
    {
        // independent beams assumed
        _beamModels[i]->ConditionalArgumentsSet(condargs);
    }   
#ifndef NDEBUG
   cout << "(RBBM::ConditionalArgumentsSet)  finished " << endl;
#endif
  }
} // End namespace

// $Id: RBBMCartesian_radius.h $
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

#ifndef __RBBMCARTESIAN_RADIUS_
#define __RBBMCARTESIAN_RADIUS_

#include <bfl/pdf/conditionalpdf.h>
#include "RBBM1Beam.h"

namespace BFL
{
  /// Non Linear Conditional Pdf for RBBM with unknown radius (Rigorously
  //Bayesian Beam Model ) when the measurement is cartesian
  /* see: 
    @Article{           delaet-2008jair,
      author          = {De~Laet, Tinne and De~Schutter, Joris and Bruyninckx,
                        Herman},
      title           = {A Rigoroulsy {B}ayesian Beam Model and an Adaptive Full Scan Model for Range Finders in Dynamic Environments},
      journal         = jair,
      year            = {2008},
      volume          = {33},
      pages           = {179--222},
      keywords        = {Bayes theory, estimation, range finder beam model,
                         variational Bayes}
    }
    */
  // ASSUMPTIONS for implementation: fixed laser position in fixed environment
  /** This class inherits only from ConditionalPdf<ColumnVector,
      ColumnVector>.
  */
  class RBBMCartesian_radius : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
    {
    //friend Probability Phit::ProbabilityGet(); 
    //friend Probability Poccl::ProbabilityGet(); 
    public:
      /// Constructor
      /**
	 @param p
	 @param zmax
	 @param sigma_m
	 @param pi3
	 @param pi4
	 @param scannerPos
	 @param environment_file
     @param nbeams  The number of beams
      */
      RBBMCartesian_radius( double p, double zmax, double sigma_m, Probability pi3, Probability pi4,ColumnVector scannerPos, string environment_file, int nbeams = 181 );

      // Default Copy constructor will do

      /// Destructor
      virtual ~RBBMCartesian_radius();

      ///Clone function
      virtual RBBMCartesian_radius* Clone() const;

      // implemented virtuals!
      virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& meas) const;
      virtual bool SampleFrom (Sample<MatrixWrapper::ColumnVector >& sample, int method=DEFAULT, void * args=NULL) const;
      virtual bool SampleFrom (std::vector<Sample<MatrixWrapper::ColumnVector > >& samples, const int num_samples,
			       int method=DEFAULT, void * args=NULL) const;
      virtual void ConditionalArgumentsSet(std::vector<ColumnVector> ConditionalArguments);
      virtual void ConditionalArgumentSet(unsigned int n_argument, const ColumnVector& argument);
    protected:
      /// Calculate the ideal measurements for all beams
      void CalculateZstars() const;
      // _mixtureRBBMCartesian_radius
      vector<RBBM1Beam* >  _beamModels;
      // variables to avoid allocation on the heap during sampling
      /// _scannerPos gives the FIXED scanner position in the world
      MatrixWrapper::ColumnVector   _scannerPos; 
      // _environment_file filename of file containing measurement of laserscan
      // of environment (no targets present)
      const std::string _environment_file;
      // _nbeams number of beams in the laserscan
      const int _nbeams;
      // _environment environment measurements 
      MatrixWrapper::ColumnVector _environment;
      // _state current first conditional argument = state;
      mutable MatrixWrapper::ColumnVector   _state; 
      // _relPos relative position of current state wrt scanner
      mutable MatrixWrapper::ColumnVector   _relPos; 
      // _zstars vector of ideal measurements
      mutable MatrixWrapper::ColumnVector _zstars;
      // the radius of the state variable
      mutable double _radiusState;
      // the maximum range of the sensor of the state variable
      const double _zmax;
    };

} // End namespace BFL

#endif // __RBBMCARTESIAN_RADIUS_


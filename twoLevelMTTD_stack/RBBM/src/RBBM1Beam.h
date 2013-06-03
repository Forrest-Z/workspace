// $Id: RBBM1Beam.h $
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

#ifndef __RBBM1Beam__
#define __RBBM1Beam__

#include <bfl/pdf/mixture.h>
#include <bfl/pdf/conditionalpdf.h>

namespace BFL
{
  /// Non Linear Conditional Pdf for RBBM1Beam (Rigorously
  //Bayesian Beam Model )
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
  class RBBM1Beam : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
    {
    //friend Probability Phit::ProbabilityGet(); 
    //friend Probability Poccl::ProbabilityGet(); 
    public:
      /// Constructor
      /**
	 @param pacc 
      */
      RBBM1Beam(double p, double zmax, double sigma_m, Probability pi3, Probability pi4);

      // Default Copy constructor will do

      /// Destructor
      virtual ~RBBM1Beam();

      ///Clone function
      virtual RBBM1Beam* Clone() const;

      //double ZstarGet() const;
      void ZstarSet(double zstar) ;

      // implemented virtuals!
      virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& meas) const;
      virtual bool SampleFrom (Sample<MatrixWrapper::ColumnVector>& sample, int method=DEFAULT, void * args=NULL) const;
      virtual bool SampleFrom (std::vector<Sample<MatrixWrapper::ColumnVector> >& samples, const int num_samples,
			       int method=DEFAULT, void * args=NULL) const;
      virtual void ConditionalArgumentsSet(std::vector<ColumnVector> ConditionalArguments);
      virtual void ConditionalArgumentSet(unsigned int n_argument, const ColumnVector& argument);
    protected:
      // _mixtureRBBM1Beam
      BFL::Mixture<MatrixWrapper::ColumnVector> _mixtureRBBM1Beam;
      // _p pparameter of Poccl model
      const double _p;
      // _zmax
      const double _zmax;
      // _sigma_m
      const double _sigma_m;
      // _pi3
      const Probability _pi3;
      // _pi4
      const Probability _pi4;
      // weights or components  
      vector<Probability> _weights;
    };

} // End namespace BFL

#endif // __CONDITIONAL_GAUSSIAN__


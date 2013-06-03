// $Id: poccl.h $
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

#ifndef __POCCL__
#define __POCCL__

#include <bfl/pdf/conditionalpdf.h>

namespace BFL
{
  /// Non Linear Conditional Pdf for poccl component of the Rigorously
  //Bayesian Beam Model 
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
  class Poccl : public ConditionalPdf<ColumnVector, MatrixWrapper::ColumnVector>
    {
    public:
      /// Constructor
      /**
	 @param p probability of unknown object
	 @param zmax the maximum range measurement
      */
      Poccl(double p, double zmax);

      // Default Copy constructor will do

      /// Destructor
      virtual ~Poccl();

      ///Clone function
      virtual Poccl* Clone() const;

      void ZstarSet(double zstar);

      // implemented virtuals!
      virtual Probability ProbabilityGet(const ColumnVector& meas) const;
      virtual bool SampleFrom (Sample<ColumnVector>& sample, int method=DEFAULT, void * args=NULL) const;
      virtual bool SampleFrom (std::vector<Sample<ColumnVector> >& samples, const int num_samples,
			       int method=DEFAULT, void * args=NULL) const;

    protected:
      // _p p parameter of Poccl model
      const double _p;
      /// _zmax the maximum range measurement
      const double _zmax;
      // _pacc pacc parameter of Poccl model
      double _pacc;
      // _tol used to determine tolerance on z=z*
      const double _tol;
      /// The ideal measurement along the beam
      double _zstar;

    };

} // End namespace BFL

#endif // __CONDITIONAL_GAUSSIAN__


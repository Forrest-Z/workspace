// $Id: phit.h $
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


#ifndef __PHIT__
#define __PHIT__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

// ASSUMPTION: independent beams
namespace BFL
{
  /// Non Linear Conditional Gaussian for phit component of the Rigorously
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
  /**
     - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
     Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
     - Covariance is independent of the ConditionalArguments, and is
     the covariance of the Noise pdf
  */
  class Phit : public AnalyticConditionalGaussianAdditiveNoise
    {
    protected:
      /// The ideal measurement along the beam
      double _zstar;
      // _measurement expected measurement
      mutable MatrixWrapper::ColumnVector   _measurement; 

    public:
      /// Constructor
      /**
	  @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
      Phit( Gaussian& additiveNoise );

      /// Destructor
      virtual ~Phit();

      ///Clone function
      virtual Phit* Clone() const;

      void ZstarSet(double zstar);

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;
      //virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& input) const;
    };

} // End namespace BFL

#endif //

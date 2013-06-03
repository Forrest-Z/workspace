// $Id: dataAssociationFilterGaussianMurty.h 
// Copyright (C) 2008 Tinne De Laet <first dot last at mech dot kuleuven dot be>
//  
 /***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU General Public                   *
 *   License as published by the Free Software Foundation;                 *
 *   version 2 of the License.                                             *
 *                                                                         *
 *   As a special exception, you may use this file as part of a free       *
 *   software library without restriction.  Specifically, if other files   *
 *   instantiate templates or use macros or inline functions from this     *
 *   file, or you compile this file and link it with other files to        *
 *   produce an executable, this file does not by itself cause the         *
 *   resulting executable to be covered by the GNU General Public          *
 *   License.  This exception does not however invalidate any other        *
 *   reasons why the executable file might be covered by the GNU General   *
 *   Public License.                                                       *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public             *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/ 

#ifndef __DATA_ASSOCIATION_FILTER_GAUSSIAN_MURTY__
#define __DATA_ASSOCIATION_FILTER_GAUSSIAN_MURTY__

#include "dataAssociationFilterMurty.h"
#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/pdf.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/filter/filter.h>
#include <vector>
#include <map>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/dataAssociation/dataAssociationFilterGaussian.h>

namespace BFL
{
  using namespace std;
  using namespace MatrixWrapper;

  /// Class for data association filters in which the underlying filters are Kalman filters
  /** This is a class that defines the data association filters
      These filters are all related to an underlying set of filters
  */
  class DataAssociationFilterGaussianMurty : public DataAssociationFilterMurty<ColumnVector, ColumnVector>
    {

    protected:
      /// vector of pointers to posteriors of filters
      std::vector< Gaussian* > _posts; 
      /// iterator for vector of pointers to posteriors of filters
      std::vector<Gaussian* >::iterator _iter_posts; 
      /// bool indicated of measurement probabilities were already calculated or not
      bool                  _measProbsCalculated; 
      /// The measurement probabilities
      vector<ColumnVector>  _measMeasProbsCalculated;
      /// Helper variable for Gaussian during measurement update 
      Gaussian              _py;
      /// Helper variable for state during measurement update 
      ColumnVector          _x;
      /// Helper variable for mean during measurement update 
      ColumnVector          _Mu_new;
      /// Helper variable for covariance during measurement update 
      Matrix                _Sigma_temp;
      /// Helper variable for covariance during measurement update 
      Matrix                _Sigma_temp2;
      /// Helper variable for covariance during measurement update 
      Matrix                _Sigma_temp_par;
      /// Helper variable for covariance during measurement update 
      SymmetricMatrix       _Sigma_new;
      /// Map to help memory allocation during measurement update (prevents re-allocation of already availabe memory)
      std::map<unsigned int, MeasUpdateVariables> _mapMeasUpdateVariables;
      /// Iterator for map to help memory allocation during measurement update (prevents re-allocation of already availabe memory)
      std::map<unsigned int, MeasUpdateVariables>::iterator _mapMeasUpdateVariables_it;


      /// Implementation of Update
      // calls update on the underlying filters
      /** @param sysmodel pointer to the used system model
	  @param u input param for proposal density
	  @param measmodel pointer to the used measurementmodel
	  @param z measurement param for proposal density
	  @param s sensor param for proposal density
      */
      bool UpdateInternal(SystemModel<ColumnVector>* const sysmodel,
				  const ColumnVector& u,
				  MeasurementModel<ColumnVector,ColumnVector>* const measmodel,
				  const vector<ColumnVector>& z,
				  const ColumnVector& s);

      /// Function to allocate necessary memory during measurement update
      /** @param meas_dimensions the vector with dimensions of the measurements to allocate    
      */
      void AllocateMeasModel(const vector<unsigned int>& meas_dimensions);
      /// Function to allocate necessary memory during measurement update
      /** @param meas_dimensions the dimension of the measurements to allocate    
      */
      void AllocateMeasModel(const unsigned int& meas_dimension);

    public:
      /// Constructor
      /** @pre you created the prior
	  @param filters pointer to the prior vector of filters
	  @param gamma probability of a false positive (measurement is not caused by any of the objects)
	  @param treshold threshold on the probability of a association from which the association is taken into account 
	  @param maxFilters the maximum number of filters (for allocation)
	  @param maxFeatures the maximum number of features (for allocation)
	  @param onlyUseCentroidMeasurement only use centroid measurement or not 
      */
      DataAssociationFilterGaussianMurty(vector<Filter<ColumnVector,ColumnVector> *> filters, double gamma = 0.0, double treshold= 0.0, int maxFilters =20, int maxFeatures=20, bool onlyUseCentroidMeasurement=false);

      /// copy constructor
      //DataAssociationFilterGaussianMurty (const DataAssociationFilterMurtyGaussianMurty & filters);

      /// destructor
      virtual ~DataAssociationFilterGaussianMurty();

      /// Get the probabilities of the measurements for each of the filters 
      /** @param sysmodel pointer to the used system model
	  @param measmodel pointer to the used measurementmodel
	  @param z vector of measurements 
	  @param s sensor param for proposal density
      @return Matrix containing the probability of each of the measurements given each of the filters
      */
      MatrixWrapper::Matrix GetMeasProbs(MeasurementModel<ColumnVector,ColumnVector>* const measmodel , const vector<ColumnVector>& z, const ColumnVector& s);

      /// Add a filter
      /** @param filter pointer to the filter that will be added
      */
      void AddFilter(Filter<ColumnVector,ColumnVector>* filter);

      /// Remove a filter
      /** @param index index of the filter that will be removed. This index
         should be between 0 and NumFitlersGet()-1, else false is returned
          @return bool indicating of the removal was succesfull or not
      */
      bool RemoveFilter(int index);

    };


} // End namespace BFL

#endif // __DATA_ASSOCIATION_FILTER_GAUSSIAN__

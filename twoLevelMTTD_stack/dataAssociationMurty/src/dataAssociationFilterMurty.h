// $Id: dataAssociationFilterMurty.h 
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

#ifndef __DATA_ASSOCIATION_FILTERMURTY__
#define __DATA_ASSOCIATION_FILTERMURTY__

#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/pdf/pdf.h>
#include <bfl/filter/filter.h>
#include <vector>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <lap/hungarian.h>
#include <lap/LAPJV.h>
#include <lap/CompleteCostMatrix.h>
#include <lap/LAPJVShared.h>
#include <lap/LAPJVTemplate.h>
#include <lap/LapGenerator.h>   
#include <linearAssignmentProblem.h>   

#include "featureWithProb.h"   

namespace BFL
{
  using namespace std;
  using namespace MatrixWrapper;

  /// Class for data association filters
  /** This is a clas that defines the data association filters
      These filters are all related to an underlying set of filters and are used
      data with the correct filters.
  */
  template <typename StateVar, typename MeasVar> class DataAssociationFilterMurty
    {
    protected:
      /// Only use centroid measurments or not
      bool _onlyUseCentroidMeasurement;

      /// Vector of pointers to filters
      std::vector< Filter<StateVar,MeasVar>* > _filters; 

      /// iterator for vector of pointers to filters
      typename std::vector< Filter<StateVar,MeasVar>* >::iterator _iter_filters; 

      /// Vector of filter numbers
      std::vector< int > _filterID; 

      /// iterator for vector of filterID
      std::vector< int >::iterator _iter_filterID; 
    
      int _timestep;

      /// Probability of false positive (feature measurement without object)
      double _gamma;

      /// Treshold of probability to take into account associations
      double _treshold;

      /// The effective number of filters
      int _numFilters;

      /// vector of pointers to posteriors of filters
      std::vector< Pdf<StateVar>* > _posts; 

      /// iterator for vector of pointers to posteriors of filters
      typename std::vector<Pdf<StateVar>* >::iterator _iter_posts; 

      /// vector containing prob(z|object) 
      Matrix _probs;
      //vector<vector<double> > _probs;
    
      /// helper variable for probabilities
      Matrix _probs_rest;

      /// maximum number of filters
      int _maxFilters ; 
      /// maximum number of features
      int _maxFeatures; 
      /// counter for the number of different filters used so far to obtain filterID
      int _filterCounter; 

      /// The probabilities that the data is associated with each of the filters
      vector<vector<double> > _association_probs;  

      /// LinearAssignmentProblem
      LinearAssignmentProblem* _assignmentProblem;

      /// The number of clusters
      int _numClusters;

      /// Vector containing the IDs of the responsable cluster for each of the measurements
      vector<int> _responsibleCluster;

      /// Boolean indicating if all necessary variables are set before running update
      bool _variablesUpdateInternalSet;
    
      /// Implementation of Update
      // calls update on the underlying filters
      /** @param sysmodel pointer to the used system model
	  @param u input param for proposal density
	  @param measmodel pointer to the used measurementmodel
	  @param z measurement param for proposal density
	  @param s sensor param for proposal density
      */
      virtual bool UpdateInternal(SystemModel<StateVar>* const sysmodel,
				  const StateVar& u,
				  MeasurementModel<MeasVar,StateVar>* const measmodel,
				  const vector<MeasVar>& z,
				  const StateVar& s)=0;

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
      DataAssociationFilterMurty (vector<Filter<StateVar,MeasVar> *> filters, double gamma = 0.0, double treshold= 0.0, int maxFilters =20, int maxFeatures=180, bool onlyUseCentroidMeasurement = false);

      /// copy constructor
      /** @pre you have another DataAssociationFilterMurty 
	  @param DataAssociationFilterMurty you want to copy 
      */
      DataAssociationFilterMurty (const DataAssociationFilterMurty<StateVar,MeasVar> & filters);

      /// Does the filter only uses centroid measurements or not 
      /** @return boolean indicating of only centroid measurements are used 
      */
      bool UseOnlyCentroidMeasurements(){return _onlyUseCentroidMeasurement;};

      /// Get the maximum number of filters
      /** @return the maximum number of filters
      */
      int GetMaxFilters(){return _maxFilters;};

      /// Get the maximum number of features
      /** @return the maximum number of features
      */
      int GetMaxFeatures(){return _maxFeatures;};

      /// destructor
      virtual ~DataAssociationFilterMurty();

      /// Get the probabilities of the measurements for each of the filters 
      /** @param sysmodel pointer to the used system model
	  @param measmodel pointer to the used measurementmodel
	  @param z vector of measurements 
	  @param s sensor param for proposal density
      @return Matrix containing the probability of each of the measurements given each of the filters
      */
      virtual MatrixWrapper::Matrix GetMeasProbs(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s)=0;

      /// Get the filters of the DataAssociationFilterMurty
      /** @return vector of Filters underlying the dataAssociationFilter
      */
      vector< Filter<StateVar,MeasVar> > FiltersGet();

      /// Get Posterior density
      /** Get the current Posterior density
	  @return a vector of pointers to the current posteriors
      */
      vector< Pdf<StateVar> * > PostGet();  

      /// Get current time
      /** Get the current time of the filter
	  @return the current timestep
      */
      int TimeStepGet() const;

      /// Get the filtersIDs of the DataAssociationFilterMurty
      /** @return vector of ints indicating the ID (identification number) of the filter 
      */
      vector< int > FilterIDGet();

      /// Get the filterCounter of the DataAssociationFilterMurty
      /** @return the filter counter (maximum ID of the filters so far created)
      */
      int FilterCounterGet();

      /// Get the probability of false positive
      /** @return the probability of a false positive 
      */
      double GammaGet();

      /// Get the treshold on the probability from which associations are taken into account
      /** @return the treshold of the probability from which associations are  taken into account 
      */
      double TresholdGet();

      /// Reset Filters
      /** @param filters new vector of filters for the DataAssociationFilterMurty
      */
      void Reset(vector< Filter<StateVar,MeasVar> *> filters);

      /// Get number of Filters
      /** @return the current number of filters 
      */
      int NumFiltersGet();

      /// Add a filter
      /** @param filter pointer to the filter that will be added
      */
      void AddFilter(Filter<StateVar,MeasVar>* filter);

      /// Remove a filter
      /** @param index index of the filter that will be removed. This index
         should be between 0 and NumFitlersGet()-1, else false is returned
          @return bool indicating of the removal was succesfull or not
      */
      bool RemoveFilter(int index);

/*
      // Find the most unexplained measurment
      int GiveMostUnexplainedMeasurement(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s, double tresholdAddFilter);

*/
      // Sort the measurements: most unexplained first
      /** 
	  @param measmodel pointer to the used measurementmodel
	  @param z vector of measurements 
	  @param s sensor param for proposal density
      @return bool indicating if sorting failed or not 
      */
      bool SortMeasurements(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s, vector<FeatureWithProb<MVar> >& vectorFeaturesWithProb);

      // Get the association probabilities of the measurements with each
      // probable association
      /** 
	  @param measmodel pointer to the used measurementmodel
	  @param z vector of measurements 
	  @param s sensor param for proposal density
      @return vector _association_probs 
      */
      void GetAssociationProbs(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s);

      /// Full Update (system with inputs/sensing params)
      /** @param sysmodel pointer to the system model to use for update
	  @param u input to the system
	  @param measmodel pointer to the measurement model to use for update
	  @param z measurement
	  @param s "sensing parameter"
       */
      bool Update(SystemModel<StateVar>* const sysmodel,
			  const StateVar& u,
			  MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z,
			  const StateVar& s);

      /// Full Update (system without inputs, with sensing params)
      /** @param sysmodel pointer to the system model to use for
	  update
	  @param measmodel pointer to the measurement model to use for
	  update
	  @param z measurement
	  @param s "sensing parameter"
       */
      bool Update(SystemModel<StateVar>* const sysmodel,
			  MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z,
			  const StateVar& s);

      /// Full Update (system without inputs/sensing params)
      /** @param sysmodel pointer to the system model to use for
	  update
	  @param measmodel pointer to the measurement model to use for
	  update
	  @param z measurement
       */
      bool Update(SystemModel<StateVar>* const sysmodel,
			  MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z);

      /// Full Update (system with inputs, without sensing params)
      /** @param sysmodel pointer to the system model to use for update
	  @param u input to the system
	  @param measmodel pointer to the measurement model to use for
	  update
	  @param z measurement
       */
      bool Update(SystemModel<StateVar>* const sysmodel,
			  const StateVar& u,
			  MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z);

      /// System Update (system with inputs)
      /** @param sysmodel pointer to the system model to use for update
	  @param u input to the system
       */
      bool Update(SystemModel<StateVar>* const sysmodel,
			  const StateVar& u);

      /// System Update (system without inputs)
      /** @param sysmodel pointer to the system model to use for update
       */
      bool Update(SystemModel<StateVar>* const sysmodel);

      /// Measurement Update (system with "sensing params")
      /** @param measmodel pointer to the measurement model to use for
	  update 
	  @param z measurement
	  @param s "sensing parameter"
       */
      bool Update(MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z,
			  const StateVar& s);

      /// Measurement Update (system without "sensing params")
      /** @param measmodel pointer to the measurement model to use for
	  update
	  @param z measurement
       */
      bool Update(MeasurementModel<MeasVar,StateVar>* const measmodel,
			  const vector<MeasVar>& z);

      /// Function to set some variables before update is run responsibleCluster and numClusters
      /** @param responsibleCluster vector containing IDs of responsible cluster for all measurements
        @param numClusters the number of clusters
      */
      bool SetVariablesUpdateInternal(vector<int> responsibleCluster, int numClusters);

      /// Function to get a copy of te association probabilities (vector<vector<double > > where vector[i][j] is the probability that measurement[j] is associated with _filterID[j])
      /** @return association_probs vector containing the association probabilities (vector<vector<double > > where vector[i][j] is the probability that measurement[j] is associated with _filterID[j])
      */
      vector<vector<double> > GetCopyDataAssociationProbs();


    };


////////////// IMPLEMENTATION/////////////////////////////////////

    #define StateVar SVar
    #define MeasVar MVar

    // Constructor
    template<typename SVar, typename MVar> 
    DataAssociationFilterMurty<SVar,MVar>::DataAssociationFilterMurty(vector<Filter<SVar,MVar> * > filters, double gamma, double treshold, int maxFilters, int maxFeatures, bool onlyUseCentroidMeasurement)
      : _timestep(0)
        , _gamma(gamma)
        , _treshold(treshold)
        , _maxFilters(maxFilters)
        , _maxFeatures(maxFeatures)
        , _onlyUseCentroidMeasurement(onlyUseCentroidMeasurement)
    {
      _numFilters = filters.size();

      _probs.resize(_maxFilters,_maxFeatures);
      _probs = 0.0;
      //_probs.assign(_maxFilters, vector<double>(_maxFeatures,0.0));
      _posts.resize(_maxFilters);
      _filters.resize(_maxFilters);
      _filterID.resize(_maxFilters);

      typename std::vector< Filter<StateVar,MeasVar>* >::iterator iter_filters_local; 
      _iter_posts = _posts.begin();
      _iter_filters = _filters.begin();
      int counter = 0;
      for(iter_filters_local = filters.begin() ; iter_filters_local != filters.end() ; iter_filters_local++) 
      {
        *_iter_filters =(*iter_filters_local); 
        *_iter_posts = (*_iter_filters)->PostGet();
        _filterID[counter] = counter+1;
        _iter_posts++;
        _iter_filters++;
        counter ++;
      }
      _filterCounter = this->NumFiltersGet();
      _iter_posts = _posts.begin();
      _iter_filters = _filters.begin(); 

      _probs_rest.resize(_maxFilters,_maxFeatures);
      _probs_rest = 0.0;

      _association_probs.resize(_maxFeatures);
      _association_probs.assign(_maxFeatures,vector<double>(_maxFilters));
      _assignmentProblem= new LinearAssignmentProblem();
    }
     
    template<typename SVar, typename MVar> 
    DataAssociationFilterMurty<SVar,MVar>::~DataAssociationFilterMurty()
    {
        // throw away filters
        for(_iter_filters = _filters.begin() ; _iter_filters != _filters.end() ; _iter_filters++) 
        {
          delete *_iter_filters;
        }
        delete _assignmentProblem;
    }

    template<typename SVar, typename MVar> 
    DataAssociationFilterMurty<SVar,MVar>::DataAssociationFilterMurty(const DataAssociationFilterMurty& filters)
        : _timestep(0)
    {   
        _onlyUseCentroidMeasurement = filters.UseOnlyCentroidMeasurements();
        _maxFilters = filters.GetMaxFilters();
        _maxFeatures = filters.GetMaxFeatures();
        _probs.resize(_maxFilters,_maxFeatures);
        _probs = 0.0;
        //_probs.assign(_maxFilters, vector<double>(_maxFeatures,0.0));
        _posts.resize(_maxFilters);
        _filters.resize(_maxFilters);
        _filterID.resize(_maxFilters);

        _numFilters = filters.NumFiltersGet();
        _gamma = filters.GammaGet();
        _treshold = filters.TresholdGet();
        _filters = filters.FiltersGet();
        _filterID = filters.FilterIDGet();
        _filterCounter = filters.FilterCounterGet();
        _iter_posts = _posts.begin();
        int counter = 0;
        for(_iter_filters = _filters.begin() ; _iter_filters != _filters.end() ; _iter_filters++) 
        {
          *_iter_posts = (*_iter_filters)->PostGet();
          _iter_posts++;
        }
        _iter_posts = _posts.begin();
        _iter_filters = _filters.begin(); 
        _probs.resize(_maxFilters,_maxFeatures);
        _probs = 0.0;
        //_probs.assign(_maxFilters, vector<double>(_maxFeatures,0.0));


        _probs_rest.resize(_maxFilters,_maxFeatures);
        _probs_rest = 0.0;

        _association_probs.resize(_maxFeatures);
        _association_probs.assign(_maxFeatures,vector<double>(_maxFilters));
        
        _assignmentProblem= new LinearAssignmentProblem();
    }

    template<typename SVar, typename MVar> vector<Filter<SVar,MVar> >
    DataAssociationFilterMurty<SVar,MVar>:: FiltersGet()
    {
        return _filters;
    }

    template<typename SVar, typename MVar> vector<int>
    DataAssociationFilterMurty<SVar,MVar>:: FilterIDGet()
    {
        return _filterID;
    }

    template<typename SVar, typename MVar> int
    DataAssociationFilterMurty<SVar,MVar>:: FilterCounterGet()
    {
        return _filterCounter;
    }
    
    template<typename SVar, typename MVar> double 
    DataAssociationFilterMurty<SVar,MVar>:: GammaGet()
    {
        return _gamma;
    }

    template<typename SVar, typename MVar> double 
    DataAssociationFilterMurty<SVar,MVar>:: TresholdGet()
    {
        return _treshold;
    }

    template<typename SVar, typename MVar>  void
    DataAssociationFilterMurty<SVar,MVar>::Reset(vector< Filter<SVar,MeasVar> *> filters)
    {
      _numFilters = filters.size();
      typename std::vector< Filter<StateVar,MeasVar>* >::iterator iter_filters_local; 
      _iter_posts = _posts.begin();
      _iter_filters = _filters.begin();
      int counter = 0;
      for(iter_filters_local = filters.begin() ; iter_filters_local != filters.end() ; iter_filters_local++) 
      {
        *_iter_filters =(*iter_filters_local); 
        *_iter_posts = (*_iter_filters)->PostGet();
        _filterID[counter] = counter+1;
        _iter_posts++;
        _iter_filters++;
        counter ++ ;
      }
      _filterCounter = this->NumFiltersGet();
      _iter_posts = _posts.begin();
      _iter_filters = _filters.begin(); 
    }

    template<typename SVar, typename MVar>  int
    DataAssociationFilterMurty<SVar,MVar>::NumFiltersGet()
    {
      return _numFilters;
    }

    template<typename SVar, typename MVar>  void
    DataAssociationFilterMurty<SVar,MVar>::AddFilter(Filter<SVar,MVar>* filter)
    {
        _filters[_numFilters] = filter;
        _posts[_numFilters] = filter->PostGet();
        _filterID[_numFilters] = _filterCounter + 1;
        _filterCounter ++;
        _numFilters++;
    }
    
    template<typename SVar, typename MVar>  bool
    DataAssociationFilterMurty<SVar,MVar>::RemoveFilter(int index)
    {
        if(index< 0 || index > _numFilters-1)
        {
            return false;
        }
        else
        {
            // get iterator to the index'th element
            _iter_filters = _filters.begin();
            _iter_posts = _posts.begin();
            _iter_filterID = _filterID.begin();
            for(int i = 0 ; i<index ; i++ )
            {
                _iter_posts++;
                _iter_filters++;
                _iter_filterID++;
            }
            // delete the filter itself!
            Filter<SVar,MVar>* temp_pointer = *_iter_filters;
            _posts.erase(_iter_posts);
            _filters.erase(_iter_filters);
            _filterID.erase(_iter_filterID);
            delete temp_pointer;
            // TODO: nothing more efficient to shift elements of vector forward?
            _posts.resize(_maxFilters);
            _filters.resize(_maxFilters);
            _numFilters--;
            return true;
        }
    }

    template<typename SVar, typename MVar> int 
    DataAssociationFilterMurty<SVar,MVar>::TimeStepGet() const
    {
      return _timestep;
    }
    
    //template<typename SVar, typename MVar> bool
    //DataAssociationFilterMurty<SVar,MVar>::UpdateInternal(SystemModel<SVar>* const sysmodel,
    //			  const SVar& u,
    //			  MeasurementModel<MVar,SVar>* const measmodel,
    //			  const MVar& z,
    //			  const SVar& s)
    //{
    //  bool return_bool = true;
    //  _iter_posts = _posts.begin();
    //  for(_iter_filters = _filters.begin() ; _iter_filters != _filters.end() ; _iter_filters++) 
    //  {
    //    return_bool = (return_bool && _iter_filters->Update(sysmodel,u,measmodel,z,s) ); 
    //    *_iter_posts = (*_iter_filters)->PostGet();
    //    _iter_posts++;
    //  }
    //  return return_bool;
    //}
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel,
    			  const SVar& u,
    			  MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z,
    			  const SVar& s)
    {
      return this->UpdateInternal(sysmodel,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel,
    			  MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z,
    			  const SVar& s)
    {
      SVar u;
      return this->UpdateInternal(sysmodel,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel,
    			  MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z)
    {
      SVar s; SVar u;
      return this->UpdateInternal(sysmodel,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel,
    			  const SVar& u,
    			  MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z)
    {
      SVar s;
      return this->UpdateInternal(sysmodel,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel,
    			  const SVar& u)
    {
      SVar s; vector<MVar> z(1);
      return this->UpdateInternal(sysmodel,u,NULL,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(SystemModel<SVar>* const sysmodel)
    {
      SVar s;  SVar u; vector<MVar> z;
      return this->UpdateInternal(sysmodel,u,NULL,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::Update(MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z,
    			  const SVar& s)
    {
      SVar u; 
      return this->UpdateInternal(NULL,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>:: Update(MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z)
    {
      SVar u; SVar s;
      return this->UpdateInternal(NULL,u,measmodel,z,s);
    }
    
    template<typename SVar, typename MVar> vector< Pdf<SVar> * >
    DataAssociationFilterMurty<SVar,MVar>::PostGet()
    {
      return _posts;
    }

    template<typename SVar, typename MVar> bool 
    DataAssociationFilterMurty<SVar,MVar>::SortMeasurements(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s, vector<FeatureWithProb<MVar> >& vectorFeaturesWithProb)
    {
        vector<int> responsibleCluster(z.size());
        // all clusters are responsible for theirself
        for(int feature=0 ; feature< z.size(); feature++)
        {   
            responsibleCluster[feature]=feature;
        }
        this->SetVariablesUpdateInternal(responsibleCluster, z.size());
        this->GetAssociationProbs(measmodel,z,s);
        //fills in _association_probs
        vectorFeaturesWithProb.resize(z.size());
        for(int feature=0 ; feature< z.size(); feature++)
        {   
            vectorFeaturesWithProb[feature].feature=z[feature];
            vectorFeaturesWithProb[feature].prob=0.0;
            for(int object=0 ; object< this->NumFiltersGet(); object++)
            {   
                vectorFeaturesWithProb[feature].prob += _association_probs[feature][object];
            }
        }
        sort(vectorFeaturesWithProb.begin(),vectorFeaturesWithProb.end());
#ifndef NDEBUG
        cout << "result after SortMeasurements: " << endl;
        for(int feature=0 ; feature< z.size(); feature++)
        {   
            cout << "vectorFeaturesWithProb[" << feature<<"].feature=" << vectorFeaturesWithProb[feature].feature << endl;
            cout << "vectorFeaturesWithProb[" << feature<<"].prob=" << vectorFeaturesWithProb[feature].prob << endl;;
        }
#endif
        return true;
    }

    //TODO: replace by:
    // update with weighting between k best different possible associations

    // update with only taking into account most probable association
    template<typename SVar, typename MVar> void 
    DataAssociationFilterMurty<SVar,MVar>::GetAssociationProbs(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar> &z, const StateVar& s)
    {
#ifndef NDEBUG
      cout << "(DataAssociationFilterMurty) entered GetAssociationProbs " << std::endl;
#endif
      int numMeasurements = z.size();
      int number_objects = this->NumFiltersGet();

#ifndef NDEBUG
      cout << "Get meas probabilities " << std::endl;
#endif
      this->GetMeasProbs(measmodel , z , s);
      // stores result in _probs
#ifndef NDEBUG
      cout << "Got meas probabilities " << std::endl;
#endif
#ifndef NDEBUG
      cout << "Create costMatrix " << std::endl;
#endif
      // Create costMatrix
      int numTargets = number_objects;
      if(_onlyUseCentroidMeasurement)
      {
            _numClusters = numMeasurements;
      }
      const unsigned int dimR = numTargets + 2* _numClusters;
      const unsigned int dimC = _numClusters + 2* numTargets;
      unsigned int dim = max(dimR, dimC);
#ifndef NDEBUG
      cout << "numMeasurements " << numMeasurements << std::endl;
      cout << "number clusters " << _numClusters << std::endl;
      cout << "number targets " << numTargets << std::endl;
      cout << "dimension costMatrix " << dim << std::endl;
#endif
      CompleteCostMatrix costmatrix(dim, dim);
      costmatrix.setConstant(ASSIGNMENTSOLVER_INFINITY);
#ifndef NDEBUG
      cout << "created costMatrix and constant set " << std::endl;
#endif
      // create LinearAssignmentProblem: will create full costmatrix from  matchProbabilities 
      vector<vector<double> > matchProbabilities(numTargets); 
      matchProbabilities.assign(numTargets,vector<double>(_numClusters,0.0));  
#ifndef NDEBUG
      std::cout << "created matchProbabilities " << std::endl;
#endif
        if(_onlyUseCentroidMeasurement)
        {
            for(int target =0; target< numTargets; target++)
            {
              for(int measurement =0; measurement< numMeasurements; measurement++)
              {
#ifndef NDEBUG
                  std::cout << "matchProbabilities[ " << target << "]["<< measurement <<"]= " << std::endl;
#endif      
                      matchProbabilities[target][measurement] =   _probs(target+1,measurement+1);
#ifndef NDEBUG
                      std::cout << matchProbabilities[target][measurement]<< std::endl;
#endif      
              }
            }
        }
        else // use all measurements
        {
            for(int target =0; target< numTargets; target++)
            {
              for(int measurement =0; measurement< numMeasurements; measurement++)
              {
#ifndef NDEBUG
                  std::cout << "matchProbabilities[ " << target << "]["<< _responsibleCluster[measurement] <<"]= " << std::endl;
#endif      
                  if(_responsibleCluster[measurement]>-1)
                  {
                      matchProbabilities[target][_responsibleCluster[measurement]] +=   _probs(target+1,measurement+1);
#ifndef NDEBUG
                      std::cout << matchProbabilities[target][_responsibleCluster[measurement]]<< std::endl;
#endif      
                  }
              }
            }
        }
#ifndef NDEBUG
      std::cout << "filled in matchProbabilities " << std::endl;
#endif
    _assignmentProblem->createGlobalCostMatrix(numTargets,_numClusters,matchProbabilities,costmatrix); 
#ifndef NDEBUG
      std::cout << "created global cost matrix" << std::endl;
#endif
      // Solve assignmentproblem
#ifndef NDEBUG
      std::cout << "dim " << dim << std::endl;
#endif
      LAPJVShared lapjv(dim, &costmatrix);
#ifndef NDEBUG
      std::cout << "created LAPJVShared" << std::endl;
      std::cout <<"cost matrix " << std::endl;
      lapjv.printCostMatrix();
#endif
      lapjv.run(); 
#ifndef NDEBUG
      std::cout <<"lapjv finished " << std::endl;
#endif
      const int* colSolution = lapjv.getColumnSolution();
#ifndef NDEBUG
      std::cout <<"lapjv got Column solution " << std::endl;
      for (unsigned int col = 0; col < dim; ++col){
         // check validity
         if (costmatrix(colSolution[col], col) < ASSIGNMENTSOLVER_INFINITY) {
             std::cout << "solution for feature " << col << ": object " << colSolution[col] << " with cost " << costmatrix(colSolution[col], col)<<std::endl;
             // valid assignment
             std::cout << "col " << col << " results in valid assignment" << std::endl;
         }
      }
      //std::cout <<"_probs " << _probs  << std::endl;
      std::cout <<"cost matrix " << std::endl;
      lapjv.printCostMatrix();
      std::cout <<"assignment matrix " << std::endl;
      lapjv.printAssignmentMatrix();
#endif
      /*
      int k = best_association;
      */
      _association_probs.assign(_maxFeatures,vector<double>(_maxFilters,0.0));
      #ifndef NDEBUG
            std::cout <<"_association_probs.size()" << _association_probs.size() << std::endl;
      #endif

      //check if feature j is assigned to object i in best association
      int object = -1;
      for (int l = 0 ; l< _numClusters  ; l++) // see with which object each feature is associated
      {
         #ifndef NDEBUG
               std::cout <<"see with wich object cluster " << l << " is associated" << std::endl;
         #endif
          // feature l is associated with object _associations[k][l]
          if (colSolution[l]  >= number_objects ) // false alarm
          {
              //prod_prob =  _gamma; 
          }
          else
          {
              // feature l is associated with object according to association k
              object = colSolution[l];
            #ifndef NDEBUG
                  std::cout <<"feature " << l << " is associated with object " << object << std::endl;
         #endif
             for (int i=0; i< numMeasurements ; i++)
             {
                #ifndef NDEBUG
                      std::cout <<"measurement " << i <<  std::endl;
                #endif
                if(_onlyUseCentroidMeasurement)
                {
                    _association_probs[i][object] = _probs(object+1,i+1);
                }
                else
                {
                    if (_responsibleCluster[i]==l)
                    {
                    #ifndef NDEBUG
                          std::cout <<"measurement " << i << " belongs to cluster " << l<< std::endl;
                    #endif
                        _association_probs[i][object] = _probs(object+1,i+1);
                    }
                }
                #ifndef NDEBUG
                      std::cout <<"_association_probs[" << i << "][" << object << "]= " <<std::endl;
                      std::cout << _association_probs[i][object]<< std::endl;
                #endif
             }
          }
      }
#ifndef NDEBUG
            std::cout <<"_association_probs.size()" << _association_probs.size() << std::endl;
            std::cout <<"_association_probs[0].size()" << _association_probs[0].size() << std::endl;
#endif
#ifndef NDEBUG
      std::cout << "(DataAssociationFilterMurty) left GetAssociationProbs " << std::endl;
#endif
    }

    template<typename SVar, typename MVar> bool
    DataAssociationFilterMurty<SVar,MVar>::SetVariablesUpdateInternal(vector<int> responsibleCluster, int numClusters)
    {
        _responsibleCluster = responsibleCluster;
        _numClusters = numClusters;
        _variablesUpdateInternalSet = true;
        return true;
    }


    template<typename SVar, typename MVar> vector<vector<double> >
    DataAssociationFilterMurty<SVar,MVar>::GetCopyDataAssociationProbs()
    {
        return _association_probs;
    }


} // End namespace BFL

#endif // __DATA_ASSOCIATION_FILTERMURTY__

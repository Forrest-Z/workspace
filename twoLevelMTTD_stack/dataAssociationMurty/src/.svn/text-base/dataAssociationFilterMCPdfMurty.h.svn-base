// $Id: dataAssociationFilterMCPdfMurty.h 
// Copyright (C) 2010 Tinne De Laet <first dot last at mech dot kuleuven dot be>
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

#ifndef __DATA_ASSOCIATION_FILTER_MCPDF_MURTY__ 
#define __DATA_ASSOCIATION_FILTER_MCPDF_MURTY__ 

#include "dataAssociationFilterMurty.h"
#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/pdf.h>
#include <bfl/pdf/mcpdf.h>
#include <bfl/filter/filter.h>
#include <vector>
#include <map>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/dataAssociation/dataAssociationFilterMCPdf.h>

namespace BFL
{
  using namespace std;
  using namespace MatrixWrapper;

  /// Class for data association filters in which the underlying filters are Kalman filters
  /** This is a class that defines the data association filters
      These filters are all related to an underlying set of filters
  */
    template <typename StateVar, typename MeasVar> 
    class DataAssociationFilterMCPdfMurty : public DataAssociationFilterMurty<StateVar, MeasVar>
    {

    protected:
      /// vector of pointers to posteriors of filters
      std::vector< MCPdf<StateVar>* > _posts; 
      /// iterator for vector of pointers to posteriors of filters
      typename std::vector<MCPdf<StateVar>* >::iterator _iter_posts; 
      /// bool indicated of measurement probabilities were already calculated or not
      bool                  _measProbsCalculated; 
      /// vector containing prob(z|particle) for all particles
      vector<Matrix> _prob_particles;
      /// The measurement probabilities
      vector<MeasVar> _measMeasProbsCalculated;
      /// The maximum number of particles
      int _maxParticles; 

      /// Implementation of Update
      // calls update on the underlying filters
      /** @param sysmodel pointer to the used system model
	  @param u input param for proposal density
	  @param measmodel pointer to the used measurementmodel
	  @param z measurement param for proposal density
	  @param s sensor param for proposal density
      */
      bool UpdateInternal(SystemModel<StateVar>* const sysmodel,
				  const StateVar& u,
				  MeasurementModel<MeasVar,StateVar>* const measmodel,
				  const vector<MeasVar>& z,
				  const StateVar& s);


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
      DataAssociationFilterMCPdfMurty (vector<Filter<StateVar,MeasVar> *> filters, double gamma = 0.0, double treshold= 0.0, int maxFilters =20, int maxFeatures=20, bool onlyUseCentroidMeasurement=false) ;

      /// copy constructor
      //DataAssociationFilterMCPdfMurty (const DataAssociationFilterMurtyGaussianMurty & filters);

      /// destructor
      virtual ~DataAssociationFilterMCPdfMurty();

      /// Get the probabilities of the measurements for each of the filters 
      /** @param sysmodel pointer to the used system model
	  @param measmodel pointer to the used measurementmodel
	  @param z vector of measurements 
	  @param s sensor param for proposal density
      @return Matrix containing the probability of each of the measurements given each of the filters
      */
      MatrixWrapper::Matrix GetMeasProbs(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar>& z, const StateVar& s);

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

    };

////////////// IMPLEMENTATION/////////////////////////////////////

    #define StateVar SVar
    #define MeasVar MVar

    // Constructor
    template<typename SVar, typename MVar> 
    DataAssociationFilterMCPdfMurty<SVar,MVar>::DataAssociationFilterMCPdfMurty(vector<Filter<SVar,MVar> * > filters, double gamma, double treshold, int maxFilters, int maxFeatures, bool onlyUseCentroidMeasurement)
      :DataAssociationFilterMurty<SVar,MVar>(filters,gamma,treshold,maxFilters,maxFeatures,onlyUseCentroidMeasurement)
    {
#ifndef NDEBUG
        cout<< "DataAssociationFilterMCPdfMurty Constructor" << endl;
#endif
        //typename std::vector<Filter<SVar,MVar>* >::iterator iter_filters; 
        _posts.resize(this->_maxFilters);
        _iter_posts = _posts.begin();
        for(this->_iter_filters = filters.begin() ; this->_iter_filters != filters.end() ; this->_iter_filters++) 
        {
          (*_iter_posts) = (MCPdf<SVar>*)((*this->_iter_filters)->PostGet());
          _iter_posts++;
        }
        _iter_posts = _posts.begin();
        //if(this->NumFiltersGet()==0)
        //{
        //}
        //else
        //{
        //  int dimension = (*_iter_posts)->DimensionGet();
        //  _x.resize(dimension);
        //  _Mu_new.resize(dimension);
        //  _Sigma_temp.resize(dimension,dimension);
        //  _Sigma_temp2.resize(dimension,dimension);
        //  _Sigma_temp_par.resize(dimension,dimension);
        //  _Sigma_new.resize(dimension);
        //}
        _maxParticles = 5000; //maximum number of features
        _prob_particles.resize(this->_maxFilters);
        _prob_particles.assign(this->_maxFilters,Matrix(_maxParticles,this->_maxFeatures));
    }

    template<typename SVar, typename MVar> 
    DataAssociationFilterMCPdfMurty<SVar,MVar>::~DataAssociationFilterMCPdfMurty()
    {}
    

    template<typename SVar, typename MVar> 
    void
    DataAssociationFilterMCPdfMurty<SVar,MVar>::AddFilter(Filter<SVar,MVar>* filter)
    {
#ifndef NDEBUG
        cout << "entered AddFilter" << endl;
#endif
        DataAssociationFilterMurty<SVar,MVar>::AddFilter(filter);
#ifndef NDEBUG
        cout << "baseclass add called" << endl;
#endif
        _posts[this->NumFiltersGet()-1] = (MCPdf<SVar>*)filter->PostGet();
#ifndef NDEBUG
        cout << "posts set" << endl;
#endif
        _measProbsCalculated = false;
    }

    template<typename SVar, typename MVar>  bool
    DataAssociationFilterMCPdfMurty<SVar,MVar>::RemoveFilter(int index)
    {
        DataAssociationFilterMurty<SVar,MVar>::RemoveFilter(index);
        int size = this->NumFiltersGet();
        if(index< 0 || index > size-1)
        {
            return false;
        }
        else
        {
            // get iterator to the index'th element
            _iter_posts = _posts.begin();
            for(int i = 0 ; i<index ; i++ )
            {
                _iter_posts++;
            }
            _posts.erase(_iter_posts);
            _posts.resize(this->_maxFilters);
            return true;
        }
        _measProbsCalculated = false;
    }

    template<typename SVar, typename MVar> MatrixWrapper::Matrix 
    DataAssociationFilterMCPdfMurty<SVar,MVar>::GetMeasProbs(MeasurementModel<MeasVar,StateVar>* const measmodel , const vector<MeasVar>& z, const StateVar& s)
    {
#ifndef NDEBUG
      cout << "measProbsCalculated " << _measProbsCalculated << endl;
#endif
      bool measSame = true;
      if (! _measProbsCalculated)
      {
            // test if measurements same as before
            measSame = (measSame && (_measMeasProbsCalculated.size() == z.size()));
            for(int i = 0 ; i < _measMeasProbsCalculated.size(); i++)
                measSame = (measSame && (_measMeasProbsCalculated[i] == z[i]));
      }
#ifndef NDEBUG
      cout << "measSame " << measSame << endl;
#endif
      if(!(_measProbsCalculated && measSame )) 
      {
        // probabilities not calculated yet
#ifndef NDEBUG
        cout << "probabilies not calculated yet" << endl;
#endif

#ifndef NDEBUG
        cout << "enter GetMeasProbs" << endl;
#endif
        int number_features = z.size();
        int number_objects = this->NumFiltersGet();
#ifndef NDEBUG
        cout << "number objects " << number_objects << endl;
        cout << "number features " << number_features << endl;
        cout << "size posts " << _posts.size() << endl;
#endif
        int num_particles = 0;
        vector< WeightedSample< SVar > > los;
    
        double inter = 0.0;

        Probability prob;

        this->_iter_filters = this->_filters.begin();
        _iter_posts = _posts.begin();
#ifndef NDEBUG
        cout << "this->NumFiltersGet()" << this->NumFiltersGet() << endl;
#endif
        for(int object = 1 ; object != this->NumFiltersGet() + 1 ; object++) 
        {
//#ifndef NDEBUG
//          cout << "object " << object << endl;
//#endif
          num_particles =  (*_iter_posts)->NumSamplesGet();
//#ifndef NDEBUG
//              cout << "number particles  " << num_particles << endl;
//#endif
          los = (*_iter_posts)->ListOfSamplesGet() ;
          // loop over the different filters (i.e. different objects)
          //
          for (int feature = 1 ; feature<=number_features ; feature ++ )
          {
              inter = 0.0;
//#ifndef NDEBUG
//              cout << "feature " << z[feature-1] << endl;
//#endif
              for (int teller_part = 0; teller_part < num_particles; teller_part++)
              {
//#ifndef NDEBUG
//                  cout << "particle " << los[teller_part].ValueGet()<< endl;
//                  cout << "weight " << los[teller_part].WeightGet()<< endl;
//#endif
                  if (measmodel->SystemWithoutSensorParams())
                  {
//#ifndef NDEBUG
//                        cout << "without sensor params" << endl;
//                        cout << "measurement " << z[feature-1]<< endl;
//                        //cout << "measurement simulation " << measmodel->Simulate(los[teller_part].ValueGet() ) << endl;
//                        //cout << "measurement pdf covariance " << measmodel->MeasurementPdfGet()->CovarianceGet() << endl;
//                        //cout << "measurement pdf expected value " << measmodel->MeasurementPdfGet()->ExpectedValueGet() << endl;
//#endif
                        prob =  measmodel->ProbabilityGet(z[feature-1], los[teller_part].ValueGet());
//#ifndef NDEBUG
//                        cout << "prob " << (double)prob << endl;
//#endif
                  }
                  else
                        prob =  measmodel->ProbabilityGet(z[feature-1], los[teller_part].ValueGet(), s);
                  _prob_particles[object-1](teller_part+1,feature) = prob;
//#ifndef NDEBUG
//                  cout << "prob " << (double)prob << endl;
//#endif
                  inter = inter + (double)prob * los[teller_part].WeightGet();
              }
//#ifndef NDEBUG
//              cout << "inter " << inter << endl;
//#endif
              this->_probs(object,feature) = inter ;
//#ifndef NDEBUG
//              cout << "prob " << this->_probs(object,feature) << endl;
//#endif
          }
#ifndef NDEBUG
          //cout << "prob_particles " << _prob_particles[object-1] << endl;
#endif
          _iter_posts++;
          this->_iter_filters++;
        }
        _measProbsCalculated = true; 
        _measMeasProbsCalculated = z;
      }
      else
      {
#ifndef NDEBUG
        cout << "probabilies were already calculated" << endl;
#endif
      }
      //return this->_probs.sub(1,number_objects,1,number_features);
#ifndef NDEBUG
      cout << "PROBS " << this->_probs << endl;
#endif
      return this->_probs;
    }

    template<typename SVar, typename MVar> bool
    DataAssociationFilterMCPdfMurty<SVar,MVar>::UpdateInternal(SystemModel<SVar>* const sysmodel,
    			  const SVar& u,
    			  MeasurementModel<MVar,SVar>* const measmodel,
    			  const vector<MVar>& z,
    			  const SVar& s)
    {
#ifndef NDEBUG
      cout << "DataAssociationFilterMCPdfMurty:: entering update internal" << endl;
#endif
      bool return_bool = true;

      if(sysmodel!=NULL)
      {
         // system update
         _iter_posts = _posts.begin();
         this->_iter_filters = this->_filters.begin();
         for( int object = 0 ; object < this->NumFiltersGet() ; object++)
         {
           return_bool = (return_bool && (*this->_iter_filters)->Update(sysmodel,u) ); 
           *_iter_posts = (MCPdf<SVar>*)( (*this->_iter_filters)->PostGet() );
           _iter_posts++;
           this->_iter_filters++;
         }
      }

      if(measmodel!=NULL)
      {
        if(!this->_onlyUseCentroidMeasurement && !this->_variablesUpdateInternalSet)
        {
#ifndef NDEBUG
            cout << "DataAssociationFilterMCPdfMurty:: the necessary variables are not set before running measUpdate" << endl;
#endif
            return false;
        }
#ifndef NDEBUG
        cout << "DataAssociationFilterMCPdfMurty:: entering meas update" << endl;
        cout << "DataAssociationFilterMCPdfMurty:: get association probs" << endl;
#endif
        this->GetAssociationProbs(measmodel , z, s);
        // writes _association_probs
#ifndef NDEBUG
        cout << "DataAssociationFilterMCPdfMurty:: got association probs" << endl;
#endif
        
        double sum_association_probs = 0.0;
        _iter_posts = _posts.begin();
        this->_iter_filters = this->_filters.begin();

        double temp =0.0;
        for( int object = 0 ; object < this->NumFiltersGet() ; object++)
        {
#ifndef NDEBUG
          cout << "DataAssociationFilterMCPdfMurty:: object " << object << endl;
          cout << "DataAssociationFilterMCPdfMurty:: expected value before meas update " <<((*this->_iter_filters)->PostGet() )->ExpectedValueGet() << endl;
#endif
          vector<WeightedSample<SVar> > particles = (*_iter_posts)->ListOfSamplesGet();
          for (int particle = 0; particle < (*_iter_posts)->NumSamplesGet() ; particle++)
          {
            //cout << "object " << object << endl;
             temp = 0.0;
             for(int feature = 0; feature < z.size(); feature ++)
             {
//#ifndef NDEBUG
//                  cout << "DataAssociationFilterMCPdfMurty:: _association_probs[ " << feature << "][" << object  << "]= " << this->_association_probs[feature][object]<< endl;
//                  cout << "DataAssociationFilterMCPdfMurty:: _prob_particles[ " << object << "](" << particle+1  << ", " << feature+1 <<" )=" << _prob_particles[object](particle+1,feature+1)<< endl;
//#endif
                  // only select measurements that have a minimum probability
                  if (this->_association_probs[feature][object]>5.0e-6)
                  {
                      temp = temp + this->_association_probs[feature][object] * _prob_particles[object](particle+1,feature+1) ;
                      //temp = temp + this->_association_probs[feature][object];
                      //temp = temp +  _prob_particles[object](particle+1,feature+1) ;
                      //temp = temp + this->_association_probs[feature][object] * particles[particle].WeightGet();
                  }
             }
            if (temp>5.0e-9)
                particles[particle].WeightSet(temp * particles[particle].WeightGet());
            else
                particles[particle].WeightSet(5.0e-9);
//#ifndef NDEBUG
//            cout << "new weight particle " << particles[particle].ValueGet() << " is : " << particles[particle].WeightGet() << endl;
//#endif
          }
          return_bool = ( return_bool && ((MCPdf<SVar>*)( (*(this->_iter_filters))->PostGet()))->ListOfSamplesUpdate(particles) );
          // listOfSamplesSet takes care of normalization
          *_iter_posts = (MCPdf<SVar>*) ((*this->_iter_filters)->PostGet() );
#ifndef NDEBUG
          cout << "DataAssociationFilterMCPdfMurty:: expected value after meas update " <<((*this->_iter_filters)->PostGet() )->ExpectedValueGet() << endl;
#endif
          // sets posterior too
          _iter_posts++;
          this->_iter_filters++;
        }
#ifndef NDEBUG
      this->_iter_filters = this->_filters.begin();
      for( int object = 0 ; object < this->NumFiltersGet() ; object++)
      {
            cout << "DataAssociationFilterMCPdfMurty:: object " << object << endl;
            cout << "DataAssociationFilterMCPdfMurty:: expected value after meas update " <<((*this->_iter_filters)->PostGet() )->ExpectedValueGet() << endl;
            this->_iter_filters++;
      }
#endif
      }
#ifndef NDEBUG
    cout << "DataAssociationFilterMCPdfMurty:: leaving update internal" << endl;
#endif
    _measProbsCalculated = false; 
    this->_variablesUpdateInternalSet = false;
    return return_bool;
    }



} // End namespace BFL

#endif // __DATA_ASSOCIATION_FILTER_MCPDF_MURTY__

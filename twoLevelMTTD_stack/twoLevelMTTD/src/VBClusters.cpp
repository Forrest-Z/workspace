#include "VBClusters.hpp"
#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::VBClusters)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     VBClusters::VBClusters(std::string name)
         : TaskContext(name,PreOperational),
            _dimension("dimension", "the dimension of the space in which VBI is done"),
            _treshold_new_cluster("treshold_new_cluster", "the treshold of the association probability from which a new cluster is created"),
            _treshold_effective_cluster("treshold_effective_cluster", "the treshold for the effective number of measurements from which a cluster is effective"),
            _max_iter("max_iter","The maximum number of iterations"),
            _max_iterFirst("max_iterFirst","The maximum number of iterations for the first iteration"),
            _maxNumberClusters("maxNumberClusters","The maximum number of clusters (reporter)"),
            _maxNumberComponents("maxNumberComponents","The maximum number of components (memory allocation)"),
            _maxNumberMeasurements("maxNumberMeasurements","The maximum number of measurements (memory allocation)"),
            _betaPrior("betaPrior","The beta parameter for VB clusters"),
            _nuPrior("nuPrior","Th nu parameter for VB clusters"),
            _alphaPrior("alphaPrior","The alpha parameter for VB clusters"),
            _WPrior("WPrior","The W parameter for VB clusters"),
            _SigmaPrior("SigmaPrior","The Sigma  parameter for VB clusters"),
            _betaNewCluster("betaNewCluster","The beta parameter for VB clusters"),
            _nuNewCluster("nuNewCluster","Th nu parameter for VB clusters"),
            _alphaNewCluster("alphaNewCluster","The alpha parameter for VB clusters"),
            _WNewCluster("WNewCluster","The W parameter for VB clusters"),
            _SigmaNewCluster("SigmaNewCluster","The Sigma  parameter for VB clusters"),
            _findClusters("findClusters", &VBClusters::findClusters, this),
            _setInitial("setInitial", &VBClusters::setInitial, this),
            _setPrior("setPrior", &VBClusters::setPrior, this),
            _estimatePort("estimateClusters"),
            _stateClustersPort("stateClusters"),
            _covarianceClustersPort("covarianceClusters"),
            _numClustersPort("numClusters"),
            _responsibleClusterPort("responsibleCluster"),
            _measurementsObjects("measurementsObjects"),
            _numMeasurementsObjects("numMeasurementsObjects"),
            _priorVBClustersParametersPort("priorVBClustersParameters"),
            _numPriorClustersPort("numPriorClustersPort"),
            _first_time(true)
     {
        // Check if all initialisation was ok:
        assert( _dimension.ready() );
        assert( _treshold_new_cluster.ready() );
        assert( _treshold_effective_cluster.ready() );
        assert( _max_iter.ready() );
        assert( _maxNumberClusters.ready() );
        assert( _maxNumberComponents.ready() );
        assert( _maxNumberMeasurements.ready() );
        assert( _betaPrior.ready() );
        assert( _nuPrior.ready() );
        assert( _alphaPrior.ready() );
        assert( _WPrior.ready() );
        assert( _SigmaPrior.ready() );
        assert( _betaNewCluster.ready() );
        assert( _nuNewCluster.ready() );
        assert( _alphaNewCluster.ready() );
        assert( _WNewCluster.ready() );
        assert( _SigmaNewCluster.ready() );
        assert( _findClusters.ready() );
        assert( _setInitial.ready() );
        assert( _setPrior.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_dimension);
        this->properties()->addProperty(&_treshold_new_cluster);
        this->properties()->addProperty(&_treshold_effective_cluster);
        this->properties()->addProperty(&_max_iter);
        this->properties()->addProperty(&_max_iterFirst);
        this->properties()->addProperty(&_maxNumberClusters);
        this->properties()->addProperty(&_maxNumberComponents);
        this->properties()->addProperty(&_maxNumberMeasurements);
        this->properties()->addProperty(&_betaPrior);
        this->properties()->addProperty(&_alphaPrior);
        this->properties()->addProperty(&_nuPrior);
        this->properties()->addProperty(&_WPrior);
        this->properties()->addProperty(&_SigmaPrior);
        this->properties()->addProperty(&_betaNewCluster);
        this->properties()->addProperty(&_alphaNewCluster);
        this->properties()->addProperty(&_nuNewCluster);
        this->properties()->addProperty(&_WNewCluster);
        this->properties()->addProperty(&_SigmaNewCluster);

        this->methods()->addMethod(&_findClusters, "Find the clusters in the data");
        this->methods()->addMethod(&_setInitial, "Set the intial values","initial struct"," ");
        this->methods()->addMethod(&_setPrior, "Set the prior values","prior struct"," ");

        this->ports()->addPort(&_estimatePort);
        this->ports()->addPort(&_stateClustersPort);
        this->ports()->addPort(&_covarianceClustersPort);
        this->ports()->addPort(&_measurementsObjects);
        this->ports()->addPort(&_numMeasurementsObjects);
        this->ports()->addPort(&_priorVBClustersParametersPort);
        this->ports()->addPort(&_numClustersPort);
        this->ports()->addPort(&_responsibleClusterPort);
        this->ports()->addPort(&_priorVBClustersParametersPort);
        this->ports()->addPort(&_numPriorClustersPort);

     }

    VBClusters::~VBClusters()
    {
    }

    bool VBClusters::configureHook()
    {
        _meas_local.resize(_dimension.value());
        _meas_local = 0.0;
    
        _stateClusters.resize(_maxNumberClusters.value());
        _stateClusters.assign(_maxNumberClusters.value(),ColumnVector(_dimension.value()));
        _covarianceClusters.resize(_maxNumberClusters.value());
        _covarianceClusters.assign(_maxNumberClusters.value(),SymmetricMatrix(_dimension.value()));

        _responsibleCluster.assign(_maxNumberMeasurements.value(),-1);

        _Xbar.resize(_maxNumberComponents.value());
        _Xbar.assign(_maxNumberComponents.value(),ColumnVector(_dimension.value()));
        _S.resize(_maxNumberComponents.value());
        _S.assign(_maxNumberComponents.value(),Matrix(_dimension.value(),_dimension.value()));

        _D.resize(_dimension.value(),_dimension.value());
        _Winv.resize(_dimension.value(),_dimension.value());
        _w.resize(_dimension.value());
        _U.resize(_dimension.value(),_dimension.value());
        _V.resize(_dimension.value(),_dimension.value());
        _temp.resize(_dimension.value(),_dimension.value());

        //allocate memory
        _Rho.resize(_maxNumberComponents.value());
        _sumR.resize(_maxNumberMeasurements.value());
        _sumC.resize(_maxNumberComponents.value());
        _R.assign(_maxNumberMeasurements.value(),ColumnVector(_maxNumberComponents.value()));

        _respons.allocate(_maxNumberMeasurements.value(),_maxNumberComponents.value());
        _new_clusters.allocate(_maxNumberComponents.value(),0,_dimension.value(), _betaNewCluster.value(), _nuNewCluster.value(), _alphaNewCluster.value(), _WNewCluster.value() ,_SigmaNewCluster.value());
        _new_prior.allocate(_maxNumberComponents.value(), 0,_dimension.value(),_betaPrior.value(), _nuPrior.value(), _alphaPrior.value(), _WPrior.value() , _SigmaPrior.value());
        _effective_clusters.allocate(_maxNumberComponents.value(),0,_dimension.value());
        _Z.assign(_maxNumberMeasurements.value(),ColumnVector(_dimension.value()));

        _prodDiffW.resize(_dimension.value());
        _differenceZM.resize(_dimension.value());
        _differenceZXbar.resize(_dimension.value());
        _covZX.resize(_dimension.value(),_dimension.value());

        _differenceXbarPriorM.resize(_dimension.value());
        _covXbarPriorM.resize(_dimension.value(),_dimension.value());

        //_Stest.resize(_maxNumberComponents.value());
        //_Stest.assign(_maxNumberComponents.value(),Matrix(_dimension.value(),_dimension.value()));
        return true;
    
    }
    bool VBClusters::startHook()
    {
        return true;
    }
    void VBClusters::updateHook()
    {
    }
    void VBClusters::stopHook()
    {
    }
    void VBClusters::cleanUpHook()
    {
    }

    OCL::VBClusters::Respons_struct VBClusters::calculateResponsabilities(std::vector<ColumnVector> Z , int N_meas, bool normalization = 1)
    {
#ifndef NDEBUG        
        log(Debug) <<"(VBClusters) entering calculateResponsabilities"<<endlog();
#endif
        // return value Respons_struct
        //      R = vector<ColumnVector> and sumR = vector<double>
        //      R[i] = ColumnVector with length number of clusters with responsibilities for i'th measurements
        //      sumR[i] = double with responsibilities of all clusters for i'th measurements
        // bool normalization indicates if returned responsibilities are
        // normalized or not
        //
        // calculate responsibilities of measurements Z with current estimation
        // of clusters _estimate    
        
        
        // number of measurements
        // number of components
        int N_comp = _estimate.N_comp_effective;
#ifndef NDEBUG        
        log(Debug) << "(VBClusters) N_comp " << N_comp << endl;
#endif
        if (N_comp ==0)
        {
            for (int i=0; i<N_meas ; i++)
            {
                _R[i] = 0.0;
                _sumR[i] = 0.0;
            }
            _sumC = 0.0;
            _respons.N_comp_effective = N_comp;
            _respons.N_meas_effective = N_meas;
            _respons.R = _R;
            _respons.sumR = _sumR;
            _respons.N = _sumC;
        }
        else
        {
            //int N_comp = _estimate.Nu.size(); 

            _sumC = 0.0;

            double sum_alfa = 0.0;
            for (int l=0; l<N_comp ; l++)
            {
                sum_alfa = sum_alfa + _estimate.Alfa[l];
#ifndef NDEBUG
                log(Debug) << "component " << l << endlog();
                log(Debug) << "_estimate.W " << _estimate.W[l] << endlog();
                log(Debug) << "_estimate.M " << _estimate.M[l] << endlog();
                log(Debug) << "_estimate.Nu " << _estimate.Nu[l] << endlog();
                log(Debug) << "_estimate.Alfa " << _estimate.Alfa[l] << endlog();
                log(Debug) << "_estimate.Beta " << _estimate.Beta[l] << endlog();
#endif
            }
            double digamma_sum_alfa = digamma(sum_alfa);
            
            double sumRho = 0.0;

            double prodDiffWDiff = 0.0;

            for (int i=0; i<N_meas ; i++)
            {
                sumRho = 0.0;
                for (int l=0; l<N_comp ; l++)
                {
                    double sumd = 0;
                    //_differenceZM = Z[i]-_estimate.M[l];
                    for (int d=0 ; d < _dimension; d++) 
                    {
                        sumd = sumd + digamma( (_estimate.Nu[l] + 1 - d)/2);
                        _differenceZM(d+1) = Z[i](d+1) - _estimate.M[l](d+1);
                    }
                    
                    double factor1 = digamma( _estimate.Alfa[l]) - digamma_sum_alfa ;
                    double factor2 = 1.0/2.0 * ( sumd + _dimension*log(2.0) + log(_estimate.W[l].determinant() )  ) ;
                    double factor3 = -1.0/2.0*log(2.0*M_PI);
                    // _prodDiffW = (Z[i] - _estimate.M[l]).transpose() * ( _estimate.W[l]  ) ;
                    for (int c=0 ; c < _dimension; c++) 
                    {
                        _prodDiffW(c+1) = 0.0;
                        for (int teller=0 ; teller < _dimension; teller++) 
                        {
                            _prodDiffW(c+1) += _differenceZM(teller+1) * _estimate.W[l](teller+1,c+1);
                        }
                    }
                    //prodDiffWdiff = (Z[i] - _estimate.M[l]).transpose() * ( _estimate.W[l] * (Z[i] - _estimate.M[l]) ) ;
                    prodDiffWDiff = 0.0;
                    for (unsigned int r=0; r<_dimension; r++)
                        prodDiffWDiff += _prodDiffW(r+1) * _differenceZM(r+1);
                    double factor4 = -1.0/2.0* ( _dimension * pow(_estimate.Beta[l],-1 ) + prodDiffWDiff * _estimate.Nu[l] );
                    //log(Debug) << "new factor 4" <<  factor4 << endlog();
                    //double factor4 = -1.0/2.0* ( _dimension * pow(_estimate.Beta[l],-1 ) +(Z[i] - _estimate.M[l]).transpose() * ( _estimate.W[l] * (Z[i] - _estimate.M[l]) ) * _estimate.Nu[l] );
                    //log(Debug) << "old factor 4" <<  -1.0/2.0* ( _dimension * pow(_estimate.Beta[l],-1 ) +(Z[i] - _estimate.M[l]).transpose() * ( _estimate.W[l] * (Z[i] - _estimate.M[l]) ) * _estimate.Nu[l] ) << endlog();
                    _Rho(l+1) =  exp( factor1 + factor2+ factor3 + factor4);
                    sumRho = sumRho + _Rho(l+1);
                }
                if (normalization)
                {
                    _R[i] = _Rho / sumRho;
                    _sumR[i] = 1;
                    _sumC = _sumC + _R[i];
                }
                else
                {
                    _R[i] = _Rho;
                    _sumR[i] = sumRho;
                    _sumC = _sumC + _R[i];
                }
            }
            
            _respons.N_comp_effective = N_comp;
            _respons.N_meas_effective = N_meas;
            _respons.R = _R;
            _respons.sumR = _sumR;
            _respons.N = _sumC;
            // find most responsable cluster for each measurement
            _responsibleCluster.assign(_maxNumberMeasurements.value(),-1);
            double maxRespons = -1.0;
            for (int i=0; i<N_meas ; i++)
            {
                maxRespons = -1.0;
                _respons.responsibleCluster[i]=-1;
                for (int l=0; l<N_comp ; l++)
                {
                    if(_respons.R[i](l+1) > maxRespons)
                    {
                        _respons.responsibleCluster[i]=l;
                        maxRespons = _respons.R[i](l+1);
                    }
                }
#ifndef NDEBUG
                log(Debug) << "measurement " << Z[i] << endlog();
                log(Debug) << "maxRespons " << maxRespons << endlog();
                log(Debug) << "_respons.responsibleCluster[i] " << _respons.responsibleCluster[i] << endlog();
#endif
            }
        }
        return _respons;
    }
    
    int VBClusters::calculateInitialPrior(std::vector<ColumnVector> Z,int N_meas)
    {
#ifndef NDEBUG
       log(Debug) <<"(VBClusters) calculateInitialPrior "<<endlog();
#endif
       // number of measurements
       int N_comp;
        
       if (_first_time)
            _first_time = false; // set first time false
       if (_numPriorClustersPort.Get() == 0) // no filters yet 
       {
            N_comp = N_meas;

            // NEW INITIAL
            //Cluster_parameters new_clusters(_maxNumberComponents.value(),N_comp,_dimension, 0.05, 10.0, 1.0/N_comp, 10.0 ,1);
            _new_clusters.N_comp_effective = N_comp;  
            
            for (int k = 0 ; k < N_comp ; k++)
            {
               _new_clusters.M[k] = Z[k];
            }
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new initial "<<endlog();
#endif
            // set initial estimate to new_clusters 
            setInitial(_new_clusters);
            // set estimate equal to initial the first time
            _estimate = _initial;

            // NEW PRIOR 
            //Cluster_parameters new_prior(_maxNumberComponents.value(), N_comp,_dimension, 0.05, 1.0, 1.0, 10.0 ,1);
            _new_prior.N_comp_effective = N_comp;  
            for (int k = 0 ; k < N_comp ; k++)
            {
                _new_prior.M[k] = Z[k];
            }
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new prior "<<endlog();
#endif
            setPrior(_new_prior);
       }
       else
       {
            _priorVBClustersParameters = _priorVBClustersParametersPort.Get();
            // number of components
            _new_prior.N_comp_effective = _numPriorClustersPort.Get();
#ifndef NDEBUG
            log(Debug) << "get priorVBClustersParameters" << endlog();
             << "_new_prior.N_comp_effective" << _new_prior.N_comp_effective << endlog();
#endif
            for (int k = 0 ; k < _new_prior.N_comp_effective ; k++)
            {
               _new_prior.M[k] = _priorVBClustersParameters.M[k];
               _new_prior.W[k] = _priorVBClustersParameters.W[k];
            }
            setPrior(_new_prior);
            setInitial(_new_prior);
            _estimate = _initial;

#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Prior set"<<endlog();
#endif
            
            // calculate responsibilities (without normalization) for current estimate
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Calculate responses during initialization"<<endlog();
#endif
            // fills in _respons, uses _estimate
            calculateResponsabilities(Z,N_meas,0);
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Calculated responses during initialization"<<endlog();
            log(Debug) <<"(VBClusters) Find measurements which are not explained by the current clusters"<<endlog();
            log(Debug) <<"(VBClusters) number of measurements " << N_meas <<endlog();
#endif
            // find measurements which are not explained by the current
            // clusters
            vector<int> unexplained;
            for(int i=0 ; i<N_meas;i++)
            {
                if( _respons.sumR[i] < _treshold_new_cluster )
                {
                    unexplained.push_back(i);
                }
            }
            int number_not_explained = unexplained.size();
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Number not explained by the current clusters"<< number_not_explained <<endlog();

            log(Debug) <<"(VBClusters) Create new clusters for unexplained measurements + double "<<endlog();
#endif
            // WITHOUT DOUBLING
            // create new clusters for unexplained measurements 
            int N_comp_old = _new_prior.N_comp_effective;
            N_comp =  N_comp_old + number_not_explained;
            _new_clusters.N_comp_effective = N_comp;  
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) new cluster parameters created "<<endlog();
#endif

            // existing components
            for (int k = 0 ; k < N_comp_old ; k++)
            {
                _new_clusters.M[k] = _estimate.M[k];
                _new_clusters.W[k] = _estimate.W[k];
#ifndef NDEBUG
                log(Debug) << "component " << k << "mean " << _estimate.M[k] << endlog();
                log(Debug) << "component " << k << "W " << _estimate.W[k] << endlog();
#endif
                // TODO: add random number in stead of fixed value

                _new_clusters.Beta[k] = _estimate.Beta[k];
                _new_clusters.Nu[k] = _estimate.Nu[k];
                _new_clusters.Alfa[k] = _estimate.Alfa[k];
                _new_clusters.Sigma[k] = _estimate.Sigma[k];
            }
            // initialize new components for unexplained measurements
            for (int k=0 ; k<number_not_explained; k ++)
            {
                _new_clusters.M[ N_comp_old + k ] = Z[ unexplained[k] ];
            }

#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new initial "<<endlog();
#endif
            // set initial estimate to new_clusters 
            setInitial(_new_clusters);

            // NEW PRIOR 
            //Cluster_parameters new_prior(_maxNumberComponents.value(), N_comp,_dimension, 0.05, 1.0, 1.0, 10.0 ,1);
            _new_prior.N_comp_effective = N_comp;  

            // double the existing components
            for (int k = 0 ; k < N_comp_old ; k++)
            {
                _new_prior.M[k] = _new_clusters.M[k];
            }
            // initialize new components for unexplained measurements
            for (int k=0 ; k<number_not_explained; k ++)
            {
                _new_prior.M[ N_comp_old + k ] = Z[ unexplained[k] ];
            }
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new prior  with " << N_comp << " components "<<endlog();
#endif
            setPrior(_new_prior);
            
/*

            // WITH DOUBLING
            // create new clusters for unexplained measurements + double the
            // existing components
            int N_comp_old = N_comp;
            N_comp = 2* N_comp + number_not_explained;

            // NEW INITIAL
            _new_clusters.N_comp_effective = N_comp;  
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) new cluster parameters created "<<endlog();
#endif
            //Cluster_parameters new_clusters(N_comp,_dimension, 0.05, 10.0, 10.0 ,1);

            // double the existing components
            for (int k = 0 ; k < N_comp_old ; k++)
            {
                _new_clusters.M[k] = _estimate.M[k];
                // TODO: add random number in stead of fixed value
                _new_clusters.M[k+N_comp_old] = _estimate.M[k] - 0.2;

                _new_clusters.Beta[k] = _estimate.Beta[k];
                _new_clusters.Beta[k+N_comp_old] = _estimate.Beta[k];
                _new_clusters.Nu[k] = _estimate.Nu[k];
                _new_clusters.Nu[k+N_comp_old] = _estimate.Nu[k];
                _new_clusters.Alfa[k] = _estimate.Alfa[k];
                _new_clusters.Alfa[k+N_comp_old] = _estimate.Alfa[k];
                _new_clusters.Sigma[k] = _estimate.Sigma[k];
                _new_clusters.Sigma[k+N_comp_old] = _estimate.Sigma[k];

                // CHO0SE A LITTLE BIT WIDER INITIAL UNCERTAINTY (not necessary)
                // TODO: get factor from property
                _new_clusters.W[k] = _estimate.W[k]/1.0;
                _new_clusters.W[k+N_comp_old] = _estimate.W[k]/1.0;
            }
            // initialize new components for unexplained measurements
            for (int k=0 ; k<number_not_explained; k ++)
            {
                _new_clusters.M[ 2*N_comp_old + k ] = Z[ unexplained[k] ];
            }

#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new initial "<<endlog();
#endif
            // set initial estimate to new_clusters 
            setInitial(_new_clusters);
            
            // NEW PRIOR 
            //Cluster_parameters new_prior(_maxNumberComponents.value(), N_comp,_dimension, 0.05, 1.0, 1.0, 10.0 ,1);
            _new_prior.N_comp_effective = N_comp;  

            // double the existing components
            for (int k = 0 ; k < N_comp_old ; k++)
            {
                _new_prior.M[k] = _new_clusters.M[k];
                _new_prior.M[k+N_comp_old] = _new_clusters.M[k+N_comp_old] ;
                //new_prior.Beta[k] = _estimate.Beta[k];
                //new_prior.Beta[k+N_comp_old] = _estimate.Beta[k];
                //new_prior.Nu[k] = _estimate.Nu[k];
                //new_prior.Nu[k+N_comp_old] = _estimate.Nu[k];
                //new_prior.Alfa[k] = _estimate.Alfa[k];
                //new_prior.Alfa[k+N_comp_old] = _estimate.Alfa[k];
                //new_prior.Sigma[k] = _estimate.Sigma[k];
                //new_prior.Sigma[k+N_comp_old] = _estimate.Sigma[k];
                //new_prior.W[k] = _estimate.W[k];
                //new_prior.W[k+N_comp_old] = _estimate.W[k];
            }
            // initialize new components for unexplained measurements
            for (int k=0 ; k<number_not_explained; k ++)
            {
                _new_prior.M[ 2*N_comp_old + k ] = Z[ unexplained[k] ];
            }
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Set new prior "<<endlog();
#endif
            setPrior(_new_prior);
*/
        }
#ifndef NDEBUG
       log(Debug) <<"(VBClusters) calculateInitialPrior ended"<<endlog();
#endif
        return N_comp;
    }

    //void VBClusters::findClusters()
    OCL::VBClusters::Cluster_parameters VBClusters::findClusters()
    {
#ifndef NDEBUG
        log(Debug) <<"(VBClusters) FindClusters entered "<<endlog();
#endif
        // set different number of iterations the first time
        int max_iter;
        if(_first_time)
            max_iter = _max_iterFirst;
        else    
            max_iter = _max_iter;

        // Z[0] = [x1; y1]
        // Z[1] = [x2; y2]
        // Z[2] = [x3; y3]
        // Get measurements of ports
        _measurementsObjects.Get(_measurementsObjects_loc);
        _numMeasurementsObjects.Get(_numMeasurementsObjects_loc);
        //_measurementsObjectsX.Get(_measurementsObjectsX_loc);
        //_measurementsObjectsY.Get(_measurementsObjectsY_loc);

        // number of measurements
        if (_numMeasurementsObjects_loc == 0)
        {   
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) _numMeasurementsObjects_loc == 0 "<<endlog();
#endif
            _effective_clusters.N_comp_effective = 0;
            // set estimate to effective clusters
            _estimate = _effective_clusters;
            _numClustersPort.Set(0);
            _stateClustersPort.Set(_stateClusters);
            _covarianceClustersPort.Set(_covarianceClusters);
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) ports set"<<endlog();
#endif
        }
        else
        {
            //_Z.assign(N_meas,ColumnVector(_dimension.value()));
            for (int i = 0; i<_numMeasurementsObjects_loc ; i++)
            {
                _meas_local(1) = _measurementsObjects_loc[i](1);
                _meas_local(2) = _measurementsObjects_loc[i](2);
                _Z[i] = _meas_local; 
            }
            assert(_dimension == _Z[0].size() ) ;

            //Set new initial and prior
            //update the clusters for unexplained measurements and double the number
            //of clusters, and adapt initial and prior to that
            
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Calculate initial prior"<<endlog();
#endif
            int N_comp = calculateInitialPrior(_Z,_numMeasurementsObjects_loc);
#ifndef NDEBUG
            log(Debug) <<"(VBClusters) Initial prior calculated"<<endlog();
#endif

            _estimate = _initial;
            

            for (int j=0 ; j<_max_iter;j++)
            { 
                //log(Debug) <<"(VBClusters) VBI iteration " << j <<endlog();
                //log(Debug) <<"(VBClusters) Calculate responses"<<endlog();
                // fills in _respons
                calculateResponsabilities(_Z,_numMeasurementsObjects_loc,1);
                //log(Debug) <<"(VBClusters) Responses calculated"<<endlog();

                // number of measurements explained per cluster = respons.N

                double sumAlfa = 0.0;
                for (int l=0 ; l< N_comp ; l++)
                {
                   _Xbar[l] = 0.0;
                   _S[l] = 0.0;
                   //_Stest[l] = 0.0;
                    if (_respons.N(l+1) > -1e-16 && _respons.N(l+1) < 1e-16)
                    {
                        _Xbar[l] = 1000;
                        _S[l] = 0.0;
                        //_Stest[l] = 0.0;
                    }
                    else
                    {
                        _Xbar[l] = 0.0;
                        for (int i =0 ; i<_numMeasurementsObjects_loc ; i++)
                        {
                            //_Xbar[l] = _Xbar[l] + _Z[i] * ( _respons.R[i](l+1) );
                            for (int d =1 ; d<_dimension+1 ; d++)
                            {
                                _Xbar[l](d) = _Xbar[l](d) + _Z[i](d) * ( _respons.R[i](l+1) );
                            }
                        }
                        //_Xbar[l] = _Xbar[l]/_respons.N(l+1);
                        for (int d =1 ; d<_dimension+1 ; d++)
                        {
                            _Xbar[l](d) = _Xbar[l](d)/_respons.N(l+1);
                        }

                        for (int i =0 ; i<_numMeasurementsObjects_loc ; i++)
                        {
                            for (int d =1 ; d<_dimension+1 ; d++)
                            {
                                _differenceZXbar(d) = _Z[i](d)-_Xbar[l](d);
                            }
                            _covZX = 0.0;
                            for (int r =1 ; r<_dimension+1 ; r++)
                            {
                                for (int c =1 ; c<_dimension+1 ; c++)
                                {
                                    _covZX(r,c) = _differenceZXbar(r) * _differenceZXbar(c);
                                    _S[l](r,c) += _covZX(r,c) * _respons.R[i](l+1);
                                }
                            }
                        }
                       for (int r =1 ; r<_dimension+1 ; r++)
                       {
                           for (int c =1 ; c<_dimension+1 ; c++)
                           {
                               _S[l](r,c) = _S[l](r,c) / _respons.N(l+1);
                           }
                       }
                       //log(Debug) << "_S[l] nieuw" << _S[l] << endlog();
                       //log(Debug) << "_Stest[l]" << _Stest[l] << endlog();
                    }
                    // update parameters of cluster l
                    _estimate.Alfa[l] = _prior.Alfa[l] + _respons.N(l+1);
                    _estimate.Beta[l] = _prior.Beta[l] + _respons.N(l+1);
                    //_estimate.M[l] = ( ( _prior.M[l] * _prior.Beta[l]) + (_Xbar[l] * _respons.N(l+1)) ) / _estimate.Beta[l];
                    for (int d =1 ; d<_dimension+1 ; d++)
                    {
                        _estimate.M[l](d) = ( ( _prior.M[l](d) * _prior.Beta[l]) + (_Xbar[l](d) * _respons.N(l+1)) ) / _estimate.Beta[l];
                        _differenceXbarPriorM(d) = (_Xbar[l](d)-_prior.M[l](d));
                    }

                    //log(Debug) <<"(VBClusters) W0" << _prior.W[l] <<endlog();
                    //log(Debug) <<"(VBClusters) Winv old" << _Winv <<endlog();
                    _Winv = _prior.W[l].inverse() ;
                    for (int r =1 ; r<_dimension+1 ; r++)
                    {
                        for (int c =1 ; c<_dimension+1 ; c++)
                        {
                            _covXbarPriorM(r,c) = _differenceXbarPriorM(r) * _differenceXbarPriorM(c);
                            _Winv(r,c) += _S[l](r,c) * _respons.N(l+1) +  _covXbarPriorM(r,c) * (_prior.Beta[l] * _respons.N(l+1) / (_prior.Beta[l] + _respons.N(l+1)) ); 
                        }
                    }
                    //log(Debug) <<"(VBClusters) Winv"  << _Winv <<endlog();
                    
                    _estimate.W[l] = _Winv.inverse();
                    _estimate.Nu[l] = _prior.Nu[l] + _respons.N(l+1);
                    //_estimate.L[l] = _estimate.Nu[l]/(1+_estimate.Beta[l])*_estimate.Beta[l]*_estimate.W[l];
                    //TODO : pow -1/2 => implement in matrix wrapper for symmetric
                    //matrix
                    
                    for (int r =1 ; r<_dimension+1 ; r++)
                    {
                        for (int c =1 ; c<_dimension+1 ; c++)
                        {
                            _temp(r,c) = _estimate.W[l](r,c)  *  ( _estimate.Nu[l]/(1+_estimate.Beta[l]) * _estimate.Beta[l] ) ;
                        }
                    }
                    _temp.SVD( _w, _U, _V);
                    
                    //_temp = _U * _D ;
                    for (int r =1 ; r<_dimension+1 ; r++)
                    {
                        for (int c =1 ; c<_dimension+1 ; c++)
                        {
                            _temp(r,c) = _U(r,c) * 1/sqrt(_w(c))  ;
                        }
                    }
                    for (int r =1 ; r<_dimension+1 ; r++)
                    {
                        for (int c =1 ; c<_dimension+1 ; c++)
                        {
                            _estimate.Sigma[l](r,c) = 0.0; 
                            for (int teller =1 ; teller<_dimension+1 ; teller++)
                            {
                                _estimate.Sigma[l](r,c) += _temp(r,teller) * _U(c,teller);
                            }
                        }
                    }
                    //log(Debug) << "_estimate.Sigma[l] new " << _estimate.Sigma[l] << endlog();
                    _estimate.Pi[l]= _estimate.Alfa[l]; 
                    sumAlfa = sumAlfa + _estimate.Alfa[l];
                }
                _estimate.Pi = _estimate.Pi / sumAlfa;
             }
#ifndef NDEBUG
            for (int i=0; i<_numMeasurementsObjects_loc ; i++)
            {
                log(Debug) << "measurement " << i <<  ": " << _Z[i] << endlog();
                log(Debug) << "most responsable cluster: " << _respons.responsibleCluster[i] << endlog();
            }
#endif

             // update _estimate such that only effective components are kept
             // only select effective components
             // find effective components
             vector<int> effective;
             int Ncomp_effective = 0;
             for (int l=0 ; l< N_comp ; l++)
             {
                if(_respons.N(l+1) > _treshold_effective_cluster)
                {
                    Ncomp_effective ++;
                    effective.push_back(l);
                }
             }
#ifndef NDEBUG
             log(Debug) <<"(VBClusters)" << Ncomp_effective << " effective components selected out of " << N_comp << " components" <<endlog();
#endif
    
            _effective_clusters.N_comp_effective = Ncomp_effective;
            for (int k=0; k<Ncomp_effective;k++)
            {
                _effective_clusters.Sigma[k] = _estimate.Sigma[ effective[k] ];
                // log(Debug) <<"(VBClusters) Sigma" << _effective_clusters.Sigma[k] <<endlog();
                _effective_clusters.M[k] = _estimate.M[ effective[k] ];
                // log(Debug) <<"(VBClusters) M" << _effective_clusters.M[k] <<endlog();
                _effective_clusters.Beta[k] = _estimate.Beta[ effective[k] ];
                _effective_clusters.Nu[k] = _estimate.Nu[ effective[k] ];
                _effective_clusters.Alfa[k] = _estimate.Alfa[ effective[k] ];
                _effective_clusters.W[k] = _estimate.W[ effective[k] ];
                _stateClusters[k] = _effective_clusters.M[k];
                _effective_clusters.Sigma[k].convertToSymmetricMatrix(_covarianceClusters[k]);
            }
            // set estimate to effective clusters
            _estimate = _effective_clusters;

            //_estimatePort.Set(_estimate);
            _stateClustersPort.Set(_stateClusters);
            _covarianceClustersPort.Set(_covarianceClusters);
            _numClustersPort.Set(Ncomp_effective);

            // find responsable effective component for measurement
            _responsibleCluster.assign(_maxNumberMeasurements.value(),-1);
            for (int i=0; i<_numMeasurementsObjects_loc ; i++)
            {
                _responsibleCluster[i] = -1; 
#ifndef NDEBUG
                log(Debug) << "most responsible cluster" << _respons.responsibleCluster[i] << endlog();
#endif
                for (int k=0; k<Ncomp_effective;k++)
                {
#ifndef NDEBUG
                    log(Debug)  << "effective component " << effective[k] << endlog();
#endif
                    if(_respons.responsibleCluster[i] == effective[k])
                    {
                        _responsibleCluster[i] = k;
                        break;
                    }
                }
#ifndef NDEBUG
                log(Debug) << "most responsible effective cluster" << _responsibleCluster[i] << endlog();
#endif
            }
            _responsibleClusterPort.Set(_responsibleCluster);
#ifndef NDEBUG
            for (int i=0; i<_numMeasurementsObjects_loc ; i++)
            {
                log(Debug) << "measurement " << i <<  ": " << _Z[i] << endlog();
                log(Debug) << "most responsable cluster: " << _responsibleCluster[i] << endlog();
            }
            log(Debug) << "(VBClusters) number of effective components " << Ncomp_effective << endlog();
#endif

        }    
#ifndef NDEBUG
        log(Debug) <<"(VBClusters) findClusters finished" <<endlog();
#endif

        return _estimate;
    }

    void VBClusters::setInitial(Cluster_parameters initial)
    {
        _initial = initial;
    }

    void VBClusters::setPrior(Cluster_parameters prior)
    {
        _prior = prior;
    }

    void VBClusters::setMaxIter(int max_iter)
    {
        _max_iter = max_iter;
    }
}//namespace


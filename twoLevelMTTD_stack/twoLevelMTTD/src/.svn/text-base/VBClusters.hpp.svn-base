#ifndef __VBCLUSTERS__
#define __VBCLUSTERS__

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Method.hpp>
#include <rtt/Command.hpp>
#include <rtt/Event.hpp>
#include <rtt/Ports.hpp>
#include <rtt/PeriodicActivity.hpp>

#include <ocl/OCL.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/bfl_constants.h>
#include <bfl/pdf/gaussian.h>

#include <fstream>

#include <boost/math/special_functions/digamma.hpp>

using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;
using namespace boost::math;

namespace OCL
{

    /**
    VBClusters
    * This class implements the clustering using Variational Bayesian Cluster
    * Finding
     */
    class VBClusters
        : public TaskContext
    {

    public:
        struct Respons_struct
        {
           /**
           Respons_struct
           * struct containing values useful to represent the responsabilities
           * of the clusters for some points (measurements)
           */
           public:
                /// Responsabilities per cluster
                std::vector<ColumnVector > R; 
                /// Most responsible cluster
                std::vector<int > responsibleCluster; 
                /// Responsabilities for all clusters for a certain point
                std::vector<double> sumR; 
                /// Sum of responsibilities for each cluster (length = number of clusters)
                ColumnVector N; 
                /// The number of effective points
                int N_meas_effective;  
                /// The number of effective clusters
                int N_comp_effective;

                /*
                * constructor.
                * @param N_meas number of points
                * @param N_comp number of clusters
                * @param N_meas_eff number of effective points
                * @param N_comp_eff number of effective clusters
                */
                Respons_struct(int N_meas = 181, int N_comp = 1, int N_meas_eff = 0, int N_comp_eff = 0)
                    : R(N_meas,ColumnVector(N_comp)), sumR(N_meas), N(N_comp), N_meas_effective(N_meas_eff), N_comp_effective(N_comp_eff)
                {
                }
                /*
                * helper function to allocate all the memory necessary to respresent the responsabilities.
                * @param N_meas number of points
                * @param N_comp number of clusters
                */
                void allocate(int N_meas, int N_comp)
                {
                    R.assign(N_meas,ColumnVector(N_comp));
                    responsibleCluster.assign(N_meas,-1);
                    sumR.resize(N_meas);
                    N.resize(N_comp);   
                }
        };

        struct Cluster_parameters
        {
           public:
                /// beta
                std::vector<double> Beta;
                /// Nu
                std::vector<double> Nu;
                /// Alfa
                std::vector<double> Alfa;
                /// Mean of each of the clusters
                std::vector<ColumnVector > M;
                /// W
                std::vector<Matrix> W;
                /// Sigma; covariance of each of the clusters
                std::vector<Matrix> Sigma;
                /// Probability of each of the clusters
                ColumnVector Pi;
                /// Number of effective components
                int N_comp_effective;
                
                /*
                * constructor.
                * @param N_comp number of clusters
                * @param N_comp_eff number of effective clusters
                * @param D dimension
                * @param Beta_init beta
                * @param Nu_init nu
                * @param Alfa_init alfa
                * @param W_init W
                * @param Sigma_init sigma
                * @param Pi_init pi
                */
                Cluster_parameters(int N_comp=1, int N_comp_eff=0, int D=1, double Beta_init = 0.05, double Nu_init = 10.0, double Alfa_init = 1.0 ,double W_init = 10.0, double Sigma_init = 1.0 , double Pi_init = 0.0)
                   : N_comp_effective(N_comp_eff), Beta(N_comp,Beta_init), Nu(N_comp,Nu_init), Alfa(N_comp,Alfa_init), Pi(N_comp)
                {
                    // allocate 
                    Pi = Pi_init;
                    M.assign(N_comp,ColumnVector(D));
                    Matrix W_el(D,D);
                    W_el = 0.0;
                    Matrix Sigma_el(D,D);
                    Sigma_el = 0.0;
                    for (int d=1; d<=D; d++)
                    {
                        W_el(d,d) = W_init;
                        Sigma_el(d,d) = Sigma_init;
                    }
                    W.assign(N_comp,W_el);
                    Sigma.assign(N_comp,Sigma_el);
                }

                /*
                * helper function to allocate all the memory necessary to respresent the cluster parameters.
                * @param N_comp number of clusters
                * @param N_comp_eff number of effective components
                * @param D dimension
                * @param Beta_init beta
                * @param Nu_init nu
                * @param Alfa_init alfa
                * @param W_init W
                * @param Sigma_init sigma
                * @param Pi_init pi
                */
                void allocate(int N_comp, int N_comp_eff=0, int D=1, double Beta_init = 0.05, double Nu_init = 10.0, double Alfa_init = 1.0 ,double W_init = 10.0, double Sigma_init = 1.0 , double Pi_init = 0.0)
                {
                    N_comp_effective = N_comp_eff;
                    Beta.assign(N_comp,Beta_init);
                    Nu.assign(N_comp,Nu_init);
                    Alfa.assign(N_comp,Alfa_init);
                    Pi.resize(N_comp);
                    Pi= Pi_init;
                    M.assign(N_comp,ColumnVector(D));
                    Matrix W_el(D,D);
                    W_el = 0.0;
                    Matrix Sigma_el(D,D);
                    Sigma_el = 0.0;
                    for (int d=1; d<=D; d++)
                    {
                        W_el(d,d) = W_init;
                        Sigma_el(d,d) = Sigma_init;
                    }
                    W.assign(N_comp,W_el);
                    Sigma.assign(N_comp,Sigma_el);
                }
        };


    private:
        /*
        * Find the clusters 
        * @return the parameters of the clusters
        */
        Cluster_parameters                  findClusters(); 
        /*
        * Calculate the responsabilities of the points using the clusters 
        * @param Z vector containing the points 
        * @param N_meas the number of measurements 
        * @param normalization boolean indicating if the responsabilities have to be normalized or not 
        * @return the responsabilities of the clusters for the points 
        */
        Respons_struct                      calculateResponsabilities(std::vector<ColumnVector >  Z, int N_meas, bool normalization);
        /*
        * Construct a prior for the clustering using the measurements
        * @param Z vector containing the points 
        * @param N_meas the number of measurements 
        * @return the number of clusters in the prior
        */
        int                                 calculateInitialPrior(std::vector<ColumnVector> Z, int N_meas);
        /// The initial estimate of the cluster parameters
        Cluster_parameters                  _initial;
        /// The prior on the cluster parameters
        Cluster_parameters                  _prior;
        /// The parameters of the estimated clusters
        Cluster_parameters                  _estimate;
        /// Helper variable for the old parameters of the estimated clusters
        Cluster_parameters                  _estimate_old;
        /// Helper variable 
        std::vector<ColumnVector>           _Xbar;
        /// Vector containing the points to be clustered
        std::vector<ColumnVector>           _Z;
        /// Helper variable 
        std::vector<Matrix>                 _S;
        /// Helper variable for vector of points to be clustered
        std::vector<ColumnVector>           _measurementsObjects_loc;
        /// Helper variable for the number of points to be clustered
        int                                 _numMeasurementsObjects_loc;
        /// The state of the estimated clusters
        std::vector<ColumnVector>           _stateClusters;
        /// The covariance of the estimated clusters
        std::vector<SymmetricMatrix>        _covarianceClusters;
        /// Vector with the most responsable cluster for each measurement
        std::vector<int>                    _responsibleCluster;
        /// Helper variable for a measurement
        ColumnVector                        _meas_local;
        /// Helper variable
        bool                                _first_time;
        /// Helper variable
        ColumnVector                        _Rho;
        /// Helper variable
        vector<double>                      _sumR;
        /// Helper variable
        ColumnVector                        _sumC;
        /// Helper variable
        vector<ColumnVector>                _R;
        /// Helper variable
        Respons_struct                      _respons;
        /// Helper variable
        Cluster_parameters                  _new_clusters;
        /// Helper variable
        Cluster_parameters                  _new_prior;
        /// Helper variable
        Cluster_parameters                  _effective_clusters;
        /// Helper variable
        Matrix                              _Winv;
        /// Helper variable
        Matrix                              _temp;
        /// Helper variable
        ColumnVector                        _w;
        /// Helper variable
        Matrix                              _U;
        /// Helper variable
        Matrix                              _V;
        /// Helper variable
        Matrix                              _D;
        /// Helper variable
        RowVector                           _prodDiffW;
        /// Helper variable
        ColumnVector                        _differenceZM;
        /// Helper variable
        ColumnVector                        _differenceZXbar;
        /// Helper variable
        Matrix                              _covZX;
        /// Helper variable
        ColumnVector                        _differenceXbarPriorM;
        /// Helper variable
        Matrix                              _covXbarPriorM;
        /// Helper variable to store the parameters of the prior for the VBClusters depending on the current target state and shape 
        OCL::VBClusters::Cluster_parameters _priorVBClustersParameters;

    protected:
        /* PROPERTIES */

        /// The dimension of the clustering space
        Property<int>                  _dimension;
        /// The threshold for the responsability below which a new cluster is started
        Property<double>               _treshold_new_cluster;
        /// The threshold for the responsability below which a cluster is considered to be effective
        Property<double>               _treshold_effective_cluster;
        /// The maximum number of iterations of the clustering algorithm
        Property<int>                  _max_iter;
        /// The maximum number of iterations for the first run 
        Property<int>                  _max_iterFirst;
        /// The maximum number of clusters (needed for reporter)
        Property<int>                  _maxNumberClusters;
        /// The maximum number of clusters (needed for allocation)
        Property<int>                  _maxNumberComponents;
        /// The maximum number of measurments (allocation)
        Property<int>                  _maxNumberMeasurements;
        /// The prior value for the beta of the cluster parameters
        Property<double>               _betaPrior;
        /// The prior value for the nu of the cluster parameters
        Property<double>               _nuPrior;
        /// The prior value for the alpha of the cluster parameters
        Property<double>               _alphaPrior;
        /// The prior value for the W of the cluster parameters
        Property<double>               _WPrior;
        /// The prior value for the Sigma of the cluster parameters
        Property<double>               _SigmaPrior;
        /// The prior value for the beta of the cluster parameters
        Property<double>               _betaNewCluster;
        /// The prior value for the nu of the cluster parameters
        Property<double>               _nuNewCluster;
        /// The prior value for the alpha of the cluster parameters
        Property<double>               _alphaNewCluster;
        /// The  value for the W for a new cluster
        Property<double>               _WNewCluster;
        /// The  value for the Sigma for a new cluster
        Property<double>               _SigmaNewCluster;

        /* METHODS */
        /// Find the new clusters
        Method<Cluster_parameters(void) >            _findClusters;
        /// Set the initial for the clustering
        Method<void(Cluster_parameters) >            _setInitial;
        /// Set the prior for the clustering
        Method<void(Cluster_parameters) >            _setPrior;

        /* DATA PORTS */
        /// Data port to write the parameters of the estimated clusters
        WriteDataPort<Cluster_parameters>            _estimatePort;
        /// Data port to write the state of the estimated clusters
        WriteDataPort<std::vector<ColumnVector> >    _stateClustersPort;
        /// Data port to write the covariances of the estimated clusters
        WriteDataPort<std::vector<SymmetricMatrix> > _covarianceClustersPort;
        /// Data port to write the number of estimated clusters
        WriteDataPort<int >                          _numClustersPort;
        /// Data port to write the most repsonsable cluster for each measurement 
        WriteDataPort<std::vector<int> >            _responsibleClusterPort;
        /// Data port to read the points to be clustered
        ReadDataPort<std::vector<ColumnVector> >    _measurementsObjects;
        /// Data port to read the number of points to be cluster
        ReadDataPort<int >                          _numMeasurementsObjects;
        /// The port to read the prior for the clustering based on the objects state and shape 
        ReadDataPort<OCL::VBClusters::Cluster_parameters>               _priorVBClustersParametersPort;
        /// The port to read the number of prior clusters
        ReadDataPort<int>               _numPriorClustersPort;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        VBClusters(std::string name);
        /** 
        * Destructor
        */
        ~VBClusters();
        /** 
        * set the initial cluster parameters for clustering
        * @param initial the initial cluster parameters
        */
        void        setInitial(Cluster_parameters initial);
        /** 
        * set the prior cluster parameters for clustering
        * @param prior the prior cluster parameters
        */
        void        setPrior(Cluster_parameters prior);
        /** 
        * set the number of iterations 
        * @param max_iter the prior cluster parameters
        */
        void        setMaxIter(int max_iter);
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}

#endif // __VBCLUSTERS__

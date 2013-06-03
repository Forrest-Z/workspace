#ifndef __ESTIMATIONGAUSSIAN__
#define __ESTIMATIONGAUSSIAN__

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Method.hpp>
#include <rtt/Command.hpp>
#include <rtt/Event.hpp>
#include <rtt/Ports.hpp>
#include <rtt/PeriodicActivity.hpp>
#include <rtt/TimeService.hpp>
#include <rtt/RealTimeToolkit.hpp>

#include <ocl/OCL.hpp>
#include <ocl/FileReporting.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/bfl_constants.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/dataAssociation/dataAssociationFilterGaussian.h>
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/discreteconditionalpdf.h>
#include <bfl/pdf/discretepdf.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <fstream>

#include <boost/math/special_functions/digamma.hpp>

#include <string>

#include <dataAssociationFilterGaussianMurty.h>

#include "VBClusters.hpp"

using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;
using namespace boost::math;

namespace OCL
{
    /***************
    EstimationGaussian
    * This class implements the Online two-level Bayesian multitarget
    * tracking and localization algorithm proposed by Tinne De Laet (add
    * reference), for the case Extended Kalman filters are used for the individual
    * target trackers
    *****************/
    class EstimationGaussian
        : public TaskContext
    {

    private:
        /// the dimension of the state space
        int                                                     _dimension; 
        /// The data association filter (JPDAF)
        DataAssociationFilterGaussianMurty* _dataAssociationFilterGaussian;
        /// Helper variable to store the clusters
        std::vector<ColumnVector>                               _clusters_loc;
        /// Helper variable to store the measurmements
        std::vector<ColumnVector>                               _measurements_loc;
        /// The number of clusters
        int                                                     _numClusters;
        /// The number of clusters
        int                                                     _numMeasurements;
        /// The pdf underlying the measurement model
        LinearAnalyticConditionalGaussian*                      _measPdf;
        /// The measurement model
        LinearAnalyticMeasurementModelGaussianUncertainty*      _measModel;
        /// The pdf underlying the system model
        LinearAnalyticConditionalGaussian*                      _sysPdf;
        /// The system model
        LinearAnalyticSystemModelGaussianUncertainty*           _sysModel;
        /// The mean of the prior pdf
        ColumnVector                                            _priorMu;
        /// The prior pdf
        Gaussian                                                _priorCont;
        /// Vector containing the state of the target trackers
        std::vector<ColumnVector>                               _stateFilters;
        /// Vector containing the covariance of the target trackers
        std::vector<SymmetricMatrix>                            _covarianceFilters;
        /// Vector containing the filter IDs (double)
        std::vector<double>                                     _filterID;
        /// Vector containing the filter IDs (int)
        std::vector<int>                                        _filterID_int;
        /// The state of the target closest to the robot (point to which distances are calculated)
        ColumnVector                                            _closestTargetState;
        /// The distance to the robot of the target closest to the robot (point to which distances are calculated)
        double                                                  _distanceClosestTarget;
        /// The sensing parameters for the measurement model
        ColumnVector                                            _s;
        /// The data association probabilities
        vector<vector<double> >                                 _association_probs;
        /// Vector containing numbers of unexplained featurs
        vector<int >                                            _features_unexplained;
        /// to measure time needed for updateHook
        RTT::TimeService::ticks                                 time_begin;
        /// the time needed for updateHook
        double                                                  time_passedUpdateHook;
        /// the time needed until preparing measurements
        double                                                  time_passedPrepare;
        /// the time needed until  clustering
        double                                                  time_passedClustering;
        /// the time needed until estimating number of objects
        double                                                  time_passedNumObjects;
        /// the time needed until updating the filters 
        double                                                  time_passedUpdateFilters;
        /// the time needed until updating reporting 
        double                                                  time_passedReporting;
        /// vector containing the timings
        Matrix                                                  _timings;
        /// counter for the updateHook
        int                                                     _counterUpdateHook;
        /// Vector containing the x positions of the target trackers
        std::vector<double>                                     _MxFilters;
        /// Vector containing the y positions of the target trackers
        std::vector<double>                                     _MyFilters;
        /// Helper variable to store the parameters of the prior for the VBClusters depending on the current target state and shape 
        OCL::VBClusters::Cluster_parameters                      _priorVBClustersParameters;
        /// Vector containing the data association probabilities prob[i][j] probability that measurement[i] is caused by target targetID[i]
        vector<vector<double> >                                 _association_probsLocal;
        /**
        * helper function to update all the target trackers
        * @return boolean indicating of updating was succesful or not
        */
        bool        updateFilters();
        /**
        * helper function to find new objects (looking for unexplained
        * measurements)
        * @param measurements the measurements 
        */
        bool        findNewObjects(vector<ColumnVector> measurements);
        /**
        * helper function delete filters not tracking any more targets 
        * @return boolean indicating of the deleting was succesful or not
        */
        bool        deleteOldObjects();
        /**
        * helper function deleting the most uncertain object 
        * @return boolean indicating of the deleting was succesful or not
        */
        bool        deleteMostUncertainObject();
        /**
        * function to calculate prior parameters for VBClusters based on shape of object , places result on _priorVBClustersParametersPort
        * @return boolean indicating of the calculation of the prior was succesful or not 
        */
        bool        calculatePriorVBClustersParameters();
        /**
        * helper function to calculate the distance of a target to the robot
        * (point to which the distances are caculated)
        * @param targetState the target state of which to calculate the distance to
        * the robot
        * @return the distance to the robot
        */
        double      calculateDistanceToRobot(ColumnVector targetState);
        /**
        * helper function calculating the factorial of an int 
        * @param a the integer of which to calculate the factorial
        * @return the factorial
        */
        int        factorial(int a);
        /**
        * function to write timing to a file 
        * @return boolean indicating if writing was succesful or not
        */
        bool        writeTimings();
        /**

    protected:
        /**********
        PROPERTIES
        ***********/
        /// Reporting needed or not 
        Property<bool>                   _reportingNeeded;
        /// PrepareMeasurement needed or not 
        Property<bool>                   _prepareMeasurementNeeded;
        /// The dimension of the measurement space
        Property<int>                   _measDimension;
        /// The threshold for the assocation probability to take into account the association
        Property<double>                _treshold;
        /// The probability of a false alarm
        Property<double>                _gamma;
        /// The threshold of the probability of a feature given all object below which a new filter is started
        Property<double>                _tresholdAddFilter;
        /// The threshold of the determinant of the covariance of the target tracker above which the filter can be deleted 
        Property<double>                _tresholdDeleteFilter;
        /// The maximum number of filters that can be used (for memory allocation)
        Property<int>                   _maxNumberFilters;
        /// The maximum number of Clusters that can be used (for memory allocation)
        Property<int>                   _maxNumberClusters;
        /// The maximum number of Measurments that can be used (for memory allocation)
        Property<int>                   _maxNumberMeasurements;
        /// The position of the robot: i.e. the point to which the distance of the targets has to be calculated
        Property<ColumnVector>          _robotState;
        /// The mean of the prior of the trackers
        Property<ColumnVector>          _priorMean;
        /// ColumnVector containting the diagonal values of the initial covariances on the target state : it is assumed that the initial estimates are uncoupled (i.e. the initial covariance matrix is  diagonal) 
        Property<ColumnVector>          _priorCovariance;
        /// The level of continuity of the system model 
        Property<int>                   _level;
        /// The mean of the white noise on the target system model
        Property<double>                _sysNoiseMeanValue;
        /// The covariance of the white noise on the target system model
        Property<double>                _sysNoiseCovarianceValue;
        /// The mean of the measurement noise
        Property<ColumnVector>          _measNoiseMean;
        /// The covariance of the measurement noise
        Property<SymmetricMatrix>       _measNoiseCovariance;
        /// The measurement matrix of the linear measurement model
        Property<Matrix>                _measModelMatrix;
        /// The expected radius of the targets
        Property<double>                _radiusTargets;

        /**********
        DATA PORTS
        ***********/
        /// Read data port for the measurements
        ReadDataPort<std::vector<ColumnVector> >        _measurementsObjectsPort;
        /// Read data port for the number of measurements
        ReadDataPort<int >                              _numMeasurementsObjectsPort;
        /// Read data port for the state of the clusters
        ReadDataPort<std::vector<ColumnVector> >        _stateClustersPort;
        /// Read data port for the covariance (shape) of the clusters
        ReadDataPort<std::vector<SymmetricMatrix> >     _covarianceClustersPort;
        /// Read data port for the number of clusters 
        ReadDataPort<int >                              _numClustersPort;
        /// Read data port for the responsable clusters of the measurements 
        ReadDataPort<vector<int> >                       _responsibleClusterPort;
        /// Read data port for the estimated number of targets
        ReadDataPort<int >                              _numObjectsPort;
        /// Write data port for the state of the estimated targets
        WriteDataPort<std::vector<ColumnVector> >       _estimateStateFiltersPort;
        /// Write data port for the covariance of the estimated targets
        WriteDataPort<std::vector<SymmetricMatrix> >    _estimateCovarianceFiltersPort;
        /// Write data port for the number of targets
        WriteDataPort<int >                             _numFiltersPort;
        /// Write data port for the filter IDs (= unique number identifying the target)
        WriteDataPort<std::vector<double >  >           _filterIDPort;
        /// The state of the target closest to the robot (point to which th distance of the targets is calculated)
        WriteDataPort<ColumnVector >                    _closestTargetStatePort;
        /// The distance of the target closest to the robot (point to which the distance of the targets is calculated)
        WriteDataPort<double >                          _distanceClosestTargetPort;
        /// For Blender visualization: x positions of the estimated targets
        WriteDataPort<std::vector<double> >             _estimateMxFiltersPort;
        /// For Blender visualization: y positions of the estimated targets
        WriteDataPort<std::vector<double> >             _estimateMyFiltersPort;
        /// The port to help writing the timings to a file 
        WriteDataPort<Matrix>                           _timingsPort;
        /// The port to write the result of the calculatePriorVBClustersParameters
        WriteDataPort<OCL::VBClusters::Cluster_parameters> _priorVBClustersParametersPort;
        /// The port to write the number of prior clusters
        WriteDataPort<int>                              _numPriorClustersPort;
        /// The port to write the association probabilities 
        WriteDataPort<std::vector<std::vector<double> > >    _associationProbsPort;

        /**********
        METHODS 
        ***********/
        /// Update the filters
        Method<bool(void) >                                 _UpdateFilters;
        /// Prepare a measurement=> call passed on to the PrepareSeparatedMeasurement peer 
        Method<void(void)>                                  prepareMeasurement;
        /// cluster the parameters (low level) => call passed on to the VBClusters peer 
        Method<OCL::VBClusters::Cluster_parameters(void)>   findClusters;
        /// estimate the number of targets => call passed on to the estimateNumObjects peer 
        Method<void(void)>                                  estimateNumObjects;
        /// take a snapshot with the reporter
        Method<void(void)>                                  snapshotReporting;
        /// write timings to a file 
        Method<bool(void)>                                  _WriteTimings;
        /// take a snapshot with the reporter for the timings 
        Method<void(void)>                                  snapshotReportingTimings;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        EstimationGaussian(std::string name);
        /** 
        * Destructor
        */
        ~EstimationGaussian();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __ESTIMATIONGAUSSIAN__

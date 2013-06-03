#include "EstimationRBBM.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::EstimationRBBM)
//ORO_CREATE_COMPONENT_TYPE()  

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     EstimationRBBM::EstimationRBBM(std::string name)
         : TaskContext(name,PreOperational)
          ,_reportingNeeded("reportingNeeded", "Reporting needed or not")
          ,_prepareMeasurementNeeded("prepareMeasurementNeeded", "Prepare measurement needed or not")
          ,_measDimension("measDimension", "dimension of the measurement space")
          ,_treshold("treshold", "the treshold of the association probability to take into account the association")
          ,_gamma("gamma", "the probability of a false alarm")
          ,_tresholdAddFilter("tresholdAddFilter", "the treshold of the probability of a feature given all objects below which a new filter is started ")
          ,_tresholdDeleteFilter("tresholdDeleteFilter", "the treshold of the determinant of the covariance above which filter is deleted ")
          ,_maxNumberFilters("maxNumberFilters","The maximum number of filters")
          ,_maxNumberClusters("maxNumberClusters","The maximum number of clusters")
          ,_maxNumberMeasurements("maxNumberMeasurements","The maximum number of measurements")
          ,_maxNumberAssociations("maxNumberAssociations","The maximum number of associations considered in the data association filter")
          ,_maxNumberAssociationCalls("maxNumberAssociationCalls","The maximum number of calls to the recursive GetAssocations function of the data association filter")
          ,_robotState("robotState","The state of the robot in the laserscanner frame")
          ,_priorMean("priorMean","The mean of the prior")
          ,_priorCovariance("priorCovariance","The covariance of the prior")
          ,_level("level", "the level of continuity of the system model: 0 = cte position, 1= cte velocity ,... ") 
          ,_sysNoiseMeanValue("sysNoiseMeanValue","The mean of the noise on the system model")
          ,_sysNoiseCovarianceValue("sysNoiseCovarianceValue","The covariance of the noise of the system model")
          ,_numSamples("numSamples","The number of samples")
          ,_resamplePeriod("resamplePeriod", "the period of resampling of the MBF, for static resampling (for dynamic resampling set to 0) ")
          ,_resampleThreshold("resampleThreshold", "the ESS threshold for resampling of the MBF, for dynamic resampling (for static resampling set to 0) ")
          ,_resampleScheme("resampleScheme", "the resample scheme for the BF ")
          ,_radiusState("radiusState", "radius of modeled state")
          ,_p("p", "p variable of RBBM")
          ,_zmax("zmax", "zmax variable of RBBM")
          ,_sigma_m("sigma_m", "sigma_m variable of RBBM")
          ,_pi3("pi3", "pi3 variable of RBBM")
          ,_pi4("pi4", "pi4 variable of RBBM")
         , _environment_file("environment_file", "file in which measurements of environment are stored")
         , _nbeams("nbeams", "number of beams of laserscanner")
          ,_UpdateFilters("updateFilters", &EstimationRBBM::updateFilters, this)
          ,_WriteTimings("writeTimings", &EstimationRBBM::writeTimings, this)
          ,_measurementsObjectsPort("measurementsObjects")
          ,_numMeasurementsObjectsPort("numMeasurementsObjects")
          ,_stateClustersPort("stateClusters")
          ,_covarianceClustersPort("covarianceClusters")
          ,_numClustersPort("numClusters")
          ,_responsibleClusterPort("responsibleCluster")
          ,_numObjectsPort("numObjectsEstimated")
          ,_estimateStateFiltersPort("estimateStateFilters")
          ,_estimateCovarianceFiltersPort("estimateCovarianceFilters")
          ,_numFiltersPort("numFilters")
          ,_filterIDPort("filterID")
          ,_closestTargetStatePort("closestTargetState")
          ,_distanceClosestTargetPort("distanceClosestTarget")
          ,_estimateMxFiltersPort("estimateMxFilters")
          ,_estimateMyFiltersPort("estimateMyFilters")
          ,_timingsPort("timings")
          ,_priorVBClustersParametersPort("priorVBClustersParameters")
          ,_numPriorClustersPort("numPriorClustersPort")
          ,_priorCont(4) //TODO: due to error in b
          ,_features_unexplained(181)
     {
        // Check if all initialisation was ok:
        assert( _reportingNeeded.ready() );
        assert( _prepareMeasurementNeeded.ready() );
        assert( _measDimension.ready() );
        assert( _treshold.ready() );
        assert( _gamma.ready() );
        assert( _tresholdAddFilter.ready() );
        assert( _tresholdDeleteFilter.ready() );
        assert( _maxNumberFilters.ready() );
        assert( _maxNumberClusters.ready() );
        assert( _maxNumberAssociations.ready() );
        assert( _maxNumberAssociationCalls.ready() );
        assert( _robotState.ready() );
        assert( _priorMean.ready() );
        assert( _priorCovariance.ready() );
        assert( _level.ready() );
        assert( _sysNoiseMeanValue.ready() );
        assert( _sysNoiseCovarianceValue.ready() );
        assert( _numSamples.ready() );
        assert( _resamplePeriod.ready() );
        assert( _resampleThreshold.ready() );
        assert( _resampleScheme.ready() );
        assert( _radiusState.ready() );
        assert( _p.ready() );
        assert( _zmax.ready() );
        assert( _sigma_m.ready() );
        assert( _pi3.ready() );
        assert( _pi4.ready() );
        assert( _environment_file.ready() );
        assert( _nbeams.ready() );
        assert( _UpdateFilters.ready() );
        assert( _WriteTimings.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_reportingNeeded);
        this->properties()->addProperty(&_prepareMeasurementNeeded);
        this->properties()->addProperty(&_measDimension);
        this->properties()->addProperty(&_treshold);
        this->properties()->addProperty(&_gamma);
        this->properties()->addProperty(&_tresholdAddFilter);
        this->properties()->addProperty(&_tresholdDeleteFilter);
        this->properties()->addProperty(&_maxNumberFilters);
        this->properties()->addProperty(&_maxNumberClusters);
        this->properties()->addProperty(&_maxNumberMeasurements);
        this->properties()->addProperty(&_maxNumberAssociations);
        this->properties()->addProperty(&_maxNumberAssociationCalls);
        this->properties()->addProperty(&_robotState);
        this->properties()->addProperty(&_priorMean);
        this->properties()->addProperty(&_priorCovariance);
        this->properties()->addProperty(&_level);
        this->properties()->addProperty(&_sysNoiseMeanValue);
        this->properties()->addProperty(&_sysNoiseCovarianceValue);
        this->properties()->addProperty(&_numSamples);
        this->properties()->addProperty(&_resamplePeriod);
        this->properties()->addProperty(&_resampleThreshold);
        this->properties()->addProperty(&_resampleScheme);
        this->properties()->addProperty(&_radiusState);
        this->properties()->addProperty(&_p);
        this->properties()->addProperty(&_zmax);
        this->properties()->addProperty(&_sigma_m);
        this->properties()->addProperty(&_pi3);
        this->properties()->addProperty(&_pi4);
        this->properties()->addProperty(&_environment_file);
        this->properties()->addProperty(&_nbeams);
        this->methods()->addMethod(&_UpdateFilters, "update the filters");
        this->methods()->addMethod(&_WriteTimings, "write timings to a file");

        this->ports()->addPort(&_measurementsObjectsPort);
        this->ports()->addPort(&_numMeasurementsObjectsPort);
        this->ports()->addPort(&_stateClustersPort);
        this->ports()->addPort(&_covarianceClustersPort);
        this->ports()->addPort(&_numClustersPort);
        this->ports()->addPort(&_responsibleClusterPort);
        this->ports()->addPort(&_numObjectsPort);
        this->ports()->addPort(&_estimateStateFiltersPort);
        this->ports()->addPort(&_estimateCovarianceFiltersPort);
        this->ports()->addPort(&_numFiltersPort);
        this->ports()->addPort(&_filterIDPort);
        this->ports()->addPort(&_closestTargetStatePort);
        this->ports()->addPort(&_distanceClosestTargetPort);
        this->ports()->addPort(&_estimateMxFiltersPort);
        this->ports()->addPort(&_estimateMyFiltersPort);
        this->ports()->addPort(&_timingsPort);
        this->ports()->addPort(&_priorVBClustersParametersPort);
        this->ports()->addPort(&_numPriorClustersPort);
     }

    EstimationRBBM::~EstimationRBBM()
    {
        delete _dataAssociationFilterMCPdf;
        delete _sysPdf;
        delete _sysModel;
        delete _measPdf;
        delete _measModel;
    }

    bool EstimationRBBM::configureHook()
    {
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) ConfigureHook entered" << endlog();
#endif
        _dimension = 2 * (_level.value()+1);

#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) check properties " << endlog();
#endif
        // Check properties
        if(_priorMean.value().size() != _dimension)
        {
            log(Error) << "priorMean Property does not has correct dimension " << endlog();
            return false;
        }
        if( (_priorCovariance.value().size() != _dimension ) )
        {
            log(Error) << "priorCovariance Property does not has correct dimension " << endlog();
            return false;
        }
        if(_robotState.value().size() != _dimension )
        {
            log(Error) << "robotState Property does not has right dimension " << endlog();
            return false;
        }
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) checked properties " << endlog();
#endif

        // Properties for new filter 
        // initial position
        _priorMu.resize(_dimension);
        _priorMu = 0.0;
        _priorCont.DimensionSet(_dimension); 
        _priorCont.ExpectedValueSet(_priorMean.value()); 

        SymmetricMatrix _priorCovarianceMatrix(_dimension);
        _priorCovarianceMatrix = 0.0;
        for(int i=1 ; i<=_dimension; i++)
            _priorCovarianceMatrix(i,i) = _priorCovariance.value()(i);

        _priorCont.CovarianceSet(_priorCovarianceMatrix); 
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) prior covariance " << endlog();
#endif

        /************************
        // make data association filter 
        ************************/
        vector<Filter<ColumnVector,ColumnVector >* > empty(0);
        _dataAssociationFilterMCPdf = new DataAssociationFilterMCPdfMurty<ColumnVector,ColumnVector>(empty,_gamma,_treshold,_maxNumberFilters.value(),_maxNumberMeasurements.value());
        log(Debug) << "(EstimationRBBM) dataAssociationFilter created " << endlog();

        /************************
        // make system model: a constant level'th derivative model
        ************************/
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) make system model " << endlog();
#endif
        vector<Matrix> AB(1);
    
        ColumnVector sysNoiseVector = ColumnVector(_level.value()+1);
        sysNoiseVector = 0.0;
        SymmetricMatrix sysNoiseMatrixOne = SymmetricMatrix(_level.value() +1);
        sysNoiseMatrixOne = 0.0;
        Matrix sysNoiseMatrixNonSymOne = Matrix(_level.value()+1,_level.value()+1);  
        sysNoiseMatrixNonSymOne = 0.0;
        for(int i =0 ; i<=_level.value(); i++) 
        {
            sysNoiseVector(i+1) = pow(this->getPeriod(),_level.value()-i+1)/double(factorial(_level.value()-i+1));
        }
        sysNoiseMatrixNonSymOne =  (sysNoiseVector* sysNoiseVector.transpose()) * _sysNoiseCovarianceValue.value() ;
        sysNoiseMatrixNonSymOne.convertToSymmetricMatrix(sysNoiseMatrixOne);

        SymmetricMatrix sysNoiseMatrix = SymmetricMatrix(_dimension);
        sysNoiseMatrix = 0.0;
        for(int i =0 ; i<=_level.value(); i++) 
        {
            for(int j =0 ; j<=_level.value(); j++)
            {
                sysNoiseMatrix(i*2+1,j*2+1)=sysNoiseMatrixOne(i+1,j+1);
                sysNoiseMatrix(i*2+2,j*2+2)=sysNoiseMatrixOne(i+1,j+1);
            }
        }

        Matrix sysModelMatrix = Matrix(_dimension,_dimension);  
        sysModelMatrix = 0.0;
        ColumnVector sysNoiseMean = ColumnVector(_dimension);
        sysNoiseMean = _sysNoiseMeanValue.value();
        for(int i =0 ; i<=_level.value(); i++) 
        {
            sysModelMatrix(i*2+1,i*2+1)=1.0;
            sysModelMatrix(i*2+2,i*2+2)=1.0;
            for(int j =i+1 ; j<=_level.value(); j++)
            {
                sysModelMatrix(i*2+1,j*2+1)=pow(this->getPeriod(),j-i)/double(factorial(j-i));
                sysModelMatrix(i*2+2,j*2+2)=pow(this->getPeriod(),j-i)/double(factorial(j-i));
            }
        }

        AB[0]= sysModelMatrix;

        Gaussian system_Uncertainty(sysNoiseMean, sysNoiseMatrix);
        _sysPdf = new LinearAnalyticConditionalGaussian(AB, system_Uncertainty);
        _sysModel = new LinearAnalyticSystemModelGaussianUncertainty(_sysPdf);
        log(Debug) << "(EstimationRBBM) made system model " << endlog();
        
        /**************************************
        Construct Measurement model = RBBM
        ***************************************/
#ifndef NDEBUG
        log(Debug) << "Make measurement model" << endlog();
        log(Debug) << "radiusState " << _radiusState.value() << endlog();
#endif
        _RBBM = new RBBMCartesian(_p.value(),_zmax.value(),_sigma_m.value(),(Probability)(_pi3.value()),(Probability)(_pi4.value()),_robotState.value(),_environment_file.value(),_nbeams.value(),_radiusState.value());
        _measModel = new MeasurementModel<ColumnVector,ColumnVector>(_RBBM);

#ifndef NDEBUG
        log(Debug) << "Measurement model constructed  " <<  endlog(); 
#endif


        /************************
        // resize variables 
        ************************/
        _stateFilters.resize(_maxNumberFilters.value());
        _stateFilters.assign(_maxNumberFilters.value(),ColumnVector(_dimension));
        _covarianceFilters.resize(_maxNumberFilters.value());
        _covarianceFilters.assign(_maxNumberFilters.value(),SymmetricMatrix(_dimension));
        _filterID.resize(_maxNumberFilters.value());
        _filterID_int.resize(_maxNumberFilters.value());

        _closestTargetState.resize(_dimension);
        
        _MxFilters.resize(_maxNumberFilters.value());
        _MyFilters.resize(_maxNumberFilters.value());

        _priorVBClustersParameters.allocate(_maxNumberFilters.value());

        _timings.resize(1e4,6);

        /************************
        // connect methods with peers 
        ************************/
        log(Debug) << "connect methods" << endlog();
        bool result =  (this->hasPeer("VBClusters") && this->hasPeer("NumberObjectsEstimator") );
        if (_reportingNeeded)
        {
            result = result && this->hasPeer("Reporter");
            result = result && this->hasPeer("ReporterTimings");
        }
        if (_prepareMeasurementNeeded)
            result = result && this->hasPeer("PrepareSeparatedMeasurement");
        if (!result)
        {
            log(Error) << "EstimationRBBM does not have expected peers" << endlog();
            log(Error) << "VBClusters " << (this->hasPeer("VBClusters")) <<endlog();
            log(Error) << "NumberObjectsEstimator " << (this->hasPeer("NumberObjectsEstimator")) <<endlog();
            if (_prepareMeasurementNeeded)
                log(Error) << "PrepareSeparatedMeasurement " << (this->hasPeer("PrepareSeparatedMeasurement")) <<endlog();
            if (_reportingNeeded)
            {
                log(Error) << "Reporting " << (this->hasPeer("Reporter")) <<endlog();
                log(Error) << "Reporting Timings" << (this->hasPeer("ReporterTimings")) <<endlog();
            }
            return result;
        }
#ifndef NDEBUG
        log(Debug) << "EstimationRBBM has expected peers " << endlog();
#endif
        if (_prepareMeasurementNeeded)
            prepareMeasurement = this->getPeer("PrepareSeparatedMeasurement")->methods()->getMethod<void(void)>("prepareMeasurement");
        findClusters = this->getPeer("VBClusters")->methods()->getMethod<OCL::VBClusters::Cluster_parameters(void)>("findClusters");
        estimateNumObjects = this->getPeer("NumberObjectsEstimator")->methods()->getMethod<void(void)>("estimateNumObjects");
        if (_reportingNeeded)
        {
            snapshotReporting = this->getPeer("Reporter")->methods()->getMethod<void(void)>("snapshot");
            snapshotReportingTimings = this->getPeer("ReporterTimings")->methods()->getMethod<void(void)>("snapshot");
        }
        result = (result &&   findClusters.ready() && estimateNumObjects.ready());
        if (_reportingNeeded)
        {
            result = (result &&  snapshotReporting.ready() );
            result = (result &&  snapshotReportingTimings.ready() );
        }
        if (_prepareMeasurementNeeded)
            result = (result &&  prepareMeasurement.ready() );
        if (!result)
        {
            log(Error) << "failed to configure  methods " << endlog();
            if (_prepareMeasurementNeeded)
                log(Error) << "prepareMeasurement " << prepareMeasurement.ready() <<endlog();
            log(Error) << "findClusters " << findClusters.ready() <<endlog();
            log(Error) << "estimateNumObjects " << estimateNumObjects.ready() <<endlog();
            if (_reportingNeeded)
            {
                log(Error) << "snapshotReporting " << snapshotReporting.ready() <<endlog();
                log(Error) << "snapshotReportingTimings " << snapshotReportingTimings.ready() <<endlog();
            }
            return result;
        }
        
        log(Debug) << "Result of configure hook: " << result << endlog();
        // TODO: check if ports are connected
        return result;
    }

    bool EstimationRBBM::startHook()
    {
        log(Debug) << "(EstimationRBBM) StartHook entered" << endlog();
        bool result = true;
        if (_prepareMeasurementNeeded)
        {
            prepareMeasurement();
            log(Debug) << "(EstimationRBBM) Measurement prepared" << endlog();
        }
        calculatePriorVBClustersParameters();
        findClusters();
        log(Debug) << "(EstimationRBBM) Clusters found" << endlog();
        estimateNumObjects();
        log(Debug) << "(Estimation) Number of objects estimated" << endlog();
        
        // create PF for found clusters
        // Get clusters of ports
        _stateClustersPort.Get(_clusters_loc);
        _numClustersPort.Get(_numClusters);
        // put clusters in other format
        // TODO: remove this allocation
        log(Debug) << "(EstimationRBBM) number Clusters found" << _numClusters<< endlog();
        vector<ColumnVector> clusters(_numClusters);
        clusters.assign(_numClusters,ColumnVector(_dimension));
        for (int cluster=0 ; cluster<_numClusters; cluster++)
        {
            clusters[cluster]= _clusters_loc[cluster];
            log(Debug) << "(EstimationRBBM) clusters[" << cluster << "]= " << clusters[cluster]<< endlog();

        }
        // Get measurements of ports
        _measurementsObjectsPort.Get(_measurements_loc);
        _numMeasurementsObjectsPort.Get(_numMeasurements);
        // put measurements in other format
        // TODO: remove this allocation
        vector<ColumnVector> measurements(_numMeasurements);
        measurements.assign(_numMeasurements,ColumnVector(_dimension));
        for (int meas=0 ; meas<_numMeasurements; meas++)
        {
            measurements[meas]= _measurements_loc[meas];
            log(Debug) << "(EstimationRBBM) measurement[" << meas << "]= " << measurements[meas]<< endlog();

        }

/*
        // Start new filters
        int num_features = clusters.size();
        for(int feature=0 ; feature< num_features; feature++)
        {   
            // add new filter
            _priorMu(1) = measurements[feature](1);
            _priorMu(2) = measurements[feature](2);
            _priorCont.ExpectedValueSet(_priorMu);

            ExtendedKalmanFilter* new_filter =  new ExtendedKalmanFilter(&_priorCont);
            _dataAssociationFilterMCPdf->AddFilter(new_filter);
            
        }
*/

        updateFilters();
        log(Debug) << "(EstimationRBBM) filters updated" << endlog();
        if (_reportingNeeded)
            snapshotReporting();
        log(Debug) << "(EstimationRBBM) after snapshot" << endlog();

        time_begin = TimeService::Instance()->getTicks();
        _counterUpdateHook=0;
        return true;
    }

    void EstimationRBBM::updateHook()
    {
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "(EstimationRBBM) UpdateHook entered" << endlog();
#endif
        if (_prepareMeasurementNeeded)
        {
            prepareMeasurement();
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) Measurements read " << endlog();
#endif
        }
        time_passedPrepare = TimeService::Instance()->secondsSince(time_begin);

        calculatePriorVBClustersParameters();
        findClusters();
        time_passedClustering = TimeService::Instance()->secondsSince(time_begin);
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) Clusters found" << endlog();
#endif
        estimateNumObjects();
        time_passedNumObjects = TimeService::Instance()->secondsSince(time_begin);
#ifndef NDEBUG
        log(Debug) << "(Estimation) Number of objects estimated" << endlog();
#endif
        updateFilters();
        time_passedUpdateFilters = TimeService::Instance()->secondsSince(time_begin);
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) Filters updated" << endlog();
#endif
        if (_reportingNeeded)
            snapshotReporting();
        time_passedReporting = TimeService::Instance()->secondsSince(time_begin);
#ifndef NDEBUG
        //log(Debug) << "(EstimationRBBM) UpdateHook finished" << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
#endif
        time_passedUpdateHook = TimeService::Instance()->secondsSince(time_begin);
        _timings(_counterUpdateHook+1,0+1)=time_passedPrepare;
        _timings(_counterUpdateHook+1,1+1)=time_passedClustering;
        _timings(_counterUpdateHook+1,2+1)=time_passedNumObjects;
        _timings(_counterUpdateHook+1,3+1)=time_passedUpdateFilters;
        _timings(_counterUpdateHook+1,4+1)=time_passedReporting;
        _timings(_counterUpdateHook+1,5+1)=time_passedUpdateHook;
        time_begin = TimeService::Instance()->getTicks();
        _counterUpdateHook++;
    }

    void EstimationRBBM::stopHook()
    {
        this->writeTimings();
    }


    void EstimationRBBM::cleanUpHook()
    {
    }

    bool EstimationRBBM::updateFilters()
    {
#ifndef NDEBUG
        log(Debug) << "--------------------------" << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) UpdateFilters entered" << endlog();
#endif
        // Get clusters of ports
        _stateClustersPort.Get(_clusters_loc);
        // put clusters in other format
        _numClustersPort.Get(_numClusters);
        // Get measurements of ports
        _measurementsObjectsPort.Get(_measurements_loc);
        // put measurements in other format
        _numMeasurementsObjectsPort.Get(_numMeasurements);

        double distance=100000;
        _distanceClosestTarget = distance;
        _closestTargetState = 0.0;

        vector<ColumnVector> clusters(_numClusters);
        clusters.assign(_numClusters,ColumnVector(_dimension));
        for (int cluster=0 ; cluster<_numClusters; cluster++)
        {
            clusters[cluster]= _clusters_loc[cluster];
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) clusters[" << cluster << "]= " << clusters[cluster]<< endlog();
#endif
        }
        vector<ColumnVector> measurements(_numMeasurements);
        measurements.assign(_numMeasurements,ColumnVector(_dimension));
        for (int meas=0 ; meas<_numMeasurements; meas++)
        {
            measurements[meas]= _measurements_loc[meas];
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) measurement[" << meas << "]= " << measurements[meas]<< endlog();
#endif
        }

        // print out data of posterior
        vector<Pdf<ColumnVector>* > posts = _dataAssociationFilterMCPdf->PostGet();
#ifndef NDEBUG
        for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet(); teller_post++)
        {
            log(Debug) << "(EstimationRBBM) teller_post " << teller_post << endlog();
            log(Debug) << "(EstimationRBBM) filter expected value"<<  posts[teller_post]->ExpectedValueGet()<<endlog();
            log(Debug) << "(EstimationRBBM) filter covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
        }
#endif
         // do system update
         _dataAssociationFilterMCPdf->Update(_sysModel);
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) System update done" << endlog();
        // print out data of posterior
        posts = _dataAssociationFilterMCPdf->PostGet();
        for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet(); teller_post++)
        {
            log(Debug) << "(EstimationRBBM) teller_post " << teller_post << endlog();
            log(Debug) << "(EstimationRBBM) filter expected value"<<  posts[teller_post]->ExpectedValueGet()<<endlog();
            log(Debug) << "(EstimationRBBM) filter covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
        }
#endif

        if(_numMeasurements>0)
        {
           // start new filters if necessary (i.e. if unexplained clusters)
#ifndef NDEBUG
           log(Debug) << "(EstimationRBBM) find new objects " << endlog();
#endif
           this->findNewObjects(clusters);
#ifndef NDEBUG
           log(Debug) << "(EstimationRBBM) new objects found" << endlog();
#endif

           // delete filters if numTarges > numEstimated
           while(_dataAssociationFilterMCPdf->NumFiltersGet() > _numObjectsPort.Get() )
           {
#ifndef NDEBUG
                log(Debug) << "num Filters > num estimated objects => delete most uncertain filter " << endlog();
#endif
                deleteMostUncertainObject();
           }

           vector<Pdf<ColumnVector>* > posts;
           // print out data of posterior
           posts = _dataAssociationFilterMCPdf->PostGet();
#ifndef NDEBUG
           for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet(); teller_post++)
           {
              log(Debug) << "(EstimationRBBM) teller_post " << teller_post << endlog();
              log(Debug) << "(EstimationRBBM) filter expected value"<<  posts[teller_post]->ExpectedValueGet()<<endlog();
              log(Debug) << "(EstimationRBBM) filter covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
           }
#endif
          // set the necessary variables
          // do measurement update
#ifndef NDEBUG
           log(Debug) << "(EstimationRBBM) Set Variables for measurement update " << endlog();
           log(Debug) << "(EstimationRBBM) _numClustersPort.Get() " << _numClustersPort.Get() << endlog();
#endif
            
           _dataAssociationFilterMCPdf->SetVariablesUpdateInternal(_responsibleClusterPort.Get(),_numClustersPort.Get() );
#ifndef NDEBUG
           log(Debug) << "(EstimationRBBM) Do Measurement update" << endlog();
#endif
           _dataAssociationFilterMCPdf->Update(_measModel,measurements);
#ifndef NDEBUG
           log(Debug) << "(EstimationRBBM) Measurement update done" << endlog();
#endif

#ifndef NDEBUG
           // print out data of posterior
           posts = _dataAssociationFilterMCPdf->PostGet();
           for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet(); teller_post++)
           {
              log(Debug) << "(EstimationRBBM) teller_post " << teller_post << endlog();
              log(Debug) << "(EstimationRBBM) filter expected value"<<  posts[teller_post]->ExpectedValueGet()<<endlog();
              log(Debug) << "(EstimationRBBM) filter covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
           }
#endif
        }
        // stop filters if necessary
        this->deleteOldObjects();

#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) Old object deleted" << endlog();
#endif

        // store and print out data of posterior
        posts = _dataAssociationFilterMCPdf->PostGet();
        _filterID_int =_dataAssociationFilterMCPdf->FilterIDGet(); 
        _filterID.assign(_maxNumberFilters.value(),0.0);
        for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet(); teller_post++)
        {
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) teller_post " << teller_post << endlog();
            log(Debug) << "(EstimationRBBM) filter expected value"<<  posts[teller_post]->ExpectedValueGet()<<endlog();
            log(Debug) << "(EstimationRBBM) filter covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
#endif
            _stateFilters[teller_post] = posts[teller_post]->ExpectedValueGet();
            _covarianceFilters[teller_post] = posts[teller_post]->CovarianceGet();
            _MxFilters[teller_post] = posts[teller_post]->ExpectedValueGet()(1);
            _MyFilters[teller_post] = posts[teller_post]->ExpectedValueGet()(2);
            _filterID[teller_post] = (double)_filterID_int[teller_post];
            distance = calculateDistanceToRobot(_stateFilters[teller_post]);
            if ( distance < _distanceClosestTarget )
            {
               _distanceClosestTarget = distance;
               _closestTargetState = _robotState.value() - _stateFilters[teller_post] ;
            }
        }
        
        _estimateStateFiltersPort.Set(_stateFilters);
        _estimateCovarianceFiltersPort.Set(_covarianceFilters);
        _numFiltersPort.Set(_dataAssociationFilterMCPdf->NumFiltersGet());
        _filterIDPort.Set(_filterID);
        _distanceClosestTargetPort.Set(_distanceClosestTarget);
        _closestTargetStatePort.Set(_closestTargetState);
        _estimateMxFiltersPort.Set(_MxFilters);
        _estimateMyFiltersPort.Set(_MyFilters);
        
#ifndef NDEBUG
        log(Debug) << "--------------------------" << endlog();
#endif
        return true;
    }

    bool EstimationRBBM::findNewObjects(vector<ColumnVector> clusters)
    {
    // add new filters for most unexplained clusters
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) findNewObjects entered" << endlog();
#endif
        // check if the estimated number of objects is greater than the current
        // number of filters
        int num_objects_estimated = _numObjectsPort.Get();
        int num_filters = _dataAssociationFilterMCPdf->NumFiltersGet();

        if(num_filters == 0) // add just one filter for the first cluster if no filters are present
        {
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) num_filters: " << num_filters << endlog();
            log(Debug) << "(EstimationRBBM) num_objects_estimated: " << num_objects_estimated << endlog();
            log(Debug) << "(EstimationRBBM) num_filters==0 " << endlog();
#endif
            // sort measurments according to most unexplained
            int num_features = clusters.size();
            if(num_features!=0) // there have to be some clusters
            {
#ifndef NDEBUG
                log(Debug) << "(EstimationRBBM) add filter for " << clusters[0] << endlog();
#endif
                // add new filter
                _priorMu(1) = clusters[0](1);
                _priorMu(2) = clusters[0](2);
                _priorCont.ExpectedValueSet(_priorMu);
#ifndef NDEBUG
                log(Debug) << "(EstimationRBBM) prior expected value" << _priorCont.ExpectedValueGet() << endlog();
                log(Debug) << "(EstimationRBBM) prior covariance" << _priorCont.CovarianceGet() << endlog();
#endif

                MCPdf<ColumnVector>* prior;
                vector<Sample<ColumnVector> > prior_samples(_numSamples.value());
                prior_samples.assign(_numSamples.value(),Sample<ColumnVector>(_dimension));

                _priorCont.SampleFrom(prior_samples,_numSamples.value(),CHOLESKY,NULL);
                prior= new MCPdf<ColumnVector>(_numSamples.value(), _dimension);
                prior->ListOfSamplesSet(prior_samples);
#ifndef NDEBUG
                log(Debug) << "prior->ExpectedValueGet() "<<prior->ExpectedValueGet() << endlog();
                log(Debug) << "Prior constructed  " <<  endlog(); 
#endif
                BootstrapFilter<ColumnVector,ColumnVector>* new_filter =  new BootstrapFilter<ColumnVector,ColumnVector>(prior,_resamplePeriod.value(),_resampleThreshold.value(),_resampleScheme.value());
                _dataAssociationFilterMCPdf->AddFilter(new_filter);
            }
            num_filters = _dataAssociationFilterMCPdf->NumFiltersGet();
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) new filter added" << _priorCont.CovarianceGet() << endlog();
#endif
        }
        
        while(num_filters < num_objects_estimated)
        {   
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) num_filters: " << num_filters << endlog();
            log(Debug) << "(EstimationRBBM) num_objects_estimated: " << num_objects_estimated << endlog();
            log(Debug) << "(EstimationRBBM) num_filters< num_objects_estimated " << endlog();
            log(Debug) << "(EstimationRBBM) sort measurements according to most unexplained " << endlog();
#endif
            int num_features = clusters.size();
            log(Debug) << "(EstimationRBBM) num_features: " << num_features << endlog();
            if(num_features==0) // there have to be some clusters, otherwise stop
            {
                cout << "entered break " << endl;
                break;
            
            }
            // sort measurments according to most unexplained
            vector<FeatureWithProb<ColumnVector> > featuresWithProb(num_features);
            _dataAssociationFilterMCPdf->SortMeasurements(_measModel,clusters,_s,featuresWithProb);

#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) measurements sorted " << endlog();
#endif
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) add filter for " << featuresWithProb[0].feature << endlog();
#endif
            // add new filter
            _priorMu(1) = featuresWithProb[0].feature(1);
            _priorMu(2) = featuresWithProb[0].feature(2);
            _priorCont.ExpectedValueSet(_priorMu);
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) prior expected value" << _priorCont.ExpectedValueGet() << endlog();
            log(Debug) << "(EstimationRBBM) prior covariance" << _priorCont.CovarianceGet() << endlog();
#endif

        MCPdf<ColumnVector>* prior;
        vector<Sample<ColumnVector> > prior_samples(_numSamples.value());
        prior_samples.assign(_numSamples.value(),Sample<ColumnVector>(_dimension));

        _priorCont.SampleFrom(prior_samples,_numSamples.value(),CHOLESKY,NULL);
        prior= new MCPdf<ColumnVector>(_numSamples.value(), _dimension);
        prior->ListOfSamplesSet(prior_samples);
#ifndef NDEBUG
        log(Debug) << "prior->ExpectedValueGet() "<<prior->ExpectedValueGet() << endlog();
        log(Debug) << "Prior constructed  " <<  endlog(); 
#endif

            //BootstrapFilter<ColumnVector,ColumnVector>* new_filter =  new BootstrapFilter<ColumnVector,ColumnVector>(prior);
            BootstrapFilter<ColumnVector,ColumnVector >* new_filter =  new BootstrapFilter<ColumnVector,ColumnVector >(prior,_resamplePeriod.value(),_resampleThreshold.value(),_resampleScheme.value());
            _dataAssociationFilterMCPdf->AddFilter(new_filter);
            num_filters = _dataAssociationFilterMCPdf->NumFiltersGet();
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) new filter added" << _priorCont.CovarianceGet() << endlog();
#endif
        }
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) num_filters: " << num_filters << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) findNewObjects finished" << endlog();
#endif
        return true;
    }

    bool EstimationRBBM::deleteMostUncertainObject()
    {
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) deleteMostUncertainObject entered" << endlog();
#endif
        vector<Pdf<ColumnVector>* > posts = _dataAssociationFilterMCPdf->PostGet();
        double value_mostUncertain = (posts[0]->CovarianceGet()(1,1) + posts[0]->CovarianceGet()(2,2)) * 0.5;
        int teller_mostUncertain = 0;
        for(int teller_post = 1 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet() ; teller_post++)
        {
            // check if covariance is higher than treshold
            double value_test = (posts[teller_post]->CovarianceGet()(1,1) + posts[teller_post]->CovarianceGet()(2,2)) * 0.5;
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) value_test "<< value_test  <<endlog();
#endif
            if( value_test > value_mostUncertain )
            {
                value_mostUncertain = value_test;
                teller_mostUncertain = teller_post;
#ifndef NDEBUG
                log(Debug) << "(EstimationRBBM) filter to delete found with covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
#endif
            }
        }
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) remove filter "  << teller_mostUncertain<< endlog();
#endif
        _dataAssociationFilterMCPdf->RemoveFilter(teller_mostUncertain);
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) deleteMostUncertainObject done" << endlog();
#endif
        return true;
    }

    bool EstimationRBBM::deleteOldObjects()
    {
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) deleteOldObjects entered" << endlog();
#endif
        vector<Pdf<ColumnVector>* > posts = _dataAssociationFilterMCPdf->PostGet();
        for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet() ; teller_post++)
        {
            // check if covariance is higher than treshold
            double value_test = (posts[teller_post]->CovarianceGet()(1,1) + posts[teller_post]->CovarianceGet()(2,2)) * 0.5;
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM) value_test "<< value_test  <<endlog();
#endif
            if( value_test > _tresholdDeleteFilter )
            {
#ifndef NDEBUG
                log(Debug) << "(EstimationRBBM) filter to delete found with covariance"<<  posts[teller_post]->CovarianceGet()<<endlog();
#endif
                _dataAssociationFilterMCPdf->RemoveFilter(teller_post);
            }
        }
#ifndef NDEBUG
        log(Debug) << "(EstimationRBBM) deleteOldObjects done" << endlog();
#endif
        return true;
    }

    double EstimationRBBM::calculateDistanceToRobot(ColumnVector targetState)
    {
        double distance = pow(targetState(1)-_robotState.value()(1) ,2.0) + pow(targetState(2)-_robotState.value()(2) ,2.0);
        return sqrt(distance);  
    }   

    bool EstimationRBBM::calculatePriorVBClustersParameters()
    {
#ifndef NDEBUG
            log(Debug) << "(EstimationRBBM::calculatePriorVBClustersParameters) entered " <<endlog();
#endif
        vector<Pdf<ColumnVector>* > posts = _dataAssociationFilterMCPdf->PostGet();
        ColumnVector state,pos(2);
        Matrix W(2,2);
        double x,y,radius,xt1,xt2,mt1,mt2,mf,xf,l1pow2,l2pow2;
        for(int teller_post = 0 ; teller_post < _dataAssociationFilterMCPdf->NumFiltersGet() ; teller_post++)
        {
            // Get expected value
            state = posts[teller_post]->ExpectedValueGet();
            cout << "state" << state << endl;
            pos(1) = state(1);
            pos(2) = state(2);
            cout << "pos " << pos << endl;
            x = state(1);
            y = state(2);
            // radius is the same for all for now _radiusState
            radius = _radiusState.value();
            cout << "radius " << radius << endl;
            W(1,1)=1.0/pow(radius,2.0);
            W(2,2)=1.0/pow(radius,2.0);
            _priorVBClustersParameters.M[teller_post] = pos;
            _priorVBClustersParameters.W[teller_post] = W;
        }
        _priorVBClustersParametersPort.Set(_priorVBClustersParameters);
        _numPriorClustersPort.Set(_dataAssociationFilterMCPdf->NumFiltersGet());
        return true;
    }

    int EstimationRBBM::factorial (int num)
    {
        if (num==1)
            return 1;
        return factorial(num-1)*num; // recursive call
    }

    bool EstimationRBBM::writeTimings()
    {
        if(_reportingNeeded)
        {
            _timingsPort.Set(_timings);
            snapshotReportingTimings();
            snapshotReportingTimings();
        }
        return true;
    }
}//namespace


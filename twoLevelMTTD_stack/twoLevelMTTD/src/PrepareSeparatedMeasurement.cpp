#include "PrepareSeparatedMeasurement.hpp"

#include <ocl/ComponentLoader.hpp>

//ORO_CREATE_COMPONENT( OCL::PrepareSeparatedMeasurement)
ORO_CREATE_COMPONENT_TYPE()

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     PrepareSeparatedMeasurement::PrepareSeparatedMeasurement(std::string name)
         : TaskContext(name,PreOperational)
          , _measurementsObjects("measurementsObjects")
          , _numMeasurementsObjects("numMeasurementsObjects")
          , _distancesObjects("distancesSeparatedObjects")
          , _anglesObjects("anglesSeparatedObjects")
          , _PrepareMeasurement("prepareMeasurement", &PrepareSeparatedMeasurement::prepareMeasurement, this)
     {
        // Check if all initialisation was ok:
        assert( _PrepareMeasurement.ready() );

        // Now add it to the interface:
        this->methods()->addMethod(&_PrepareMeasurement, "prepare a background subtracted measurement");

        this->ports()->addPort(&_measurementsObjects);
        this->ports()->addPort(&_numMeasurementsObjects);
        this->ports()->addPort(&_distancesObjects);
        this->ports()->addPort(&_anglesObjects);
     }

    PrepareSeparatedMeasurement::~PrepareSeparatedMeasurement()
    {
    }

    bool PrepareSeparatedMeasurement::configureHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurement) ConfigureHook entered" << endlog();
        
        bool result  = true;

        /************************
        // resize variables 
        ************************/

        /************************
        // connect methods with peers 
        ************************/
       // log(Debug) << "connect methods" << endlog();
       // bool result =  (this->hasPeer("VBClusters") && this->hasPeer("NumberObjectsEstimator") && this->hasPeer("ReaderGit") &&  this->hasPeer("Reporter") );
       // if (!result)
       // {
       //     log(Error) << "PrepareSeparatedMeasurement does not have expected peers" << endlog();
       //     log(Error) << "VBClusters " << (this->hasPeer("VBClusters")) <<endlog();
       //     log(Error) << "NumberObjectsEstimator " << (this->hasPeer("NumberObjectsEstimator")) <<endlog();
       //     log(Error) << "ReaderGit " << (this->hasPeer("ReaderGit")) <<endlog();
       //     log(Error) << "Reporting " << (this->hasPeer("Reporter")) <<endlog();
       //     return result;
       // }
       // log(Debug) << "PrepareSeparatedMeasurement has expected peers " << endlog();
       // readMeasurement = this->getPeer("ReaderGit")->methods()->getMethod<void(void)>("readMeasurement");
       // findClusters = this->getPeer("VBClusters")->methods()->getMethod<OCL::VBClusters::Cluster_parameters(void)>("findClusters");
       // estimateNumObjects = this->getPeer("NumberObjectsEstimator")->methods()->getMethod<void(void)>("estimateNumObjects");
       // snapshotReporting = this->getPeer("Reporter")->methods()->getMethod<void(void)>("snapshot");
       // result = (result && readMeasurement.ready() &&  findClusters.ready() && estimateNumObjects.ready() && snapshotReporting.ready());
       // if (!result)
       // {
       //     log(Error) << "failed to configure  methods " << endlog();
       //     log(Error) << "readMeasurement " << readMeasurement.ready() <<endlog();
       //     log(Error) << "findClusters " << findClusters.ready() <<endlog();
       //     log(Error) << "estimateNumObjects " << estimateNumObjects.ready() <<endlog();
       //     log(Error) << "snapshotReporting " << snapshotReporting.ready() <<endlog();
       //     return result;
       // }
       // 
       // log(Debug) << "Result of configure hook: " << result << endlog();
       return result;
    }

    bool PrepareSeparatedMeasurement::startHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurement) StartHook entered" << endlog();
        prepareMeasurement();
        return true;
    }

    void PrepareSeparatedMeasurement::updateHook()
    {
        prepareMeasurement();
    }

    void PrepareSeparatedMeasurement::stopHook()
    {
    }

    void PrepareSeparatedMeasurement::cleanUpHook()
    {
    }

}//namespace


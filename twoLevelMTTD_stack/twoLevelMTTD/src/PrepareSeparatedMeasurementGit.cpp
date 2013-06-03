#include "PrepareSeparatedMeasurementGit.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_LIST_COMPONENT_TYPE( OCL::PrepareSeparatedMeasurementGit)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     PrepareSeparatedMeasurementGit::PrepareSeparatedMeasurementGit(std::string name)
         : PrepareSeparatedMeasurement(name)
         , _measurementsObjectsGit("measurementsObjectsGit")
         , _numMeasurementsObjectsGit("numMeasurementsObjectsGit")
         , _distancesObjectsGit("distancesSeparatedObjectsGit")
         , _anglesObjectsGit("anglesSeparatedObjectsGit")
     {
        this->ports()->addPort(&_measurementsObjectsGit);
        this->ports()->addPort(&_numMeasurementsObjectsGit);
        this->ports()->addPort(&_distancesObjectsGit);
        this->ports()->addPort(&_anglesObjectsGit);
     }

    PrepareSeparatedMeasurementGit::~PrepareSeparatedMeasurementGit()
    {
    }

    bool PrepareSeparatedMeasurementGit::configureHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurementGit) ConfigureHook entered" << endlog();

        /************************
        // resize variables 
        ************************/

        /************************
        // connect methods with peers 
        ************************/
        log(Debug) << "connect methods" << endlog();
        bool result =  this->hasPeer("ReaderGit");
        if (!result)
        {
            log(Error) << "PrepareSeparatedMeasurementGit does not have expected peers" << endlog();
            log(Error) << "ReaderGit " << (this->hasPeer("ReaderGit")) <<endlog();
            return result;
        }
        log(Debug) << "PrepareSeparatedMeasurementGit has expected peers " << endlog();
        readMeasurement = this->getPeer("ReaderGit")->methods()->getMethod<void(void)>("readMeasurement");
        result = (result && readMeasurement.ready() );
        if (!result)
        {
            log(Error) << "failed to configure  methods " << endlog();
            log(Error) << "readMeasurement " << readMeasurement.ready() <<endlog();
            return result;
        }
        
        log(Debug) << "Result of configure hook: " << result << endlog();
        return result;
    }


    void PrepareSeparatedMeasurementGit::prepareMeasurement()
    {
        // call read measurement on git data reader
        readMeasurement();
        log(Debug) << "measurement read " << endlog();
        // put the data on the ports
        _measurementsObjects.Set(_measurementsObjectsGit.Get());
        _numMeasurementsObjects.Set(_numMeasurementsObjectsGit.Get());
        _distancesObjects.Set(_distancesObjectsGit.Get());
        _anglesObjects.Set(_anglesObjectsGit.Get());
    }


}//namespace


#include "PrepareSeparatedMeasurementLaserScanner.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_LIST_COMPONENT_TYPE( OCL::PrepareSeparatedMeasurementLaserScanner)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     PrepareSeparatedMeasurementLaserScanner::PrepareSeparatedMeasurementLaserScanner(std::string name)
         : PrepareSeparatedMeasurement(name)
         , _measurementsObjectsLaserScanner("measurementsObjectsSeparated")
         , _numMeasurementsObjectsLaserScanner("numMeasurementsObjectsSeparated")
         , _distancesObjectsLaserScanner("distancesSeparatedObjectsSeparated")
         , _anglesObjectsLaserScanner("anglesSeparatedObjectsSeparated")
     {
        this->ports()->addPort(&_measurementsObjectsLaserScanner);
        this->ports()->addPort(&_numMeasurementsObjectsLaserScanner);
        this->ports()->addPort(&_distancesObjectsLaserScanner);
        this->ports()->addPort(&_anglesObjectsLaserScanner);
     }

    PrepareSeparatedMeasurementLaserScanner::~PrepareSeparatedMeasurementLaserScanner()
    {
    }

    bool PrepareSeparatedMeasurementLaserScanner::configureHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurementLaserScanner) ConfigureHook entered" << endlog();

        /************************
        // resize variables 
        ************************/

        /************************
        // connect methods with peers 
        ************************/
        log(Debug) << "connect methods" << endlog();
        bool result =  this->hasPeer("Separator");
        if (!result)
        {
            log(Error) << "PrepareSeparatedMeasurementLaserScanner does not have expected peers" << endlog();
            log(Error) << "Separator " << (this->hasPeer("Separator")) <<endlog();
            return result;
        }
        log(Debug) << "PrepareSeparatedMeasurementLaserScanner has expected peers " << endlog();
        separateObjects = this->getPeer("Separator")->methods()->getMethod<void(void)>("separateObjects");
        result = (result &&  separateObjects.ready());
        if (!result)
        {
            log(Error) << "failed to configure  methods " << endlog();
            log(Error) << "separateObjects " << separateObjects.ready() <<endlog();
            return result;
        }
        
        log(Debug) << "Result of configure hook: " << result << endlog();
        return result;
    }


    void PrepareSeparatedMeasurementLaserScanner::prepareMeasurement()
    {
        // call read measurement on git data reader
        separateObjects();
        log(Debug) << "measurements not originated from environment separated " << endlog();
        // put the data on the ports
        _measurementsObjects.Set(_measurementsObjectsLaserScanner.Get());
        _numMeasurementsObjects.Set(_numMeasurementsObjectsLaserScanner.Get());
        _distancesObjects.Set(_distancesObjectsLaserScanner.Get());
        _anglesObjects.Set(_anglesObjectsLaserScanner.Get());
    }


}//namespace


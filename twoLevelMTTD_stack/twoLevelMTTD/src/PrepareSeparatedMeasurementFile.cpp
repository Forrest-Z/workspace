#include "PrepareSeparatedMeasurementFile.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_LIST_COMPONENT_TYPE( OCL::PrepareSeparatedMeasurementFile)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     PrepareSeparatedMeasurementFile::PrepareSeparatedMeasurementFile(std::string name)
         : PrepareSeparatedMeasurement(name)
         , _measurementsObjectsFile("measurementsObjectsSeparated")
         , _numMeasurementsObjectsFile("numMeasurementsObjectsSeparated")
         , _distancesObjectsFile("distancesSeparatedObjectsSeparated")
         , _anglesObjectsFile("anglesSeparatedObjectsSeparated")
     {
        this->ports()->addPort(&_measurementsObjectsFile);
        this->ports()->addPort(&_numMeasurementsObjectsFile);
        this->ports()->addPort(&_distancesObjectsFile);
        this->ports()->addPort(&_anglesObjectsFile);
     }

    PrepareSeparatedMeasurementFile::~PrepareSeparatedMeasurementFile()
    {
    }

    bool PrepareSeparatedMeasurementFile::configureHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurementFile) ConfigureHook entered" << endlog();

        /************************
        // resize variables 
        ************************/

        /************************
        // connect methods with peers 
        ************************/
        log(Debug) << "connect methods" << endlog();
        bool result =  this->hasPeer("Reader") && this->hasPeer("Separator");
        if (!result)
        {
            log(Error) << "PrepareSeparatedMeasurementFile does not have expected peers" << endlog();
            log(Error) << "Reader " << (this->hasPeer("Reader")) <<endlog();
            log(Error) << "Separator " << (this->hasPeer("Separator")) <<endlog();
            return result;
        }
        log(Debug) << "PrepareSeparatedMeasurementFile has expected peers " << endlog();
        readMeasurement = this->getPeer("Reader")->methods()->getMethod<void(void)>("readMeasurement");
        separateObjects = this->getPeer("Separator")->methods()->getMethod<void(void)>("separateObjects");
        result = (result && readMeasurement.ready() && separateObjects.ready());
        if (!result)
        {
            log(Error) << "failed to configure  methods " << endlog();
            log(Error) << "readMeasurement " << readMeasurement.ready() <<endlog();
            log(Error) << "separateObjects " << separateObjects.ready() <<endlog();
            return result;
        }
        
        log(Debug) << "Result of configure hook: " << result << endlog();
        return result;
    }


    void PrepareSeparatedMeasurementFile::prepareMeasurement()
    {
        // call read measurement on git data reader
        readMeasurement();
        log(Debug) << "measurement read " << endlog();
        separateObjects();
        log(Debug) << "measurements not originated from environment separated " << endlog();
        // put the data on the ports
        _measurementsObjects.Set(_measurementsObjectsFile.Get());
        _numMeasurementsObjects.Set(_numMeasurementsObjectsFile.Get());
        _distancesObjects.Set(_distancesObjectsFile.Get());
        _anglesObjects.Set(_anglesObjectsFile.Get());
    }


}//namespace


#include "PrepareSeparatedMeasurementSimulator.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_LIST_COMPONENT_TYPE( OCL::PrepareSeparatedMeasurementSimulator)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     PrepareSeparatedMeasurementSimulator::PrepareSeparatedMeasurementSimulator(std::string name)
         : PrepareSeparatedMeasurement(name)
         , _measurementsObjectsSimulator("measurementsObjectsSimulator")
         , _numMeasurementsObjectsSimulator("numMeasurementsObjectsSimulator")
         , _distancesObjectsSimulator("distancesSeparatedObjectsSimulator")
         , _anglesObjectsSimulator("anglesSeparatedObjectsSimulator")
     {
        this->ports()->addPort(&_measurementsObjectsSimulator);
        this->ports()->addPort(&_numMeasurementsObjectsSimulator);
        this->ports()->addPort(&_distancesObjectsSimulator);
        this->ports()->addPort(&_anglesObjectsSimulator);
     }

    PrepareSeparatedMeasurementSimulator::~PrepareSeparatedMeasurementSimulator()
    {
    }

    bool PrepareSeparatedMeasurementSimulator::configureHook()
    {
        log(Debug) << "(PrepareSeparatedMeasurementSimulator) ConfigureHook entered" << endlog();

        /************************
        // resize variables 
        ************************/

        /************************
        // connect methods with peers 
        ************************/
        log(Debug) << "connect methods" << endlog();
        bool result =  this->hasPeer("MeasurementSimulator");
        if (!result)
        {
            log(Error) << "PrepareSeparatedMeasurementSimulator does not have expected peers" << endlog();
            log(Error) << "MeasurementSimulator " << (this->hasPeer("MeasurementSimulator")) <<endlog();
            return result;
        }
        log(Debug) << "PrepareSeparatedMeasurementSimulator has expected peers " << endlog();
        //simulateMeasurements = this->getPeer("MeasurementSimulator")->methods()->getMethod<void(void)>("SimulateMeasurements");
        //result = (result && simulateMeasurements.ready() );
        //if (!result)
        //{
        //    log(Error) << "failed to configure  methods " << endlog();
        //    log(Error) << "simulateMeasurements " << simulateMeasurements.ready() <<endlog();
        //    return result;
        //}
        
        log(Debug) << "Result of configure hook: " << result << endlog();
        return result;
    }


    void PrepareSeparatedMeasurementSimulator::prepareMeasurement()
    {
        // call read measurement on MTTDMeasurementSimulator
        //simulateMeasurements();
        // put the data on the ports
        _measurementsObjects.Set(_measurementsObjectsSimulator.Get());
        _numMeasurementsObjects.Set(_numMeasurementsObjectsSimulator.Get());
        _distancesObjects.Set(_distancesObjectsSimulator.Get());
        _anglesObjects.Set(_anglesObjectsSimulator.Get());
    }


}//namespace


#ifndef __PREPARESEPARATEDMEASUREMENTSIMULATOR__
#define __PREPARESEPARATEDMEASUREMENTSIMULATOR__

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

#include <fstream>

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/bfl_constants.h>

#include <string>

#include "PrepareSeparatedMeasurement.hpp"

using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;

namespace OCL
{
    /***************
    PrepareSeparatedMeasurementSimulator
    * This class prepares a measurement with the background (environment)
    * subtracted such that it is ready to feed to the twolevel MTTD algorithm
    * (e.g. EstimationGaussian) by calling the SimulateMeasurements function on the
    * MTTDMeasurementSimulator (which simulates laser scanner measurements for
    * multiple targets moving in a prerecorded environment)
    *****************/
    class PrepareSeparatedMeasurementSimulator: public PrepareSeparatedMeasurement
//        : public TaskContext
    {

    private:
        /**
        * helper function to prepare a separated measurement
        */
        void        prepareMeasurement();

    protected:
        /**********
        PROPERTIES
        ***********/

        /**********
        DATA PORTS
        ***********/
        // Read data ports depend on the specific implementation

        /// Read Data port for the measurements of the read measurements
        ReadDataPort<std::vector<ColumnVector> >       _measurementsObjectsSimulator;
        /// Read Data port for the number of measurements read from the file
        ReadDataPort<int >                             _numMeasurementsObjectsSimulator;
        /// Read Data port for the distances of the read laser scanner measurement
        ReadDataPort<std::vector<double> >             _distancesObjectsSimulator;
        /// Read Data port for the angles of the read laser scanner measurement
        ReadDataPort<std::vector<double> >             _anglesObjectsSimulator;
        /**********
        METHODS 
        ***********/
        /// Simulate a measurement=> call passed on to the MTTDMeasurementSimulator peer 
        //Method<void(void)>                                  simulateMeasurements;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        PrepareSeparatedMeasurementSimulator(std::string name);
        /** 
        * Destructor
        */
        ~PrepareSeparatedMeasurementSimulator();
        bool        configureHook();
        
    };
}
#endif // __PREPARESEPARATEDMEASUREMENTSIMULATOR__

#ifndef __PREPARESEPARATEDMEASUREMENTLASERSCANNER__
#define __PREPARESEPARATEDMEASUREMENTLASERSCANNER__

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

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/bfl_constants.h>

#include <fstream>

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
    PrepareSeparatedMeasurementLaserScanner
    * This class prepares a measurement with the background (environment)
    * subtracted such that it is ready to feed to the twolevel MTTD algorithm
    * (e.g. EstimationGaussian) by calling the separateMeasurement function on
    * the SeparateObjects peer (which separates environment from non-environment
    * laser scanner measurements and is connected to a Laser Scanner component)
    *****************/
    class PrepareSeparatedMeasurementLaserScanner: public PrepareSeparatedMeasurement
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
        ReadDataPort<std::vector<ColumnVector> >        _measurementsObjectsLaserScanner;
        /// Read Data port for the number of measurements read from the file
        ReadDataPort<int >                              _numMeasurementsObjectsLaserScanner;
        /// Read Data port for the distances of the read laser scanner measurement
        ReadDataPort<std::vector<double> >              _distancesObjectsLaserScanner;
        /// Read Data port for the angles of the read laser scanner measurement
        ReadDataPort<std::vector<double> >              _anglesObjectsLaserScanner;
        /**********
        METHODS 
        ***********/
        /// Read a measurement=> call passed on to the ReadLaserScannerData peer 
        Method<void(void)>                                  separateObjects;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        PrepareSeparatedMeasurementLaserScanner(std::string name);
        /** 
        * Destructor
        */
        ~PrepareSeparatedMeasurementLaserScanner();
        bool        configureHook();
        
    };
}
#endif // __PREPARESEPARATEDMEASUREMENTLASERSCANNER__

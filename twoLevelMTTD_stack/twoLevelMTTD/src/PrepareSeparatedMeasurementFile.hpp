#ifndef __PREPARESEPARATEDMEASUREMENTFILE__
#define __PREPARESEPARATEDMEASUREMENTFILE__

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
    PrepareSeparatedMeasurementFile
    * This class prepares a measurement with the background (environment)
    * subtracted such that it is ready to feed to the twolevel MTTD algorithm
    * (e.g. EstimationGaussian) by first calling the readMeasurement function on the
    * ReadLaserScannerMeasurementsFile peer  (which reads a pre-recorded laser scanner measurement
    * from a file) and subsequently calling the separateMeasurement function on
    * the SeparateObjects peer (which separates environment from non-environment
    * laser scanner measurements)
    *****************/
    class PrepareSeparatedMeasurementFile: public PrepareSeparatedMeasurement
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
        ReadDataPort<std::vector<ColumnVector> >        _measurementsObjectsFile;
        /// Read Data port for the number of measurements read from the file
        ReadDataPort<int >                              _numMeasurementsObjectsFile;
        /// Read Data port for the distances of the read laser scanner measurement
        ReadDataPort<std::vector<double> >              _distancesObjectsFile;
        /// Read Data port for the angles of the read laser scanner measurement
        ReadDataPort<std::vector<double> >              _anglesObjectsFile;
        /**********
        METHODS 
        ***********/
        /// Separate environment and non-environment measurements => call passed on to the SeparateObjects peer 
        Method<void(void)>                                  separateObjects;
        /// Read a measurement=> call passed on to the ReadFile peer 
        Method<void(void)>                                  readMeasurement;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        PrepareSeparatedMeasurementFile(std::string name);
        /** 
        * Destructor
        */
        ~PrepareSeparatedMeasurementFile();
        bool        configureHook();
        
    };
}
#endif // __PREPARESEPARATEDMEASUREMENTFILE__

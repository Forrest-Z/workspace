#ifndef __PREPARESEPARATEDMEASUREMENT__
#define __PREPARESEPARATEDMEASUREMENT__

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

using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;

namespace OCL
{
    /***************
    PrepareSeparatedMeasurement
    * This abstract class prepares a measurement with the background (environment)
    * subtracted such that it is ready to feed to the twolevel MTTD algorithm
    * (e.g. EstimationGaussian)
    *****************/
    class PrepareSeparatedMeasurement
        : public TaskContext
    {

    private:
        /**
        * helper function to prepare a separated measurement: to be implemented
        * by the daughter classes
        */
        virtual void        prepareMeasurement(){};

    protected:
        /**********
        PROPERTIES
        ***********/

        /**********
        DATA PORTS
        ***********/
        // Read data ports depend on the specific implementation

        /// Write Data port for the measurements of the read measurements
        WriteDataPort<std::vector<ColumnVector> >       _measurementsObjects;
        /// Write Data port for the number of measurements read from the file
        WriteDataPort<int >                             _numMeasurementsObjects;
        /// Write Data port for the distances of the read laser scanner measurement
        WriteDataPort<std::vector<double> >             _distancesObjects;
        /// Write Data port for the angles of the read laser scanner measurement
        WriteDataPort<std::vector<double> >             _anglesObjects;

        /**********
        METHODS 
        ***********/
        /// Prepare a measurement
        Method<void(void)>                                  _PrepareMeasurement;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        PrepareSeparatedMeasurement(std::string name);
        /** 
        * Destructor
        */
        ~PrepareSeparatedMeasurement();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __PREPARESEPARATEDMEASUREMENT__

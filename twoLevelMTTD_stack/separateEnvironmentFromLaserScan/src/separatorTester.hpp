#ifndef __SEPARATORTESTER__
#define __SEPARATORTESTER__

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
#include <rtt/TimeService.hpp>
#include <rtt/RealTimeToolkit.hpp>

#include <ocl/OCL.hpp>
#include <ocl/FileReporting.hpp>

#include <fstream>
#include <string>


using std::ifstream;

using namespace std;
using namespace RTT;
using namespace Orocos;

namespace OCL
{

    /**
    Class to test SeparateObjects
     */
    class SeparatorTester
        : public TaskContext
    {

    private:
        
        /// Helper variable to check timing
        RTT::TimeService::ticks                         time_begin;
        /// Helper variable to check timing
        double                                          time_passed;

    protected:
        /// Read a measurement from a file
        Method<void(void)>                              readMeasurement;
        /// Separate the non-environment and environment measurements
        Method<void(void)>                              separateObjects;
        /// Take reporting snapshot
        Method<void(void)>                              snapshotReporting;

    public:
        /*
        * Constructor
        * @param name the name of the component
        */
        SeparatorTester(std::string name);
        /*
        * Destructor
        */
        ~SeparatorTester();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __SeparatorTester_

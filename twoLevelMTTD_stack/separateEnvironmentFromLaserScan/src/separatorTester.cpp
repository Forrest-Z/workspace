#include "separatorTester.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT(OCL::SeparatorTester)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     SeparatorTester::SeparatorTester(std::string name)
         : TaskContext(name,PreOperational)
     {
        // Check if all initialisation was ok:

        // Now add it to the interface:
    }

    SeparatorTester::~SeparatorTester()
    {
    }

    bool SeparatorTester::configureHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "configureHook entered" << endlog();

        bool result =  true;
        /**************************************
        Connect methods with peers
        ***************************************/
        result = result && this->hasPeer("Reader") ;
        readMeasurement = this->getPeer("Reader")->methods()->getMethod<void(void)>("readMeasurement");
        result = result && readMeasurement.ready();

        result = result && this->hasPeer("Separator") ;
        separateObjects = this->getPeer("Separator")->methods()->getMethod<void(void)>("separateObjects");
        result = result && separateObjects.ready() ;
        
        result = result && this->hasPeer("Reporter") ;
        snapshotReporting = this->getPeer("Reporter")->methods()->getMethod<void(void)>("snapshot");
        result = result && snapshotReporting.ready();

        log(Debug) << "Methods connected with peers " <<  endlog(); 

        log(Debug) << "configureHook finished" << endlog();
        log(Debug) << "Result of configureHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();

        
        return result;
    }

    bool SeparatorTester::startHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "startHook entered" << endlog();
        bool result = true;
        readMeasurement();
        log(Debug) << "Measurements read" << endlog();
        separateObjects();
        log(Debug) << "Objects separated" << endlog();
        
        snapshotReporting();
        log(Debug) << "Results reporter" << endlog();

        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "startHook finished" << endlog();
        log(Debug) << "Result of startHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return true;
    }

    void SeparatorTester::updateHook()
    {
        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "updateHook entered" << endlog();

        readMeasurement();
        log(Debug) << "Measurements read" << endlog();
        separateObjects();
        log(Debug) << "Objects separated" << endlog();
    
        snapshotReporting();
        log(Debug) << "Reporting snapshot taken " << endlog();
         
        time_passed = TimeService::Instance()->secondsSince(time_begin);
        log(Debug) << "Time passed " << time_passed <<  endlog();
        
        log(Debug) << "updateHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void SeparatorTester::stopHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "stopHook entered" << endlog();

        log(Debug) << "stopHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void SeparatorTester::cleanUpHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "cleanupHook entered" << endlog();

        log(Debug) << "cleanupHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }


}//namespace


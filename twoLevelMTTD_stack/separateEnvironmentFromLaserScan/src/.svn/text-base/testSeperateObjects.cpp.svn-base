
#include <rtt/Logger.hpp>
#include <rtt/os/main.h>
#include <iostream>
#include <rtt/TaskContext.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/NonPeriodicActivity.hpp>
#include <ocl/TaskBrowser.hpp>
#include "SeperateObjects.hpp"


using namespace std;
using namespace RTT;
using namespace Orocos;


int ORO_main(int argc, char** argv)
{
    Logger::In in("main()");

    // Set log level more verbose than default,
    // such that we can see output :
    if ( log().getLogLevel() < Logger::Info ) {
        log().setLogLevel( Logger::Info );
        log(Info) << argv[0] << " manually raises LogLevel to 'Info' (5). See also file 'orocos.log'."<<endlog();
    }

    log(Info) << "**** Creating the 'SeperateObjects' component ****" <<endlog();
    // Create the task:
    SeperateObjects seperateObjects("SeperateObjects");
    // Create the activity which runs the task's engine:
    // 1: Priority
    // 0.01: Period (100Hz)
    // hello.engine(): is being executed.
    NonPeriodicActivity act(1, seperateObjects.engine() );

    int nbeams = 181;
    
    std::vector<double> Z(nbeams);
    // read measurement data from file
    ifstream indata; // indata is like cin
    double num; // variable for input value

    indata.open("cpf/results_withpeople.txt" ); // opens the file
    if(!indata)  // file couldn't be opened
        cerr << "Error: file with measurementscould not be opened" << endl;

     indata >> num; // first one is time stamp
     int counter = 0;
     while ( counter < nbeams) { 
        indata >> num; // sets EOF flag if no value found
        Z[counter] = num;
        std::cout << num  <<std::endl;
        counter++;
     }
     indata.close();

    log(Info) << "**** Starting the 'SeperateObjects' component ****" <<endlog();
    // Start the component's activity:
    act.start();

    // Switch to user-interactive mode.
    TaskBrowser browser( &seperateObjects );

    // Accept user commands from console.
    browser.loop();

    return 0;
}

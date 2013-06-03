#ifndef __READGITDATA__
#define __READGITDATA__

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


#include <bfl/bfl_constants.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/pdf/mcpdf.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <stdlib.h>


using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;

namespace OCL
{

    /**
    * ReadGitData is a class to read data provided in the format of the BORG Lab
    * at Georgia Tech (www.kinetrack.org/experimental-data.html)
    */
    class ReadGitData
        : public TaskContext
    {

    private:
        /// Dimension of the read measurements
        int                                                     _dimension;
        /// Helper variable to store the distances read from the file
        std::vector<double>                                     _distances;
        /// Helper variable to store the angles read from the file
        std::vector<double>                                     _angles;
        /// The number of measurements
        int                                                     _numMeas;
        /// The time stamp of the measurement
        int                                                     _timeStamp;
        /// The time stamp currently considered
        int                                                     _currentTimestamp;
        /// Helper variable to store the measurements read from the file
        std::vector<ColumnVector>                               _measurements; 
        /// File stream to read from the file containing the x-measurements
        ifstream                                                _xDataStream;
        /// File stream to read from the file containing the y-measurements
        ifstream                                                _yDataStream;
        /// File stream to read from the file containing the time-measurements
        ifstream                                                _timeDataStream;
        /// Helper variable to count
        int teller;
        /// Helper variable to check the timing
        RTT::TimeService::ticks                                 time_begin;
        /// Helper variable to check the timing
        double                                                  time_passed;

        /*
        * Read a measurement from the text file
        */
        void readMeasurement();

    protected:
        /*PROPERTIES*/
        /// Name of the file containing the x-measurements
        Property<std::string>                           _xFile;
        /// Name of the file containing the y-measurements
        Property<std::string>                           _yFile;
        /// Name of the file containing the time-measurements
        Property<std::string>                           _timeFile;
        /// The maximum number of measurements (allocation)
        Property<int>                                   _maxNumMeas;
        /// Boolean indicating if the files are comma separated or not
        Property<bool>                                  _commaSeparated;
        /// Factor to multiply the read x and y measurements with (e.g. to convert pixel coordinates)
        Property<double>                                _conversionFactor;

        /*DATA PORTS*/
        /// Write Data port for the measurements of the read measurements
        WriteDataPort<std::vector<ColumnVector> >       _measurementsObjects;
        /// Write Data port for the number of measurements read from the file
        WriteDataPort<int >                             _numMeasurementsObjects;
        /// Write Data port for the distances of the read laser scanner measurement
        WriteDataPort<std::vector<double> >             _distancesObjects;
        /// Write Data port for the angles of the read laser scanner measurement
        WriteDataPort<std::vector<double> >             _anglesObjects;

        /*METHODS*/
        Method<void(void)>                              _readMeasurement;

    public:
        /*
        * Constructor
        * @param name the name of the component
        */
        ReadGitData(std::string name);
        /*
        * Destructor
        */
        ~ReadGitData();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __READGITDATA__

#include "ReadMeasurementsFile.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::ReadMeasurementsFile)

using namespace std;
using namespace RTT;
using namespace Orocos;

namespace OCL
{
     ReadMeasurementsFile::ReadMeasurementsFile(std::string name)
         : TaskContext(name,PreOperational),
          _measurements_file("measurements_file", "text file containing the laserscanner measurements"),
          _nbeams("nbeams","the number of beams"),
          _readMeasurement("readMeasurement", &ReadMeasurementsFile::readMeasurement,this),
          _distances("LaserDistance"),
          _angles("LaserAngle")
     {
        // Check if all initialisation was ok:
        assert( _measurements_file.ready() );
        assert( _nbeams.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_measurements_file);
        this->properties()->addProperty(&_nbeams);

        this->methods()->addMethod(&_readMeasurement, "Read a measurement from the file and put it on the ports");

        this->ports()->addPort(&_distances);
        this->ports()->addPort(&_angles);

        ////Read properties
        //if(!marshalling()->readProperties(_propertyfile))
        //  log(Error) <<"(ReadMeasurementsFile) Reading Properties from "<<_propertyfile<<" failed!!"<<endlog();
     }

    ReadMeasurementsFile::~ReadMeasurementsFile()
    {
        _measurementStream.close();
    }

    bool ReadMeasurementsFile::configureHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "configureHook entered" << endlog();
        bool result = true;
        _distances_local.resize(_nbeams.value());
        _angles_local.resize(_nbeams.value());

        _measurementStream.open( (_measurements_file.value()).c_str() ); // opens the file
        if(!_measurementStream)  // file couldn't be opened
        {
            log(Error) << "Error: file could not be opened" << endlog();
            log(Debug) << "(ReadMeasurementsFile) finished ConfigureHook() " << endlog();
            result = false;
            return result;
        }   
        log(Debug) << "configureHook finished" << endlog();
        log(Debug) << "Result of configureHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }
    bool ReadMeasurementsFile::startHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "startHook entered" << endlog();
        bool result = true;
        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "startHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    void ReadMeasurementsFile::readMeasurement()
    {
        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "readMeasurement entered" << endlog();
        if(!_measurementStream)  // file couldn't be opened
        {
            log(Error) << "Error: file could not be opened" << endlog();
        }   
        double num;
        _measurementStream >> num; // first one is time stamp
        log(Info) << "num " << num << endl;
        int counter = 0;
        while ( counter < _nbeams.value()) { 
           _measurementStream >> num; // sets EOF flag if no value found
           _distances_local[counter] = num;
           counter++;
        }
        counter = 0;
        while ( counter < _nbeams.value()) { 
           _measurementStream >> num; // sets EOF flag if no value found
           _angles_local[counter] = num;
           counter++;
        }
        _distances.Set(_distances_local);
        _angles.Set(_angles_local);

        time_passed = TimeService::Instance()->secondsSince(time_begin);
        log(Debug) << "Time passed " << time_passed <<  endlog();
        log(Debug) << "readMeasurement finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }
}//namespace


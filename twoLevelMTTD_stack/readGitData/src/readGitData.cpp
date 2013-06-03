#include "readGitData.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::ReadGitData )

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     ReadGitData::ReadGitData(std::string name)
         : TaskContext(name,PreOperational)
         , _xFile("xFile", "file containing the x positions of the measurements")
         , _yFile("yFile", "file containing the x positions of the measurements")
         , _timeFile("timeFile", "file containing the timestamps of the measurements")
         , _maxNumMeas("maxNumMeas","the maximum number of measurements ")
         , _commaSeparated("commaSeparated", "comma separated file or not" , 1)
         , _conversionFactor("conversionFactor", "factor to multiply x an y values" , 1)
         , _measurementsObjects("measurementsObjectsGit")
         , _numMeasurementsObjects("numMeasurementsObjectsGit")
         , _distancesObjects("distancesSeparatedObjectsGit")
         , _anglesObjects("anglesSeparatedObjectsGit")
         , _readMeasurement("readMeasurement", &ReadGitData::readMeasurement,this)
     {
        // Check if all initialisation was ok:
        assert( _xFile.ready() );
        assert( _yFile.ready() );
        assert( _timeFile.ready() );
        assert( _maxNumMeas.ready() );
        assert( _commaSeparated.ready() );
        assert( _conversionFactor.ready() );

        assert( _readMeasurement.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_xFile);
        this->properties()->addProperty(&_yFile);
        this->properties()->addProperty(&_timeFile);
        this->properties()->addProperty(&_maxNumMeas);
        this->properties()->addProperty(&_commaSeparated);
        this->properties()->addProperty(&_conversionFactor);

        this->ports()->addPort(&_measurementsObjects);
        this->ports()->addPort(&_numMeasurementsObjects);
        this->ports()->addPort(&_distancesObjects);
        this->ports()->addPort(&_anglesObjects);

        this->methods()->addMethod(&_readMeasurement, "Read a new measurement from the file");
    }

    ReadGitData::~ReadGitData()
    {
    }

    bool ReadGitData::configureHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "configureHook entered" << endlog();

        bool result =  true;
        /**************************************
        Read data from file and store
        ***************************************/
        _xDataStream.open( _xFile.value().c_str() );
        if(!_xDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: " <<  _xFile.value() <<" could not be opened" << endlog();
            log(Debug) << " finished ConfigureHook() " << endlog();
            result = false;
            return result;
        }   
        _yDataStream.open( _yFile.value().c_str() );
        if(!_xDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: " <<  _yFile.value() <<" could not be opened" << endlog();
            log(Debug) << " finished ConfigureHook() " << endlog();
            result = false;
            return result;
        }   
        _timeDataStream.open( _timeFile.value().c_str() );
        if(!_xDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: " <<  _timeFile.value() <<" could not be opened" << endlog();
            log(Debug) << " finished ConfigureHook() " << endlog();
            result = false;
            return result;
        }   

         // Already read the first timestamp
        _timeDataStream >> _timeStamp; //get timestamp
        _currentTimestamp = _timeStamp;
        
        /**************************************
        Set the state space dimension
        ***************************************/
        _dimension = 2;
        log(Debug) << "dimension: " << _dimension << endlog(); 


        /**************************************
        Resize variables
        ***************************************/
        _distances.resize(_maxNumMeas.value());
        _angles.resize(_maxNumMeas.value());
        _measurements.resize(_maxNumMeas.value());
        _measurements.assign(_maxNumMeas.value(),ColumnVector(2));

        log(Debug) << "_maxNumMeas.value() " << _maxNumMeas.value() <<  endlog(); 

        log(Debug) << "configureHook finished" << endlog();
        log(Debug) << "Result of configureHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    bool ReadGitData::startHook()
    {
        
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "startHook entered" << endlog();
        bool result = true;

        //readMeasurement();

        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "startHook finished" << endlog();
        log(Debug) << "Result of startHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    void ReadGitData::updateHook()
    {
        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "updateHook entered" << endlog();

        readMeasurement();
    
        time_passed = TimeService::Instance()->secondsSince(time_begin);
        log(Debug) << "Time passed " << time_passed <<  endlog();
        
        log(Debug) << "updateHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void ReadGitData::stopHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "stopHook entered" << endlog();

        log(Debug) << "stopHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void ReadGitData::cleanUpHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "cleanupHook entered" << endlog();

        _xDataStream.close();
        _yDataStream.close();

        log(Debug) << "***************************************" << endlog();
    }

    void ReadGitData::readMeasurement()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "readMeasurement entered" << endlog();
        if(!_xDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: file " << _xFile.value() << " could not be opened" << endlog();
        }   
        if(!_yDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: file " << _yFile.value() << " could not be opened" << endlog();
        }   
        if(!_timeDataStream)  // file couldn't be opened
        {
            log(Error) << "Error: file " << _timeFile.value() << " could not be opened" << endlog();
        }   

        //int currentTimeStamp = -1;
        //int timestamp = -1;
        int xnumMeas = 0;
        int ynumMeas = 0;
        _numMeas = 0;
        bool firstTime = true;
        _currentTimestamp = _timeStamp;
        while( firstTime ||  _timeStamp == _currentTimestamp) // still the same timestamp: add measurements
        {
            std::string xline, yline;
            double x,y;
            string        cell;
            getline(_xDataStream,xline);
            stringstream  xlineStream(xline);
            if (_commaSeparated.value() )
            { 
                while(std::getline(xlineStream,cell,',') )
                {
                    //cell >> x;
                    x = atof(cell.c_str());
                    _measurements[xnumMeas](1) = x * _conversionFactor.value();
                    //log(Debug) << "read x " << x << endlog();
                    //log(Debug) << "cell " << cell << endlog();
                    xnumMeas++;
                }
            }
            else
            {
                while(std::getline(xlineStream,cell,' ') )
                {
                    //cell >> x;
                    x = atof(cell.c_str());
                    _measurements[xnumMeas](1) = x * _conversionFactor.value();
                    //log(Debug) << "read x " << x << endlog();
                    //log(Debug) << "cell " << cell << endlog();
                    xnumMeas++;
                }
            }
            //log(Debug) << "numMeas " << _numMeas << endlog();

            getline(_yDataStream,yline);
            stringstream  ylineStream(yline);
            if (_commaSeparated.value() )
            { 
                while(std::getline(ylineStream,cell,',') )
                {
                    //cell >> x;
                    y = atof(cell.c_str());
                    _measurements[ynumMeas](2) = y * _conversionFactor.value();
                    //log(Debug) << "read y " << y << endlog();
                    //log(Debug) << "cell " << cell << endlog();
                    ynumMeas++;
                }
            }
            else
            {
                while(std::getline(ylineStream,cell,' '))
                {
                    //cell >> x;
                    y = atof(cell.c_str());
                    _measurements[ynumMeas](2) = y * _conversionFactor.value();
                    //log(Debug) << "read y " << y << endlog();
                    //log(Debug) << "cell " << cell << endlog();
                    ynumMeas++;
                }
            }
            _timeDataStream >> _timeStamp; //get timestamp
            if(firstTime)
            { 
                firstTime = false;
            }
            //log(Debug) << "timestamp " << _timeStamp << endlog();
            //log(Debug) << "currentTimeStamp " << _currentTimestamp << endlog();
        }
        //log(Debug) << "numMeas " << _numMeas << endlog();

        _numMeas = xnumMeas;

        if (!(xnumMeas == ynumMeas))
        {
            log(Error) << "The number of x positions read is not equal to the number of y positions read " << endlog();
        }
        _numMeas = xnumMeas;

        // calculate polar coordinates of _measurements: _distances and _angles
        for(int i=0; i<_numMeas ; i++)
        {
            _distances[i] = sqrt(pow(_measurements[i](1),2.0) + pow(_measurements[i](2),2.0) );
            _angles[i] = atan2(_measurements[i](2),_measurements[i](1));
        }
    
        /**************************************
        Put info on ports 
        ***************************************/
        _measurementsObjects.Set(_measurements);
        _numMeasurementsObjects.Set(_numMeas);
        _distancesObjects.Set(_distances);
        _anglesObjects.Set(_angles);

        log(Debug) << "readMeasurement finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

}//namespace


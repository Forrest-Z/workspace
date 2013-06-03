#include "SeparateObjects.hpp"
#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::SeparateObjects)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     SeparateObjects::SeparateObjects(std::string name)
         : TaskContext(name,PreOperational),
          _environment_file("environment_file", "text file containing the expected distance measurements of the environment"),
          _sigma_environment_detection("sigma_environment_detection", "the covariance of the distribution for environment detection"),
          _zmax("zmax", "the maximum distance measurement of the range finder"),
          _treshold("treshold", "the treshold probability from when a measurement is considered as part of the environment"),
          _pC1("pC1","The probability that a measurment is from the environment a priori"),
          _nbeams("nbeams","The number of beams in the environment measurement"),
          _separateObjects("separateObjects", &SeparateObjects::separateObjects, this),
          _getEnvironment("getEnvironment", &SeparateObjects::getEnvironment, this),
          _measurementsObjects("measurementsObjectsSeparated"),
          _numMeasurementsObjects("numMeasurementsObjectsSeparated"),
          _probMeasurementsEnv("probMeasurementsEnv"),
          _distancesSeparatedObjects("distancesSeparatedObjectsSeparated"),
          _anglesSeparatedObjects("anglesSeparatedObjectsSeparated"),
          _distances("LaserDistance"),
          _angles("LaserAngle"),
          _measurement(2)
     {
        // Check if all initialisation was ok:
        assert( _environment_file.ready() );
        assert( _sigma_environment_detection.ready() );
        assert( _zmax.ready() );
        assert( _treshold.ready() );
        assert( _pC1.ready() );
        assert( _nbeams.ready() );
        assert( _separateObjects.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_environment_file);
        this->properties()->addProperty(&_sigma_environment_detection);
        this->properties()->addProperty(&_zmax);
        this->properties()->addProperty(&_treshold);
        this->properties()->addProperty(&_nbeams);
        this->properties()->addProperty(&_pC1);

        this->methods()->addMethod(&_separateObjects, "Separate the object measurements from the environment");
        this->methods()->addMethod(&_getEnvironment, "Get the environment measurements");

        this->ports()->addPort(&_measurementsObjects);
        this->ports()->addPort(&_numMeasurementsObjects);
        this->ports()->addPort(&_probMeasurementsEnv);
        this->ports()->addPort(&_distancesSeparatedObjects);
        this->ports()->addPort(&_anglesSeparatedObjects);
        this->ports()->addPort(&_distances);
        this->ports()->addPort(&_angles);
    }
    

    SeparateObjects::~SeparateObjects()
    {
        delete _envGaussian;
    }

    bool  SeparateObjects::configureHook()
    {
        /************************
        * Resize class variables
        ************************/
        _prob.resize(_nbeams.value());
        _environment.resize(_nbeams.value());
        _distances_local.resize(_nbeams.value());
        _angles_local.resize(_nbeams.value());
        _measurements.resize(_nbeams.value());
        _measurements.assign(_nbeams.value(),ColumnVector(2));
        _distancesSeparated.resize(_nbeams.value());
        _anglesSeparated.resize(_nbeams.value());
        _measurement.resize(2);

        /************************
        *Create environment Gaussian
        ************************/
        MatrixWrapper::ColumnVector mean(1);
        mean(1) = 0.0; 
        MatrixWrapper::SymmetricMatrix sigma(1);
        sigma(1,1) = _sigma_environment_detection; 
        _envGaussian =  new Gaussian(mean,sigma);

        /************************
        *Read environment from file
        ************************/
        // File should have the following columns timestamp | laserdistance | laserangles
        ifstream indata; // indata is like cin
        double num; // variable for input value

        indata.open( (_environment_file.value()).c_str() ); // opens the file
        if(!indata)  // file couldn't be opened
        {
            log(Error) << "Failed to configure SeparateObjects: file " << _environment_file.value() << " could not be opened" << endlog();
            return false;
        }

        indata >> num; // first one is timestamp
        indata >> num;
        int counter = 0;
        while ( !indata.eof()  && counter < _nbeams.value()) { // keep reading until end-of-file
           _environment[counter] = num;
           log(Debug) << "_environment[" << counter << "]: " << num << endlog();
           indata >> num; // sets EOF flag if no value found
           counter++;
        }
        indata.close();


        return true;
     }

    void SeparateObjects::separateObjects()
    {
        log(Debug) << "[SeparateObjects] separateObjects() entered" << endlog();
        // get the measurements from the port
        _distances.Get(_distances_local);
        //log(Debug) << "[SeparateObjects] distances get" << endlog();
        _angles.Get(_angles_local);
        //log(Debug) << "[SeparateObjects] angles get" << endlog();
        // check if environement has as much beams as the measurement Z
        assert(_distances_local.size() == _nbeams.value());
        double tmp;
        MatrixWrapper::ColumnVector cv(1);
        int num_meas_non_env = 0;
        vector<int> indices_non_env(0);
        //log(Debug) << "[SeparateObjects] go over environment" << endlog();
        //log(Debug) << "[SeparateObjects] environment size " <<  _environment.size()<< endlog();
        //log(Debug) << "[SeparateObjects] distances_local size " <<  _distances_local.size()<< endlog();
        //log(Debug) << "[SeparateObjects] prob size " <<  _prob.size()<< endlog();
        for (unsigned int i= 0 ; i < _environment.size(); i++)
        {
            //log(Debug) << "[SeparateObjects] i" << i << endlog();
            //log(Debug) << "_distances_local[" << i <<"]:" << _distances_local[i]<< endlog();
            //log(Debug) << "_environment[" << i <<"]:" << _environment[i]<< endlog();
            cv(1) = _distances_local[i] - _environment[i];
            //log(Debug) << "cv(1): " << cv(1) << endlog();
            tmp = _envGaussian->ProbabilityGet(cv);
            //log(Debug) << "_envGaussian->ProbabilityGet: " << tmp << endlog();
            _prob[i] = tmp/(tmp + 1/_zmax*(1-_pC1) ); 
            //log(Debug) << "_prob[" << i << "]: " << _prob[i] << endlog();
            if(_prob[i] < _treshold && _distances_local[i] < 0.9 *_zmax.value())
                    // 0.9 =  factor to not get measurements too close to
                    // maximum
            {   
                //measurement does not come from the environment
                num_meas_non_env++;
                indices_non_env.push_back(i);
            }   
        }
        //_measurements.resize(num_meas_non_env);
        //_distancesSeparated.resize(num_meas_non_env);
        //_anglesSeparated.resize(num_meas_non_env);
        
        for (int k = 0 ; k<num_meas_non_env; k++)
        {
            _measurement(1) = _distances_local[indices_non_env[k]] * cos( _angles_local[ indices_non_env[k] ] );
            _measurement(2) = _distances_local[indices_non_env[k]] * sin( _angles_local[ indices_non_env[k] ] );
            _measurements[k] = _measurement;
            _distancesSeparated[k] =_distances_local[indices_non_env[k]] ;
            _anglesSeparated[k] =  _angles_local[ indices_non_env[k] ] ;
        }
        _measurementsObjects.Set(_measurements);
        _numMeasurementsObjects.Set(num_meas_non_env);
        _probMeasurementsEnv.Set(_prob);
        _distancesSeparatedObjects.Set(_distancesSeparated);
        _anglesSeparatedObjects.Set(_anglesSeparated);
        log(Debug) << "[SeparateObjects] number non environment measurements " << num_meas_non_env << endlog();  
        log(Debug) << "[SeparateObjects] separateObjects() finished" << endlog();
    }

    std::vector<double> SeparateObjects::getEnvironment()
    {
        return _environment;
    }
}//namespace


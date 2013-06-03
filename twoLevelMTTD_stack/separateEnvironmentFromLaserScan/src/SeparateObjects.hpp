#ifndef __SEPARATEOBJECTS__
#define __SEPARATEOBJECTS__

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Method.hpp>
#include <rtt/Command.hpp>
#include <rtt/Event.hpp>
#include <rtt/Ports.hpp>

#include <ocl/OCL.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/bfl_constants.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/bfl_constants.h>

#include <fstream>
using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace Orocos;

namespace OCL
{

    /** SeparateObjects is a class to get the non-environment measurements out
 * of laser scanner measurements for a static laser scanner. A pre-recorded 
 * static environment measurement
 * should be provided.
     */
    class SeparateObjects
        : public TaskContext
    {
    protected:
        /* PROPERTIES */
        /// The name of the text file containing the prerecorded environment
        Property<std::string>               _environment_file;
        /// The sigma of the Gaussian uncertainty around the distance measurement
        Property<double>                    _sigma_environment_detection;
        /// The maximum distance measured by the laser scanner
        Property<double>                    _zmax;
        /// The treshold for prob(z = environment measurement) from when a measurement is considered as part of the environment
        Property<double>                    _treshold;
        /// The a priori probability that a measurement belongs to the environment
        Property<double>                    _pC1;
        /// The number of beams in the laser scan
        Property<int>                       _nbeams;

        /* METHODS */
        /// Separate the non-environment and environment measurements
        Method<void(void) >                 _separateObjects;
        /// Read the static environment from a file
        Method<std::vector<double>(void) >  _getEnvironment;

        /* DATA PORTS */
        /// The non-environment measurements
        WriteDataPort<std::vector<ColumnVector> >   _measurementsObjects;
        /// The number of non-environment measurements
        WriteDataPort<int>                          _numMeasurementsObjects;
        /// The probability that the measurements belong to the environment
        WriteDataPort<std::vector<double> >         _probMeasurementsEnv;
        /// The distances of the non-environment measurements
        WriteDataPort<std::vector<double> >         _distancesSeparatedObjects;
        /// The angles of the non-environment measurements 
        WriteDataPort<std::vector<double> >         _anglesSeparatedObjects;
        /// The distances of the laser scanner measurements
        ReadDataPort<std::vector<double> >          _distances;
        /// The angles of the laser scanner measurements
        ReadDataPort<std::vector<double> >          _angles;

    private:
        /// Helper variable to store the probabilities that the measurements belong to the environment
        std::vector<double>         _prob; 
        /// The static environment read from a file
        std::vector<double>         _environment; 
        /// Helper variable to store the distances of the laser scanner measurements
        std::vector<double>         _distances_local; 
        /// Helper variable to store the angles of the laser scanner measurements
        std::vector<double>         _angles_local; 
        /// Helper variable to store the laser scanner measurements
        std::vector<ColumnVector>   _measurements; 
        /// Helper variable to store the distances of the non-environment laser scanner measurements
        std::vector<double>         _distancesSeparated; 
        /// Helper variable to store the angles of the non-environment laser scanner measurements
        std::vector<double>         _anglesSeparated; 
        /// Helper variable to store one measurement
        ColumnVector                _measurement; 
        /// The probability density function prob(measurement element of environement)
        Gaussian*                   _envGaussian;

        /*
        * Separate the non-environment and environment measurements
        */
        void                        separateObjects(); 
        /*
        * Read the prerecorded environment from the file
        * @return vector containing the environment measurements
        */
        std::vector<double>         getEnvironment();

    public:
        /*
        * Constructor
        * @param name the name of the component
        */
        SeparateObjects(std::string name);
        /*
        * Destructor
        */
        ~SeparateObjects();
        virtual bool configureHook();
        
    };
}
#endif // __SEPARATEOBJECTS__

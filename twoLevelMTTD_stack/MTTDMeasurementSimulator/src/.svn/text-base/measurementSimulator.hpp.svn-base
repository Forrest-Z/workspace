#ifndef __MEASUREMENT_SIMULATOR__
#define __MEASUREMENT_SIMULATOR__

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
#include <string>


using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;

namespace OCL
{

    /**
    This is a class to simulate laser scanner measurements for multiple circular
targets using a pre-recorded environment (static laser scanner)
     */
    class MeasurementSimulator
        : public TaskContext
    {

    private:
        /// ColumnVector containint the prerecorded environment 
        MatrixWrapper::ColumnVector _environment;
        /// Additive Gaussian System Model Uncertainty, for the target system
        //model
        Gaussian*                                               _sysUncertainty;
        /// The system model probability density function
        LinearAnalyticConditionalGaussian*                      _sysPdf;
        /// The system Model
        LinearAnalyticSystemModelGaussianUncertainty*           _sysModel;
        /// Dimension of the state of a single target
        int                                                     _dimension;
        /// Vector with the target states 
        std::vector<ColumnVector>                               _stateObjects;
        /// Vector containing the laser distance measurements 
        std::vector<double>                                     _distances;
        /// Vector containing the laser angles 
        std::vector<double>                                     _angles;
        /// Helper variable for the target state
        ColumnVector                                            _state;
        /// Helper variable for the relative position of the target 
        ColumnVector                                            _relPos;
        /// The number of measurements
        int                                                     _numMeas;
        /// Vector containing the measurements
        std::vector<ColumnVector>                               _measurements; 
        /// Helper variable to count
        int                                                     teller;
        /// to measure time needed for updateHook
        RTT::TimeService::ticks                                 time_begin;
        /// the time needed for updateHook
        double                                                  time_passed;


        /**
        * Simulate the measurements
        */
        void SimulateMeasurements();
        /**
        * Calculate the measurements
        */
        void CalculateMeasurements();
        /**
        * Update the targets' states
        */
        void UpdateObjects();
        /**
        * helper function calculating the factorial of an int 
        * @param a the integer of which to calculate the factorial
        * @return the factorial
        */
        int        factorial(int a);
    
        
    protected:
        /**********
        PROPERTIES
        ***********/
        /// File containing the pre-recorded static environment
        Property<std::string>                           _environment_file;
        /// Laser scanner position in the world
        Property<ColumnVector>                          _scannerPos;
        /// The number of laser beams
        Property<int>                                   _nbeams;
        /// The number of targets
        Property<int>                                   _nComp;
        /// The initial position of the targets
        Property<std::vector<ColumnVector> >            _mus;
        /// The maximum range measurement of a laser scanner
        Property<double>                                _rangeMax;
        /// The radius of the different targets (circular shape)
        Property<std::vector<double > >                 _radiusStates;
        /// The maximum number of targets (allocation)
        Property<int>                                   _maxNumComp;
        /// The maximum number of measurements (allocation)
        Property<int>                                   _maxNumMeas;
        /// The level of continuity of the system model 
        Property<int>                                   _level;
        /// The mean of the white noise on the target system model
        Property<double>                                _sysNoiseMeanValue;
        /// The covariance of the white noise on the target system model
        Property<double>                                _sysNoiseCovarianceValue;
        ///// The mean of the Gaussian system uncertainty
        //Property<ColumnVector>                          _muSysUncertainty;
        ///// The covariance of the Gaussian system uncertainty
        //Property<SymmetricMatrix>                       _sigmaSysUncertainty;
        ///// The system model matrix of the linear system model
        //Property<Matrix>                                _sysModelMatrix;
        /// Turn on reporting or not
        Property<bool>                                  _reporting;

        /**********
        DATA PORTS
        ***********/
        /// Write data port for the laser scanner measurements for the targets
        WriteDataPort<std::vector<ColumnVector> >       _measurementsObjects;
        /// Write data port for the number of laser scanner measurements for the targets
        WriteDataPort<int >                             _numMeasurementsObjects;
        /// Write data port for the number of distances of the laser scanner measurements for the targets
        WriteDataPort<std::vector<double> >             _distancesObjects;
        /// Write data port for the angles of distances of the laser scanner measurements for the targets
        WriteDataPort<std::vector<double> >             _anglesObjects;
        /// Write data port for the states of the targets
        WriteDataPort<std::vector<ColumnVector> >       _stateObjectsPort;
        /// Write data port for the number of targets
        WriteDataPort<int >                             _numObjectsPort;

        /**********
        METHODS 
        ***********/
        /// Take a snapshot with the reporter the number of targets => call passed on to the NumberObjectsEstimator peer 
        Method<void(void)>                              snapshotReporting;
        /// Simulate the measurements for the targets 
        Method<void(void)>                              _SimulateMeasurements;


    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        MeasurementSimulator(std::string name);
        /** 
        * Destructor
        */
        ~MeasurementSimulator();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __MEASUREMENT_SIMULATOR__

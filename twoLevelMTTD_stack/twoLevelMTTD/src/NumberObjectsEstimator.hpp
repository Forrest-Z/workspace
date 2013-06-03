#ifndef __NUMBEROBJECTSESTIMATOR__
#define __NUMBEROBJECTSESTIMATOR__

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Method.hpp>
#include <rtt/Command.hpp>
#include <rtt/Event.hpp>
#include <rtt/Ports.hpp>
#include <rtt/PeriodicActivity.hpp>

#include <ocl/OCL.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/bfl_constants.h>
#include <bfl/filter/histogramfilter.h>
#include <bfl/pdf/discreteconditionalpdf.h>
#include <bfl/pdf/discretepdf.h>

#include <fstream>

#include <boost/math/special_functions/digamma.hpp>

using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;
using namespace boost::math;

namespace OCL
{

    /**
    NumberObjectsEstimator
    * This class implements a discrete filter to estimate the number of targets
    * from the number of clusters (clustered measurements) 
     */
    class NumberObjectsEstimator
        : public TaskContext
    {

    public:


    private:
        ///Histogram filter to estimate the number of targets from the number of clusters
        HistogramFilter<int>*               _numObjectsFilter;
        /// The pdf underlying the measurement model
        DiscreteConditionalPdf*             _measPdf;
        /// The measurement model
        MeasurementModel<int,int>*          _measModel;
        /// The pdf underlying the system model
        DiscreteConditionalPdf*             _sysPdf;
        /// The system model
        SystemModel<int>*                   _sysModel;
        /// The prior for the number of targets
        DiscretePdf*                        _prior;
        /**
        * estimates the number of targets
        */
        void                                estimateNumObjects();
        // The probability of the number of clusters
        vector<double>                      _probObjects;

    protected:
        /* PROPERTIES */
        /// The maximum number of clusters
        Property<int>               _maxNumberClusters;
        /// The maximum number of targets
        Property<int>               _maxNumberObjects;
        /// The probability that the number of targets decreases with one (system model)
        Property<double>            _probSysOneObjectLess;
        /// The probability that the number of targets increases with one (system model)
        Property<double>            _probSysOneObjectMore;
        /// The probability that the number of targets decreases with two (system model)
        Property<double>            _probSysTwoObjectsLess;
        /// The probability that the number of targets increases with two (system model)
        Property<double>            _probSysTwoObjectsMore;

        /* METHODS */
        /// Estimates the number of targets 
        Method<void(void) >         _estimateNumObjects;

        /* DATA PORTS */
        /// Data port to write the estimated number of targets
        WriteDataPort<int >         _numObjectsPort;
        /// Data port to write the probability of the number of targets
        WriteDataPort<vector<double> > _probObjectsPort;
        /// Data port to read the number of clusters
        ReadDataPort<int>           _numClustersPort;

    public:
        /** 
        * Constructor
        * @param name the name of the component  
        */
        NumberObjectsEstimator(std::string name);
        /** 
        * Destructor
        */
        ~NumberObjectsEstimator();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}

#endif // __NUMBEROBJECTSESTIMATOR__

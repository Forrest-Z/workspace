#include "visualizationMeasSimPreprocessor.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::VIS_MEASSIMPREPROC)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     VIS_MEASSIMPREPROC::VIS_MEASSIMPREPROC(std::string name)
         : TaskContext(name,PreOperational)
         , _maxNumComp("maxNumComp","the maximum number of components possible (for reporting) ")
         , _numComp("numComp")
         , _compMeans("compMeans")
         , _numFilters("numFilters")
         , _estimateMxFilters("estimateMxFilters")
         , _estimateMyFilters("estimateMyFilters")
         , _filterID("filterID")

     {
        // Check if all initialisation was ok:
        assert( _maxNumComp.ready() );
        // Now add it to the interface:
        this->properties()->addProperty(&_maxNumComp);

        this->ports()->addPort(&_numComp);
        this->ports()->addPort(&_compMeans);

        this->ports()->addPort(&_numFilters);
        this->ports()->addPort(&_estimateMxFilters);
        this->ports()->addPort(&_estimateMyFilters);
        this->ports()->addPort(&_filterID);
    }

    VIS_MEASSIMPREPROC::~VIS_MEASSIMPREPROC()
    {
    }

    bool VIS_MEASSIMPREPROC::configureHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "configureHook entered" << endlog();

        bool result =  true;

        /**************************************
        Resize variables 
        ***************************************/
        _estimateX.resize(_maxNumComp.value());
        _estimateY.resize(_maxNumComp.value());
        _ids.resize(_maxNumComp.value());

        /**************************************
        Put prior info on ports 
        ***************************************/

        return result;
    }

    bool VIS_MEASSIMPREPROC::startHook()
    {
        
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "startHook entered" << endlog();
        bool result = true;
        log(Debug) << "startHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    void VIS_MEASSIMPREPROC::updateHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "updateHook entered" << endlog();
        for (int i =0 ; i<_numComp.Get(); i++)
        {
            // TODO:turn off?? SET ALL COMPOMENT WEIGHTS EQUAL
            _estimateX[i] = _compMeans.Get()[i](1);
            _estimateY[i] = _compMeans.Get()[i](2);
            _ids[i] = (double)(i+1);
        }
        _estimateMxFilters.Set(_estimateX);
        _estimateMyFilters.Set(_estimateY);
        _numFilters.Set(_numComp.Get());
        _filterID.Set(_ids);
        
        log(Debug) << "updateHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void VIS_MEASSIMPREPROC::stopHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "stopHook entered" << endlog();

        log(Debug) << "stopHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void VIS_MEASSIMPREPROC::cleanUpHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "cleanupHook entered" << endlog();

        log(Debug) << "cleanupHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

}//namespace


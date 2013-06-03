#ifndef __VIS_MEASSIMPREPROC__
#define __VIS_MEASSIMPREPROC__

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
#include <bfl/wrappers/matrix/matrix_wrapper.h>
//#include <bfl/pdf/gaussian.h>
//#include <bfl/pdf/mcpdf.h>
//#include <bfl/pdf/mixture.h>
//#include <bfl/filter/mixtureBootstrapFilter.h>
//#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
//#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <fstream>
#include <string>



using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace MatrixWrapper;
using namespace Orocos;
//using namespace boost::math;

namespace OCL
{

    /**
     */
    class VIS_MEASSIMPREPROC
        : public TaskContext
    {

    private:
        std::vector<double>                                     _estimateX;
        std::vector<double>                                     _estimateY;
        std::vector<double>                                     _ids;

        ////////////////////////////////////////////////////////////////////////////////

    protected:
        Property<int>                                  _maxNumComp;
        ReadDataPort<int >                             _numComp;
        ReadDataPort<std::vector<ColumnVector> >       _compMeans;

        WriteDataPort<int >                             _numFilters;
        WriteDataPort<std::vector<double> >             _estimateMxFilters;
        WriteDataPort<std::vector<double> >             _estimateMyFilters;
        WriteDataPort<std::vector<double> >             _filterID;

    public:
        VIS_MEASSIMPREPROC(std::string name);
        ~VIS_MEASSIMPREPROC();
        bool        configureHook();
        bool        startHook();
        void        updateHook();
        void        stopHook();
        void        cleanUpHook();
        
    };
}
#endif // __VIS_PREPROC__

#include "NumberObjectsEstimator.hpp"
#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::NumberObjectsEstimator)

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     NumberObjectsEstimator::NumberObjectsEstimator(std::string name)
         : TaskContext(name,PreOperational),
          _maxNumberClusters("maxNumberClusters","The maximum number of clusters "),
          _maxNumberObjects("maxNumberObjects","The maximum number of clusters "),
          _probSysOneObjectLess("probSysOneObjectLess","The system model probability that the number of obects decreases with one ",0.00000000002),
          _probSysOneObjectMore("probSysOneObjectMore","The system model probability that the number of obects increases with one ",0.0000100),
          _probSysTwoObjectsLess("probSysTwoObjectsLess","The system model probability that the number of obects decreases with two ",0.0),
          _probSysTwoObjectsMore("probSysTwoObjectsMore","The system model probability that the number of obects increases with two ",0.0),
          _estimateNumObjects("estimateNumObjects", &NumberObjectsEstimator::estimateNumObjects, this),
          _numObjectsPort("numObjectsEstimated"),
          _probObjectsPort("probObjects"),
          _numClustersPort("numClusters")
     {

#ifndef NDEBUG
        log(Debug) <<"(NumberObjectsEstimator) Entering constructor"<<endlog();
#endif
        // Check if all initialisation was ok:
        assert( _maxNumberClusters.ready() );
        assert( _maxNumberObjects.ready() );
        assert( _probSysOneObjectLess.ready() );
        assert( _probSysOneObjectMore.ready() );
        assert( _probSysTwoObjectsLess.ready() );
        assert( _probSysTwoObjectsMore.ready() );
        assert( _estimateNumObjects.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_maxNumberClusters);
        this->properties()->addProperty(&_maxNumberObjects);
        this->properties()->addProperty(&_probSysOneObjectLess);
        this->properties()->addProperty(&_probSysOneObjectMore);
        this->properties()->addProperty(&_probSysTwoObjectsLess);
        this->properties()->addProperty(&_probSysTwoObjectsMore);

        this->methods()->addMethod(&_estimateNumObjects, "Find the number of objects");

        this->ports()->addPort(&_numClustersPort);
        this->ports()->addPort(&_numObjectsPort);
        this->ports()->addPort(&_probObjectsPort);

        log(Debug) <<"(NumberObjectsEstimator) Leaving constructor"<<endlog();
     }

    NumberObjectsEstimator::~NumberObjectsEstimator()
    {
    }

    bool NumberObjectsEstimator::configureHook()
    {
        log(Debug) <<"(NumberObjectsEstimator) Entering configureHook"<<endlog();
        ////Read properties
        //if(!marshalling()->readProperties(_propertyfile))
        //  log(Error) <<"(NumberObjectsEstimator) Reading Properties from "<<_propertyfile<<" failed!!"<<endlog();

        int num_states = _maxNumberObjects.value();
        int num_meas_states = _maxNumberClusters.value();

        log(Debug) << "[NumberObjectsEstimator] num_states " << num_states << endlog();
        log(Debug) << "[NumberObjectsEstimator] num_meas_states " << num_meas_states << endlog();

        _probObjects.resize(num_states);

        //System model
        log(Debug) << "[NumberObjectsEstimator] constructing system model " << endlog();
        int cond_arg_dims[num_states];
        cond_arg_dims[0] = num_states;
        _sysPdf = new DiscreteConditionalPdf(num_states, 1,cond_arg_dims);
        log(Debug) << "[NumberObjectsEstimator] _sysPdf->NumStatesGet() " << _sysPdf->NumStatesGet() << endlog();

        std::vector<int> cond_args(1);

        //double minOne =  0.00000000007;
        //double plusOne = 0.000100;
        
        //double minOne =  0.00000000002;
        //double plusOne = 0.000010;
        if(_probSysOneObjectLess.value()<0 || _probSysOneObjectLess.value()>1)
        {
            log(Error) << "probSysOneObjectLess has to be positive and smaller than one" << endlog();
            return false;
        }
        if(_probSysOneObjectMore.value()<0 || _probSysOneObjectMore.value()>1)
        {
            log(Error) << "probSysOneObjectMore has to be positive and smaller than one" << endlog();
            return false;
        }
        if(_probSysTwoObjectsLess.value()<0 || _probSysTwoObjectsLess.value()>1)
        {
            log(Error) << "probSysTwoObjectsLess has to be positive and smaller than one" << endlog();
            return false;
        }
        if(_probSysTwoObjectsMore.value()<0 || _probSysTwoObjectsMore.value()>1)
        {
            log(Error) << "probSysTwoObjectsMore has to be positive and smaller than one" << endlog();
            return false;
        }
        double minOne = _probSysOneObjectLess.value();
        double plusOne = _probSysOneObjectMore.value();
        double minTwo =  _probSysTwoObjectsLess.value();
        double plusTwo = _probSysTwoObjectsMore.value();
        double mainValue = 1.0-minOne -minTwo - plusOne -plusTwo;
        if(mainValue < 0)
        {
            log(Error) << "the sum of probSysOneObjectMore, probSysOneObjectLess, probSysTwoObjectsMore and probSysTwoObjectsLess has to be smaller than 1" << endlog();
            return false;
        }
        for(int state = 0 ; state< num_states; state++)
        {
            for(int state_kmin = 0 ; state_kmin< num_states; state_kmin++)
            {
                cond_args[0] = state_kmin;
                //log(Debug) << "state" << state << endlog();
                //log(Debug) << "cond_args[0]" << cond_args[0] << endlog();
                //log(Debug) << "state - state_kmin " << state - state_kmin<< endlog();
                switch (state - state_kmin)
                {
                    case 0:
                     _sysPdf->ProbabilitySet(mainValue,state,cond_args);
                     break;
                    case -1:
                     _sysPdf->ProbabilitySet(minOne,state,cond_args);
                     break;
                    case +1:
                     _sysPdf->ProbabilitySet(plusOne,state,cond_args);
                     break;
                    case -2:
                     _sysPdf->ProbabilitySet(minTwo,state,cond_args);
                     break;
                    case +2:
                     _sysPdf->ProbabilitySet(plusTwo,state,cond_args);
                     break;
                    default:   
                     _sysPdf->ProbabilitySet(0.00,state,cond_args);
                }
            }
        }
        cond_args[0] = 0;
        _sysPdf->ProbabilitySet(mainValue+minTwo+minOne,0,cond_args);
        cond_args[0] = 1;
        _sysPdf->ProbabilitySet(mainValue+minTwo,1,cond_args);
        cond_args[0] = num_states-1;
        _sysPdf->ProbabilitySet(mainValue+plusTwo+plusOne,num_states-1,cond_args);
        cond_args[0] = num_states-2;
        _sysPdf->ProbabilitySet(mainValue+plusTwo,num_states-2,cond_args);

        _sysModel = new SystemModel<int>(_sysPdf);

        //Measurement model
        minOne = 0.200;
        minTwo = 0.100;
        double minThree = 0.100;
        plusOne = 0.200;
        plusTwo = 0.100;
        double plusThree = 0.000;
        mainValue = 1-minOne -minTwo -minThree - plusOne -plusTwo -plusThree;
        _measPdf = new DiscreteConditionalPdf(num_meas_states, 1,cond_arg_dims);
        for(int state = 0 ; state< num_meas_states; state++)
        {
            for(int state_kmin = 0 ; state_kmin< num_states; state_kmin++)
            {
                cond_args[0] = state_kmin;
                //log(Debug) << "state" << state << endlog();
                //log(Debug) << "cond_args[0]" << cond_args[0] << endlog();
                //log(Debug) << "state - state_kmin " << endlog();
                switch (state - state_kmin)
                {
                    case 0:
                     _measPdf->ProbabilitySet(mainValue,state,cond_args);
                     break;
                    case -1:
                     _measPdf->ProbabilitySet(minOne,state,cond_args);
                     break;
                    case +1:
                     _measPdf->ProbabilitySet(plusOne,state,cond_args);
                     break;
                    case -2:
                     _measPdf->ProbabilitySet(minTwo,state,cond_args);
                     break;
                    case -3:
                     _measPdf->ProbabilitySet(minThree,state,cond_args);
                     break;
                    case +2:
                     _measPdf->ProbabilitySet(plusTwo,state,cond_args);
                    case +3:
                     _measPdf->ProbabilitySet(plusThree,state,cond_args);
                     break;
                    default:   
                     _measPdf->ProbabilitySet(0.00,state,cond_args);
                }
            }
        }
        cond_args[0] = 0;
        _measPdf->ProbabilitySet(mainValue+minThree+minTwo+minOne,0,cond_args);
        cond_args[0] = 1;
        _measPdf->ProbabilitySet(mainValue+minThree+minTwo,1,cond_args);
        cond_args[0] = 2;
        _measPdf->ProbabilitySet(mainValue+minThree,1,cond_args);
        cond_args[0] = num_states-1;
        _measPdf->ProbabilitySet(mainValue+plusTwo+plusOne,num_states-1,cond_args);
        cond_args[0] = num_states-2;
        _measPdf->ProbabilitySet(mainValue+plusTwo,num_states-2,cond_args);

        // Set special probabilities
        cond_args[0] = 0;
        _measPdf->ProbabilitySet(0.80,0,cond_args);
        _measPdf->ProbabilitySet(0.14,1,cond_args);
        _measPdf->ProbabilitySet(0.05,2,cond_args);
        _measPdf->ProbabilitySet(0.01,3,cond_args);
        _measPdf->ProbabilitySet(0.00,4,cond_args);
        _measPdf->ProbabilitySet(0.00,5,cond_args);
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        cond_args[0] = 1;
        _measPdf->ProbabilitySet(0.30,0,cond_args);
        _measPdf->ProbabilitySet(0.40,1,cond_args);
        _measPdf->ProbabilitySet(0.24,2,cond_args);
        _measPdf->ProbabilitySet(0.05,3,cond_args);
        _measPdf->ProbabilitySet(0.01,4,cond_args);
        _measPdf->ProbabilitySet(0.00,5,cond_args);
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        cond_args[0] = 2;
        _measPdf->ProbabilitySet(0.20,0,cond_args);
        _measPdf->ProbabilitySet(0.30,1,cond_args);
        _measPdf->ProbabilitySet(0.35,2,cond_args);
        _measPdf->ProbabilitySet(0.14,3,cond_args);
        _measPdf->ProbabilitySet(0.05,4,cond_args);
        _measPdf->ProbabilitySet(0.01,5,cond_args);
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        cond_args[0] = 3;
        _measPdf->ProbabilitySet(0.10,0,cond_args);
        _measPdf->ProbabilitySet(0.16,1,cond_args);
        _measPdf->ProbabilitySet(0.26,2,cond_args);
        _measPdf->ProbabilitySet(0.28,3,cond_args);
        _measPdf->ProbabilitySet(0.14,4,cond_args);
        _measPdf->ProbabilitySet(0.05,5,cond_args);
        _measPdf->ProbabilitySet(0.01,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        cond_args[0] = 4;
        _measPdf->ProbabilitySet(0.05,0,cond_args);
        _measPdf->ProbabilitySet(0.10,1,cond_args);
        _measPdf->ProbabilitySet(0.15,2,cond_args);
        _measPdf->ProbabilitySet(0.23,3,cond_args);
        _measPdf->ProbabilitySet(0.27,4,cond_args);
        _measPdf->ProbabilitySet(0.14,5,cond_args);
        _measPdf->ProbabilitySet(0.05,6,cond_args);
        _measPdf->ProbabilitySet(0.01,7,cond_args);
        cond_args[0] = 5;
        _measPdf->ProbabilitySet(0.05,0,cond_args);
        _measPdf->ProbabilitySet(0.05,1,cond_args);
        _measPdf->ProbabilitySet(0.12,2,cond_args);
        _measPdf->ProbabilitySet(0.17,3,cond_args);
        _measPdf->ProbabilitySet(0.20,4,cond_args);
        _measPdf->ProbabilitySet(0.24,5,cond_args);
        _measPdf->ProbabilitySet(0.08,6,cond_args);
        _measPdf->ProbabilitySet(0.05,7,cond_args);
        _measPdf->ProbabilitySet(0.01,8,cond_args);
        cond_args[0] = 6;
        _measPdf->ProbabilitySet(0.05,0,cond_args);
        _measPdf->ProbabilitySet(0.05,1,cond_args);
        _measPdf->ProbabilitySet(0.13,2,cond_args);
        _measPdf->ProbabilitySet(0.14,3,cond_args);
        _measPdf->ProbabilitySet(0.16,4,cond_args);
        _measPdf->ProbabilitySet(0.18,5,cond_args);
        _measPdf->ProbabilitySet(0.20,6,cond_args);
        _measPdf->ProbabilitySet(0.05,7,cond_args);
        _measPdf->ProbabilitySet(0.04,8,cond_args);
        _measPdf->ProbabilitySet(0.01,9,cond_args);
        cond_args[0] = 7;
        _measPdf->ProbabilitySet(0.05,0,cond_args);
        _measPdf->ProbabilitySet(0.05,1,cond_args);
        _measPdf->ProbabilitySet(0.09,2,cond_args);
        _measPdf->ProbabilitySet(0.11,3,cond_args);
        _measPdf->ProbabilitySet(0.13,4,cond_args);
        _measPdf->ProbabilitySet(0.15,5,cond_args);
        _measPdf->ProbabilitySet(0.17,6,cond_args);
        _measPdf->ProbabilitySet(0.19,7,cond_args);
        _measPdf->ProbabilitySet(0.04,8,cond_args);
        _measPdf->ProbabilitySet(0.01,9,cond_args);
        _measPdf->ProbabilitySet(0.01,10,cond_args);
        cond_args[0] = 8;
        _measPdf->ProbabilitySet(0.05,0,cond_args);
        _measPdf->ProbabilitySet(0.05,1,cond_args);
        _measPdf->ProbabilitySet(0.09,2,cond_args);
        _measPdf->ProbabilitySet(0.11,3,cond_args);
        _measPdf->ProbabilitySet(0.13,4,cond_args);
        _measPdf->ProbabilitySet(0.14,5,cond_args);
        _measPdf->ProbabilitySet(0.15,6,cond_args);
        _measPdf->ProbabilitySet(0.16,7,cond_args);
        _measPdf->ProbabilitySet(0.17,8,cond_args);
        _measPdf->ProbabilitySet(0.01,9,cond_args);
        _measPdf->ProbabilitySet(0.01,10,cond_args);
        cond_args[0] = 9;
        _measPdf->ProbabilitySet(0.00,0,cond_args);
        _measPdf->ProbabilitySet(0.05,1,cond_args);
        _measPdf->ProbabilitySet(0.05,2,cond_args);
        _measPdf->ProbabilitySet(0.09,3,cond_args);
        _measPdf->ProbabilitySet(0.11,4,cond_args);
        _measPdf->ProbabilitySet(0.13,5,cond_args);
        _measPdf->ProbabilitySet(0.14,6,cond_args);
        _measPdf->ProbabilitySet(0.15,7,cond_args);
        _measPdf->ProbabilitySet(0.16,8,cond_args);
        _measPdf->ProbabilitySet(0.17,9,cond_args);
        _measPdf->ProbabilitySet(0.01,10,cond_args);
        _measPdf->ProbabilitySet(0.01,11,cond_args);
        cond_args[0] = 10;
        _measPdf->ProbabilitySet(0.00,0,cond_args);
        _measPdf->ProbabilitySet(0.00,1,cond_args);
        _measPdf->ProbabilitySet(0.05,2,cond_args);
        _measPdf->ProbabilitySet(0.05,3,cond_args);
        _measPdf->ProbabilitySet(0.09,4,cond_args);
        _measPdf->ProbabilitySet(0.11,5,cond_args);
        _measPdf->ProbabilitySet(0.13,6,cond_args);
        _measPdf->ProbabilitySet(0.14,7,cond_args);
        _measPdf->ProbabilitySet(0.15,8,cond_args);
        _measPdf->ProbabilitySet(0.16,9,cond_args);
        _measPdf->ProbabilitySet(0.17,10,cond_args);
        _measPdf->ProbabilitySet(0.01,11,cond_args);
        _measPdf->ProbabilitySet(0.01,12,cond_args);
        _measPdf->ProbabilitySet(0.01,13,cond_args);
        cond_args[0] = 11;
        _measPdf->ProbabilitySet(0.00,0,cond_args);
        _measPdf->ProbabilitySet(0.00,1,cond_args);
        _measPdf->ProbabilitySet(0.00,2,cond_args);
        _measPdf->ProbabilitySet(0.05,3,cond_args);
        _measPdf->ProbabilitySet(0.05,4,cond_args);
        _measPdf->ProbabilitySet(0.09,5,cond_args);
        _measPdf->ProbabilitySet(0.11,6,cond_args);
        _measPdf->ProbabilitySet(0.13,7,cond_args);
        _measPdf->ProbabilitySet(0.14,8,cond_args);
        _measPdf->ProbabilitySet(0.15,9,cond_args);
        _measPdf->ProbabilitySet(0.16,10,cond_args);
        _measPdf->ProbabilitySet(0.17,11,cond_args);
        _measPdf->ProbabilitySet(0.01,12,cond_args);
        _measPdf->ProbabilitySet(0.01,13,cond_args);
        _measPdf->ProbabilitySet(0.01,14,cond_args);
        cond_args[0] = 12;
        _measPdf->ProbabilitySet(0.00,1,cond_args);
        _measPdf->ProbabilitySet(0.00,2,cond_args);
        _measPdf->ProbabilitySet(0.00,3,cond_args);
        _measPdf->ProbabilitySet(0.05,4,cond_args);
        _measPdf->ProbabilitySet(0.05,5,cond_args);
        _measPdf->ProbabilitySet(0.09,6,cond_args);
        _measPdf->ProbabilitySet(0.11,7,cond_args);
        _measPdf->ProbabilitySet(0.13,8,cond_args);
        _measPdf->ProbabilitySet(0.14,9,cond_args);
        _measPdf->ProbabilitySet(0.15,10,cond_args);
        _measPdf->ProbabilitySet(0.16,11,cond_args);
        _measPdf->ProbabilitySet(0.17,12,cond_args);
        _measPdf->ProbabilitySet(0.01,13,cond_args);
        _measPdf->ProbabilitySet(0.01,14,cond_args);
        _measPdf->ProbabilitySet(0.01,15,cond_args);
        cond_args[0] = 13;
        _measPdf->ProbabilitySet(0.00,2,cond_args);
        _measPdf->ProbabilitySet(0.00,3,cond_args);
        _measPdf->ProbabilitySet(0.00,4,cond_args);
        _measPdf->ProbabilitySet(0.05,5,cond_args);
        _measPdf->ProbabilitySet(0.05,6,cond_args);
        _measPdf->ProbabilitySet(0.09,7,cond_args);
        _measPdf->ProbabilitySet(0.11,8,cond_args);
        _measPdf->ProbabilitySet(0.13,9,cond_args);
        _measPdf->ProbabilitySet(0.14,10,cond_args);
        _measPdf->ProbabilitySet(0.15,11,cond_args);
        _measPdf->ProbabilitySet(0.16,12,cond_args);
        _measPdf->ProbabilitySet(0.17,13,cond_args);
        _measPdf->ProbabilitySet(0.01,14,cond_args);
        _measPdf->ProbabilitySet(0.01,15,cond_args);
        _measPdf->ProbabilitySet(0.01,16,cond_args);
        cond_args[0] = 14;
        _measPdf->ProbabilitySet(0.00,3,cond_args);
        _measPdf->ProbabilitySet(0.00,4,cond_args);
        _measPdf->ProbabilitySet(0.00,5,cond_args);
        _measPdf->ProbabilitySet(0.05,6,cond_args);
        _measPdf->ProbabilitySet(0.05,7,cond_args);
        _measPdf->ProbabilitySet(0.09,8,cond_args);
        _measPdf->ProbabilitySet(0.11,9,cond_args);
        _measPdf->ProbabilitySet(0.13,10,cond_args);
        _measPdf->ProbabilitySet(0.14,11,cond_args);
        _measPdf->ProbabilitySet(0.15,12,cond_args);
        _measPdf->ProbabilitySet(0.16,13,cond_args);
        _measPdf->ProbabilitySet(0.17,14,cond_args);
        _measPdf->ProbabilitySet(0.01,15,cond_args);
        _measPdf->ProbabilitySet(0.01,16,cond_args);
        _measPdf->ProbabilitySet(0.01,17,cond_args);
        cond_args[0] = 15;
        _measPdf->ProbabilitySet(0.00,4,cond_args);
        _measPdf->ProbabilitySet(0.00,5,cond_args);
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.05,7,cond_args);
        _measPdf->ProbabilitySet(0.05,8,cond_args);
        _measPdf->ProbabilitySet(0.09,9,cond_args);
        _measPdf->ProbabilitySet(0.11,10,cond_args);
        _measPdf->ProbabilitySet(0.13,11,cond_args);
        _measPdf->ProbabilitySet(0.14,12,cond_args);
        _measPdf->ProbabilitySet(0.15,13,cond_args);
        _measPdf->ProbabilitySet(0.16,14,cond_args);
        _measPdf->ProbabilitySet(0.17,15,cond_args);
        _measPdf->ProbabilitySet(0.01,16,cond_args);
        _measPdf->ProbabilitySet(0.01,17,cond_args);
        _measPdf->ProbabilitySet(0.01,18,cond_args);
        cond_args[0] = 16;
        _measPdf->ProbabilitySet(0.00,5,cond_args);
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        _measPdf->ProbabilitySet(0.05,8,cond_args);
        _measPdf->ProbabilitySet(0.05,9,cond_args);
        _measPdf->ProbabilitySet(0.09,10,cond_args);
        _measPdf->ProbabilitySet(0.11,11,cond_args);
        _measPdf->ProbabilitySet(0.13,12,cond_args);
        _measPdf->ProbabilitySet(0.14,13,cond_args);
        _measPdf->ProbabilitySet(0.15,14,cond_args);
        _measPdf->ProbabilitySet(0.16,15,cond_args);
        _measPdf->ProbabilitySet(0.17,16,cond_args);
        _measPdf->ProbabilitySet(0.01,17,cond_args);
        _measPdf->ProbabilitySet(0.01,18,cond_args);
        _measPdf->ProbabilitySet(0.01,19,cond_args);
        cond_args[0] = 17;
        _measPdf->ProbabilitySet(0.00,6,cond_args);
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        _measPdf->ProbabilitySet(0.00,8,cond_args);
        _measPdf->ProbabilitySet(0.05,9,cond_args);
        _measPdf->ProbabilitySet(0.05,10,cond_args);
        _measPdf->ProbabilitySet(0.09,11,cond_args);
        _measPdf->ProbabilitySet(0.11,12,cond_args);
        _measPdf->ProbabilitySet(0.13,13,cond_args);
        _measPdf->ProbabilitySet(0.14,14,cond_args);
        _measPdf->ProbabilitySet(0.15,15,cond_args);
        _measPdf->ProbabilitySet(0.16,16,cond_args);
        _measPdf->ProbabilitySet(0.18,17,cond_args);
        _measPdf->ProbabilitySet(0.01,18,cond_args);
        _measPdf->ProbabilitySet(0.01,19,cond_args);
        cond_args[0] = 18;
        _measPdf->ProbabilitySet(0.00,7,cond_args);
        _measPdf->ProbabilitySet(0.00,8,cond_args);
        _measPdf->ProbabilitySet(0.00,9,cond_args);
        _measPdf->ProbabilitySet(0.05,10,cond_args);
        _measPdf->ProbabilitySet(0.05,11,cond_args);
        _measPdf->ProbabilitySet(0.09,12,cond_args);
        _measPdf->ProbabilitySet(0.11,13,cond_args);
        _measPdf->ProbabilitySet(0.13,14,cond_args);
        _measPdf->ProbabilitySet(0.14,15,cond_args);
        _measPdf->ProbabilitySet(0.15,16,cond_args);
        _measPdf->ProbabilitySet(0.16,17,cond_args);
        _measPdf->ProbabilitySet(0.19,18,cond_args);
        _measPdf->ProbabilitySet(0.01,19,cond_args);
        cond_args[0] = 19;
        _measPdf->ProbabilitySet(0.00,8,cond_args);
        _measPdf->ProbabilitySet(0.00,9,cond_args);
        _measPdf->ProbabilitySet(0.00,10,cond_args);
        _measPdf->ProbabilitySet(0.05,11,cond_args);
        _measPdf->ProbabilitySet(0.05,12,cond_args);
        _measPdf->ProbabilitySet(0.09,13,cond_args);
        _measPdf->ProbabilitySet(0.11,14,cond_args);
        _measPdf->ProbabilitySet(0.13,15,cond_args);
        _measPdf->ProbabilitySet(0.14,16,cond_args);
        _measPdf->ProbabilitySet(0.15,17,cond_args);
        _measPdf->ProbabilitySet(0.16,18,cond_args);
        _measPdf->ProbabilitySet(0.20,19,cond_args);
        _measModel = new MeasurementModel<int,int>(_measPdf);
        //cout << "Measpdf" << endl;
        //for(int state = 0; state<num_states;state++)
        //{
        //     _measPdf->ConditionalArgumentSet(0,state);
        //     for(int meas = 0; meas < num_meas_states;meas++)
        //     {
        //         cout << "prob(" << meas << "|" << state << ")="<< _measPdf->ProbabilityGet(meas) << endl;
        //     }
        // }
        

        // prior
        //DiscretePdf _prior(num_states);
        _prior = new DiscretePdf(num_states);
        vector<Probability> prior_probs(num_states);
        Probability prior_prob = (1.0)/(num_states);
        for(int state = 0; state<num_states;state++)
            prior_probs[state] = prior_prob;
        //prior_probs[0] = 0.08;
        //prior_probs[1] = 0.08;
        //prior_probs[2] = 0.08;
        //prior_probs[3] = 0.08;
        //prior_probs[4] = 0.08;
        //prior_probs[5] = 0.08;
        //prior_probs[6] = 0.08;
        //prior_probs[7] = 0.08;
        //prior_probs[8] = 0.08;
        _prior->ProbabilitiesSet(prior_probs);
    
        _numObjectsFilter = new HistogramFilter<int>(_prior);
        log(Debug) <<"(NumberObjectsEstimator) Finished configureHook"<<endlog();

        vector<Probability> probs(num_states);
        probs = _numObjectsFilter->PostGet()->ProbabilitiesGet();
        for(int state = 0; state<num_states;state++)
            log(Debug) << "probs[" << state << "]: " << (double)probs[state]<< endlog();
        return true;
    }

    bool NumberObjectsEstimator::startHook()
    {
        return true;
    }

    void NumberObjectsEstimator::updateHook()
    {
    }

    void NumberObjectsEstimator::stopHook()
    {
    }
    void NumberObjectsEstimator::cleanUpHook()
    {
        delete _sysPdf,_sysModel,_measPdf,_measModel,_prior,_numObjectsFilter;
    }

    void
    NumberObjectsEstimator::estimateNumObjects()
    {
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] estimateNumObjects started" << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] most probable state " << (_numObjectsFilter->PostGet()->MostProbableStateGet()) << endlog();
#endif
        int num_clusters_measured = _numClustersPort.Get();
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] num_clusters_measured " << num_clusters_measured << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] most probable state " << (_numObjectsFilter->PostGet()->MostProbableStateGet()) << endlog();
#endif

        /*
        int num_states = _numObjectsFilter->PostGet()->NumStatesGet();
        vector<Probability> probs(num_states);
        probs = _numObjectsFilter->PostGet()->ProbabilitiesGet();
        for(int state = 0; state<num_states;state++)
#ifndef NDEBUG
            log(Debug) << "probs[" << state << "]: " << probs[state]<< endlog();
#endif
        std::vector<int> cond_args(num_states);
#ifndef NDEBUG
        log(Debug) << "System model " << endlog();
#endif
        ((DiscreteConditionalPdf*)(_sysModel->SystemPdfGet()));
#ifndef NDEBUG
        log(Debug) << "test " << endlog();
#endif
        for(int cond_arg = 0; cond_arg<num_states;cond_arg++)
        {
           cond_args[0] = cond_arg;
#ifndef NDEBUG
           log(Debug) <<  "cond_arg" << cond_arg << endlog();
#endif
           ((DiscreteConditionalPdf*)(_sysModel->SystemPdfGet()))->ConditionalArgumentSet(0,cond_arg);
            for(int state = 0; state<num_states;state++)
            {
#ifndef NDEBUG
                log(Debug) <<  "state" << state << endlog();
#endif
#ifndef NDEBUG
                log(Debug) << "p( " << state << "|" << cond_arg << "): "<<((DiscreteConditionalPdf*)(_sysModel->SystemPdfGet()))->ProbabilityGet(state)<< endlog();
#endif
            }
        }
#ifndef NDEBUG
        log(Debug) << "Measurement model " << endlog();
#endif
        for(int cond_arg = 0; cond_arg<num_states;cond_arg++)
        {
           cond_args[0] = cond_arg;
           ((DiscreteConditionalPdf*)(_measModel->MeasurementPdfGet()))->ConditionalArgumentSet(0,cond_arg);
            for(int state = 0; state<_maxNumberClusters.value();state++)
            {
#ifndef NDEBUG
                log(Debug) <<  "state" << state << endlog();
#endif
#ifndef NDEBUG
                log(Debug) << "p( " << state << "|" << cond_arg << "): " <<((DiscreteConditionalPdf*)(_measModel->MeasurementPdfGet()))->ProbabilityGet(state)<< endlog();
#endif
            }
        }

        _numObjectsFilter->Update(_sysModel);
#ifndef NDEBUG
        log(Debug) << "after system update " << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] num_meas_states " << _maxNumberClusters.value() << endlog();
#endif

        for(int state = 0 ; state <  _numObjectsFilter->PostGet()->NumStatesGet(); state++) 
        {
            _probObjects[state] = (double)_numObjectsFilter->PostGet()->ProbabilityGet(state);
#ifndef NDEBUG
            log(Debug) << "_probObjects[" << state << "]=" << _probObjects[state] << endlog();
#endif
        }

        _numObjectsFilter->Update(_measModel, num_clusters_measured);
#ifndef NDEBUG
        log(Debug) << "after measurement update " << endlog();
#endif
        for(int state = 0 ; state <  _numObjectsFilter->PostGet()->NumStatesGet(); state++) 
        {
            _probObjects[state] = (double)_numObjectsFilter->PostGet()->ProbabilityGet(state);
#ifndef NDEBUG
            log(Debug) << "_probObjects[" << state << "]=" << _probObjects[state] << endlog();
#endif
        }
        */

        _numObjectsFilter->Update(_sysModel, _measModel, num_clusters_measured);
        _numObjectsPort.Set(_numObjectsFilter->PostGet()->MostProbableStateGet());
        _probObjectsPort.Set(_probObjects);
#ifndef NDEBUG
        log(Debug) << "_numObjects" << _numObjectsFilter->PostGet()->MostProbableStateGet() << endlog();
#endif
#ifndef NDEBUG
        log(Debug) << "[NumberObjectsEstimator] estimateNumObjects finished" << endlog();
#endif
    }

}//namespace


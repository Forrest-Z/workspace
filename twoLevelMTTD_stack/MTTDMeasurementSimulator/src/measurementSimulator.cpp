#include "measurementSimulator.hpp"

#include <ocl/ComponentLoader.hpp>

ORO_CREATE_COMPONENT( OCL::MeasurementSimulator )

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace BFL;

namespace OCL
{
     MeasurementSimulator::MeasurementSimulator(std::string name)
         : TaskContext(name,PreOperational)
         , _environment_file("environment_file", "file in which measurements of environment are stored")
         , _scannerPos("scannerPos", "scanner position in world")
         , _nbeams("nbeams", "number of beams")
         , _nComp("nComp", "number of components of the mixture")
         , _mus("mus", "mean of components of the mixture")
         , _rangeMax("rangeMax","rangeMax the maximum range of the range finder ")
         , _radiusStates("radiusStates","the radius of the different modeled objects ")
         , _maxNumComp("maxNumComp","the maximum number of components ")
         , _maxNumMeas("maxNumMeas","the maximum number of measurements ")
         , _level("level", "the level of continuity of the system model: 0 = cte position, 1= cte velocity ,... ") 
         , _sysNoiseMeanValue("sysNoiseMeanValue","The mean of the noise on the system model")
         , _sysNoiseCovarianceValue("sysNoiseCovarianceValue","The covariance of the noise of the system model")
         , _reporting("reporting", "report to a file or not")
         , _measurementsObjects("measurementsObjectsSimulator")
         , _numMeasurementsObjects("numMeasurementsObjectsSimulator")
         , _distancesObjects("distancesSeparatedObjectsSimulator")
         , _anglesObjects("anglesSeparatedObjectsSimulator")
         , _stateObjectsPort("stateObjectsSimulator")
         , _numObjectsPort("numObjectsSimulator")
         , _SimulateMeasurements("SimulateMeasurements", &MeasurementSimulator::SimulateMeasurements, this)
     {
        // Check if all initialisation was ok:
        assert( _environment_file.ready() );
        assert( _scannerPos.ready() );
        assert( _nbeams.ready() );
        assert( _nComp.ready() );
        assert( _mus.ready() );
        assert( _rangeMax.ready() );
        assert( _radiusStates.ready() );
        assert( _maxNumComp.ready() );
        assert( _maxNumMeas.ready() );
        assert( _level.ready() );
        assert( _sysNoiseMeanValue.ready() );
        assert( _sysNoiseCovarianceValue.ready() );
        assert( _reporting.ready() );

        assert( _SimulateMeasurements.ready() );

        // Now add it to the interface:
        this->properties()->addProperty(&_environment_file);
        this->properties()->addProperty(&_scannerPos);
        this->properties()->addProperty(&_nbeams);
        this->properties()->addProperty(&_nComp);
        this->properties()->addProperty(&_mus);
        this->properties()->addProperty(&_rangeMax);
        this->properties()->addProperty(&_radiusStates);
        this->properties()->addProperty(&_maxNumComp);
        this->properties()->addProperty(&_maxNumMeas);
        this->properties()->addProperty(&_level);
        this->properties()->addProperty(&_sysNoiseMeanValue);
        this->properties()->addProperty(&_sysNoiseCovarianceValue);
        this->properties()->addProperty(&_reporting);

        this->ports()->addPort(&_measurementsObjects);
        this->ports()->addPort(&_numMeasurementsObjects);
        this->ports()->addPort(&_distancesObjects);
        this->ports()->addPort(&_anglesObjects);
        this->ports()->addPort(&_stateObjectsPort);
        this->ports()->addPort(&_numObjectsPort);

        this->methods()->addMethod(&_SimulateMeasurements, "Simulate measurements");
    }

    MeasurementSimulator::~MeasurementSimulator()
    {
    }

    bool MeasurementSimulator::configureHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "configureHook entered" << endlog();

        bool result =  true;
        /**************************************
        Set the state space dimension
        ***************************************/
        _dimension = 2 * (_level.value()+1);
        log(Debug) << "dimension: " << _dimension << endlog(); 

        /**************************************
       Check dimension of properties 
        ***************************************/
        if ( !( (_nComp.value()==0) ||(_radiusStates.value().size() == _nComp.value()) ) )
        {
            log(Error) << "RadiusStates size " << _radiusStates.value().size() << " not equal to the number of components: " << _nComp <<   endlog()  ;
            return false;
        }

        /**************************************
        Connect methods with peers
        ***************************************/
        if(_reporting.value())
        {
            result = result && this->hasPeer("Reporter") ;
            if (!result)
            {
                log(Error) << "MeasurementSimulator does not have expected peers" << endlog();
                log(Error) << "Reporter " << (this->hasPeer("Reporter")) <<endlog();
                return result;
            }
            snapshotReporting = this->getPeer("Reporter")->methods()->getMethod<void(void)>("snapshot");
            result = result && snapshotReporting.ready();
            if (!result)
            {
                log(Error) << "failed to configure  methods " << endlog();
                log(Error) << "snapshotReporting " << snapshotReporting.ready() <<endlog();
                return result;
            }
        }

        log(Debug) << "Methods connected with peers " <<  endlog(); 

        /**************************************
        Resize variables
        ***************************************/
        _stateObjects.resize(_maxNumComp.value());
        _stateObjects.assign(_maxNumComp.value(),ColumnVector(_dimension));
        _distances.resize(_maxNumMeas.value());
        _angles.resize(_maxNumMeas.value());
        _state.resize(_dimension);
        _relPos.resize(2);
        _environment.resize(_nbeams.value());
        _measurements.resize(_maxNumMeas.value());
        _measurements.assign(_maxNumMeas.value(),ColumnVector(2));

        log(Debug) << "_maxNumComp.value() " << _maxNumComp.value() <<  endlog(); 
        log(Debug) << "_maxNumMeas.value() " << _maxNumMeas.value() <<  endlog(); 

        /************************
        // make system model: a constant level'th derivative model
        ************************/
        log(Debug) << "(EstimationGaussian) make system model " << endlog();
        vector<Matrix> AB(1);
    
        ColumnVector sysNoiseVector = ColumnVector(_level.value()+1);
        sysNoiseVector = 0.0;
        SymmetricMatrix sysNoiseMatrixOne = SymmetricMatrix(_level.value() +1);
        sysNoiseMatrixOne = 0.0;
        Matrix sysNoiseMatrixNonSymOne = Matrix(_level.value()+1,_level.value()+1);  
        sysNoiseMatrixNonSymOne = 0.0;
        for(int i =0 ; i<=_level.value(); i++) 
        {
            sysNoiseVector(i+1) = pow(this->getPeriod(),_level.value()-i+1)/double(factorial(_level.value()-i+1));
        }
        sysNoiseMatrixNonSymOne =  (sysNoiseVector* sysNoiseVector.transpose()) * _sysNoiseCovarianceValue.value() ;
        sysNoiseMatrixNonSymOne.convertToSymmetricMatrix(sysNoiseMatrixOne);

        SymmetricMatrix sysNoiseMatrix = SymmetricMatrix(_dimension);
        sysNoiseMatrix = 0.0;
        for(int i =0 ; i<=_level.value(); i++) 
        {
            for(int j =0 ; j<=_level.value(); j++)
            {
                sysNoiseMatrix(i*2+1,j*2+1)=sysNoiseMatrixOne(i+1,j+1);
                sysNoiseMatrix(i*2+2,j*2+2)=sysNoiseMatrixOne(i+1,j+1);
            }
        }

        Matrix sysModelMatrix = Matrix(_dimension,_dimension);  
        sysModelMatrix = 0.0;
        ColumnVector sysNoiseMean = ColumnVector(_dimension);
        sysNoiseMean = _sysNoiseMeanValue.value();
        for(int i =0 ; i<=_level.value(); i++) 
        {
            sysModelMatrix(i*2+1,i*2+1)=1.0;
            sysModelMatrix(i*2+2,i*2+2)=1.0;
            for(int j =i+1 ; j<=_level.value(); j++)
            {
                sysModelMatrix(i*2+1,j*2+1)=pow(this->getPeriod(),j-i)/double(factorial(j-i));
                sysModelMatrix(i*2+2,j*2+2)=pow(this->getPeriod(),j-i)/double(factorial(j-i));
            }
        }

        AB[0]= sysModelMatrix;
        log(Debug) << "sysModelMatrix " << sysModelMatrix << endlog();
        log(Debug) << "sysNoiseMatrix " << sysNoiseMatrix << endlog();

        _sysUncertainty = new Gaussian(sysNoiseMean, sysNoiseMatrix);
        _sysPdf = new LinearAnalyticConditionalGaussian(AB, *_sysUncertainty);
        _sysModel = new LinearAnalyticSystemModelGaussianUncertainty(_sysPdf);
        log(Debug) << "_sysModel->SystemPdfGet()->CovarianceGet()" << _sysModel->SystemPdfGet()->CovarianceGet() << endlog();
        log(Debug) << "(EstimationGaussian) made system model " << endlog();

        /**************************************
        Construct prior 
        ***************************************/
        if ( !( (_nComp.value()==0) ||(_mus.value().size() == _nComp.value()) ) )
        {
            log(Error) << "Prior inconsistent: " << "number of components: " << _nComp << ", size prior means: " << _mus.value().size()<<  endlog()  ;
            return false;
        }

        for (int i=0 ; i<_nComp.value() ; i++)
        {
            if (  !(_mus.value()[i].size() == _dimension ) )
            {
                log(Error) << "Dimensions of prior for component " << i << "is inconsitent with state space dimension( " << _dimension << ") : " << "size of prior mean: " << _mus.value()[i].size() <<   endlog()  ;
                return false;
            }
            _stateObjects[i] = _mus.value()[i];
        }
        for (int i=0 ; i<_nComp.value() ; i++)
        {
            log(Debug) << "_stateObjects[" << i << "] "<<_stateObjects[i] << endlog();
        }


        /**************************************
        Read environment from file
        ***************************************/
        ifstream indata; // indata is like cin
        double num; // variable for input value

        indata.open(_environment_file.value().c_str()); // opens the file
        if(!indata)  // file couldn't be opened
        {
            log(Error) << "Error: file could not be opened" << endlog();
            return false;
        }

        indata >> num; // first one is timestamp
        indata >> num;
        int counter = 0;
        while ( !indata.eof()  && counter < _nbeams) //keep reading until end-of-file
        { 
            _environment[counter] = num;
            indata >> num; // sets EOF flag if no value found
            counter++;
        }
        indata.close();

        /**************************************
        Put prior info on ports 
        ***************************************/
        _stateObjectsPort.Set(_stateObjects);
        _numObjectsPort.Set(_nComp.value());
        // to check prior
        //snapshotReporting();
        //
        teller=0;

        log(Debug) << "configureHook finished" << endlog();
        log(Debug) << "Result of configureHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    bool MeasurementSimulator::startHook()
    {
        
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "startHook entered" << endlog();
        bool result = true;


        time_begin = TimeService::Instance()->getTicks();
        log(Debug) << "startHook finished" << endlog();
        log(Debug) << "Result of startHook: " << result << endlog();
        log(Debug) << "***************************************" << endlog();
        return result;
    }

    void MeasurementSimulator::updateHook()
    {
        time_begin = TimeService::Instance()->getTicks();
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "updateHook entered" << endlog();
#endif

        SimulateMeasurements();
    
        if(_reporting.value())
        {
            snapshotReporting();
#ifndef NDEBUG
            log(Debug) << "Reporting snapshot taken " << endlog();
#endif
        }


        time_passed = TimeService::Instance()->secondsSince(time_begin);
#ifndef NDEBUG
        log(Debug) << "Time passed " << time_passed <<  endlog();
        
        log(Debug) << "updateHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
#endif
    }

    void MeasurementSimulator::stopHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "stopHook entered" << endlog();

        log(Debug) << "stopHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void MeasurementSimulator::cleanUpHook()
    {
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "cleanupHook entered" << endlog();

        delete _sysUncertainty;
        delete _sysPdf;
        delete _sysModel;

        log(Debug) << "cleanupHook finished" << endlog();
        log(Debug) << "***************************************" << endlog();
    }

    void MeasurementSimulator::UpdateObjects()
    {
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "UpdateObjects entered" << endlog();
#endif
        // loop over all objects
        for (int j=0 ; j<_nComp.value() ; j++)
        {
            // do one system model step for each of the objects
            _stateObjects[j] = _sysModel->Simulate(_stateObjects[j]);
        }
#ifndef NDEBUG
        log(Debug) << "UpdateObjects finished" << endlog();
        log(Debug) << "***************************************" << endlog();
#endif
    }

    void MeasurementSimulator::SimulateMeasurements()
    {
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "SimulateMeasurements entered" << endlog();
#endif

        // do system update of objects => updates _stateObjects
        UpdateObjects();
        // simulate measurements for new objects positions => updates
        // _distances and _angles
        CalculateMeasurements();

        /**************************************
        Put info on ports 
        ***************************************/
        _numObjectsPort.Set(_nComp.value());
        _measurementsObjects.Set(_measurements);
        _numMeasurementsObjects.Set(_numMeas);
        _stateObjectsPort.Set(_stateObjects);
        _distancesObjects.Set(_distances);
        _anglesObjects.Set(_angles);

#ifndef NDEBUG
        log(Debug) << "Number of simulated measurements "  << _numMeas << endlog();
#endif
        for (int i = 0 ; i < _numMeas ; i++) 
        {
#ifndef NDEBUG
            log(Debug) << "_distancesObjects[ "  << _distances[i] << "]"<<endlog();
            log(Debug) << "_anglesObjects[ "  << _angles[i] << "]" << endlog();
#endif
        } 

#ifndef NDEBUG
        log(Debug) << "SimulateMeasurements finished" << endlog();
        log(Debug) << "***************************************" << endlog();
#endif
    }
    void MeasurementSimulator::CalculateMeasurements()
    {
#ifndef NDEBUG
        log(Debug) << "***************************************" << endlog();
        log(Debug) << "CalculateMeasurements entered" << endlog();
#endif
        _distances.assign(_maxNumMeas.value(),0.0);
        _angles.assign(_maxNumMeas.value(),0.0);
        _numMeas = 0;
        for(int i=1; i<=_nbeams.value();i++) 
        {
            double m,m2;
            int solution;
            m2=0.0;
            if(_nbeams.value()==1)
                m = 0.0;
            else
                m = tan( (i-1)*M_PI/(_nbeams-1));
            if (fabs(m)<100)
            { 
                solution = 1; // y=m x is equation of beam
            }
            else
            {
                solution = 2; // x=m2 y is equation of beam
                m2 = tan(M_PI/2- (i-1)*M_PI/(_nbeams.value()-1) ) ;
            }
            // find measurement for that beam
            double exp_meas_beam = -1.0;
            for (int j=0 ; j<_nComp.value() ; j++)
            {
                // Calculate which beams state affects
                // get state under consideration
                _state = _stateObjects[j];
                // calculate relative position of state wrt scanner
                _relPos.resize(_state.sub(1,2).size());
                _relPos = _state.sub(1,2) - _scannerPos;
                // Calculate intersection between beams and circle around state (x,y)
                double x,y; 
                double a,b,c,D;
                double x1,x2,y1,y2;
                double exp_meas,distsq ;
                switch (solution)
                {
                    case 1 :
                        a = pow(m,2.0)+1;
                        b = -2*(_relPos(1)+m*_relPos(2));
                        c = pow(_relPos(1),2.0) + pow(_relPos(2),2.0) - pow(_radiusStates.value()[j],2.0);
                        D = pow(b,2.0) - 4 * a * c;
                        distsq = pow(_relPos(1),2.0) + pow(_relPos(2),2.0);
                        if( (distsq>pow(_radiusStates.value()[j],2.0)) && (D>=0)  ) 
                        // there is an intersection between stateCircle and beam, and
                        // still to check intersection point is on positive beam 
                        {
                            if (a==0)
                            {
                               x = -c/b; 
                               y = m*x;
                            }
                            else
                            {
                                x1 = (-b + sqrt(D))/(2*a);
                                x2 = (-b - sqrt(D))/(2*a);
                                // solution is x with smalles absvalue
                                if(fabs(x1) <= fabs(x2) )
                                {
                                    x = x1;
                                    y = m*x;
                                }
                                else
                                {
                                    x = x2;
                                    y = m*x;
                                }
                                exp_meas = sqrt( pow(x,2.0) + pow(y,2.0) );
                                if ( (exp_meas <= _environment[i-1] ) && (exp_meas <= _rangeMax.value()) && ( (exp_meas_beam == -1.0) || (exp_meas < exp_meas_beam) ) && ( ( (m!=0) && (x*m)>=0) || (i==1 && x>=0) || (i==_nbeams && x<=0)  )  )
                                // only measurement if distance < measurement range and enviroment measurement
                                // and intersection point is on positive beam
                                {
                                    exp_meas_beam = exp_meas; 
                                }
                            }
                        }
                        break;
                    case 2 :
                        a = pow(m2,2.0)+1;
                        b = -2*(_relPos(2)+m2*_relPos(1));
                        c = pow(_relPos(1),2.0) + pow(_relPos(2),2.0) - pow(_radiusStates.value()[j],2.0);
                        D = pow(b,2.0) - 4 * a * c;
                        distsq = pow(_relPos(1),2.0) + pow(_relPos(2),2.0);
                        if( (distsq>pow(_radiusStates.value()[j],2.0)) && (D>=0) )
                        // there is an intersection between stateCircle and beam, and
                        // intersection point is on positive beam 
                        {
                            if (a==0)
                            {
                               y = -c/b; 
                               x = m2*y;
                            }
                            else
                            {
                                y1 = (-b + sqrt(D))/(2*a);
                                y2 = (-b - sqrt(D))/(2*a);
                                // solution is y with smallest absvalue
                                if(fabs(y1) <= fabs(y2) )
                                {
                                    y = y1;
                                    x = m2*y;
                                }
                                else
                                {
                                    y = y2;
                                    x = m2*y;
                                }
                                exp_meas = sqrt( pow(x,2.0) + pow(y,2.0) );
                                if ( (exp_meas <= _environment[i-1] ) && (exp_meas <= _rangeMax.value()) && ( (exp_meas_beam == -1.0) || (exp_meas < exp_meas_beam) ) )// only measurement if distance < measurement range
                                {
                                    exp_meas_beam = exp_meas; 
                                }
                            }
                        }
                        break;
                    default:
                        log(Error) << "(MeasurementSimulator::CalculateMeasurements) Error no solution found: case wrong implemented " << endlog();
                }
            }
            if ( !(exp_meas_beam == -1.0)  ) // found object on this beam
            {
                _distances[_numMeas]=exp_meas_beam;
                _angles[_numMeas]=((i-1)*M_PI/(_nbeams.value()-1));
                _measurements[_numMeas](1) = _distances[_numMeas] * cos( _angles[_numMeas] );
                _measurements[_numMeas](2) = _distances[_numMeas] * sin( _angles[_numMeas] );
                _numMeas++;
            }
            
        }
#ifndef NDEBUG
        log(Debug) << "CalculateMeasurements finished" << endlog();
        log(Debug) << "***************************************" << endlog();
#endif
        
    }

    int MeasurementSimulator::factorial (int num)
    {
        if (num==1)
            return 1;
        return factorial(num-1)*num; // recursive call
    }

}//namespace


#include <rtt/TaskContext.hpp>
#include <rtt/Activities.hpp>
#include <rtt/Ports.hpp>
#include <ocl/ComponentLoader.hpp>
#include <iostream>
#include <sstream>

#include <MTTD_msgs/People.h>
//#include "OrocosData.hpp"

using namespace RTT;
//using namespace std_msgs;

class MTTDOutput : public RTT::TaskContext{
public:
  MTTDOutput(const std::string& name):
    TaskContext(name,PreOperational),
    _estimateMxFiltersPort("estimateMxFilters"),
    _estimateMyFiltersPort("estimateMyFilters"),
    _filterIDPort("filterID"),
    _numFiltersPort("numFilters"),
    _ros_outport("peopledata")
//     wport1("WritePort1",0.0),
//     wport2("WritePort2",std::vector<double>(6,0.0)),
//     wport3("WritePort3")
  {
    this->ports()->addPort(&_estimateMxFiltersPort);
    this->ports()->addPort(&_estimateMyFiltersPort);
    this->ports()->addPort(&_filterIDPort);
    this->ports()->addPort(&_numFiltersPort);

    this->ports()->addPort(&_ros_outport);
//     this->ports()->addPort(&wport2);
//     this->ports()->addPort(&wport3);

    
  };
 
  bool configureHook()
  {
    ///Max number of clusters, numfilters
    //TODO into a property
    _output.people.resize(20);
    return true;
  }
  
  void updateHook(){  
    _filterIDPort.Get(_id);
    _estimateMxFiltersPort.Get(_xpoint);
    _estimateMyFiltersPort.Get(_ypoint);
    _numFiltersPort.Get(_num);
    
   // std::cout << _num << std::endl;
    _output.header.stamp=ros::Time::now();
    _output.header.frame_id="map";
    _output.num=_num;
    for(int i=0;i<_num;i++)
    {
      _output.people[i].header.frame_id="map";
      _output.people[i].name="nome";
      //oss<<i;
      std::ostringstream oss;
      oss<<_id[i];
      _output.people[i].object_id=oss.str();
      _output.people[i].pos.x=_xpoint[i];
      _output.people[i].pos.y=_ypoint[i];
      _output.people[i].pos.z=0;
      //TODO:: add covariance
    }
    /* Old stuff, 9 june
    //TODO: introduce verbose mode on based on property
    //TODO: introduce frame_id remapping 
    //TODO: refactoring ROSmessage more friendly
    //TODO: make resize more quickly, into a start or config method
    //std::cout << "\n******************" << std::endl;

    output.header.frame_id="map";
    output.filter_id.resize(_id.size());
    output.coordinates.resize(_id.size());
      //Now we'll Fill the ROS message
     // for(unsigned int i=0;i<_id.size();i++)
      {
	//std::cout << _id[i] << "\t" << std::endl;
	output.filter_id[i]=_id[i];
	output.coordinates[i].x=_xpoint[i];
	output.coordinates[i].y=_ypoint[i];
	output.coordinates[i].z=0; // I don't care until now
      }*/
      
     //Send/Set ROSmessage
     _ros_outport.Set(_output);
     
    };

  
  ~MTTDOutput(){};
  
private:

    std::vector<double> _id;
    std::vector<double> _xpoint;
    std::vector<double> _ypoint;
    int _num;
    
    MTTD_msgs::People _output;
     
    ReadDataPort<std::vector<double> > _estimateMxFiltersPort;
    ReadDataPort<std::vector<double> > _estimateMyFiltersPort;
    ReadDataPort<std::vector<double> > _filterIDPort;
  
    ReadDataPort<int> _numFiltersPort;
    //  WriteDataPort<MTTD_msgs::PeopleCoordinates> _ros_outport;
    WriteDataPort<MTTD_msgs::People> _ros_outport;

};

ORO_CREATE_COMPONENT(MTTDOutput)

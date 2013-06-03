#include <rtt/TaskContext.hpp>
#include <rtt/Activities.hpp>
#include <rtt/Ports.hpp>
#include <ocl/ComponentLoader.hpp>


//TODO: the first or the secondo include?
//#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/wrappers/rng/rng.h>
#include <sensor_msgs/PointCloud.h>
#include "OrocosData.hpp"

/**
*  This Component received PointCloud ROS msg and convert it into Column Vector datastruct avaiable in BFL.
*  Autor: Enea Scioni	enea.scioni@student.unife.it
**/
using namespace RTT;


class MTTDInput : public RTT::TaskContext{
public:
  MTTDInput(const std::string& name):
    TaskContext(name,PreOperational),
    _cloud_data("object_point_cloud"),
    _numMeasurementsObjects("numMeasurementsObjects"),
    _measurementsObjects("measurementsObjects"),
    wport2("WritePort2")

  {
    this->ports()->addPort(&_cloud_data);
    this->ports()->addPort(&_measurementsObjects);
    this->ports()->addPort(&_numMeasurementsObjects);
    this->ports()->addPort(&wport2);
    
  };
  
  bool configureHook()
  {
    _cloud_data.Get(_readcloud);
    //TODO::parametrize with the max.value avaialble for the laser, device's dependences
    _measurement.resize(2);
    _measurements.resize(361);
    _measurements.assign(361,MatrixWrapper::ColumnVector(2));
    return true;
  }
  
  void updateHook(){
   // float_.data++;
    
    _cloud_data.Get(_readcloud);
    
    _numMeasurementsObjects.Set(_readcloud.points.size());
   for(unsigned int i=0;i<_readcloud.points.size();i++)
   {
     _measurement(1)=_readcloud.points[i].x;
     _measurement(2)=_readcloud.points[i].y;
     _measurements[i]=_measurement;
   }
   
   _measurementsObjects.Set(_measurements);
    wport2.Set(_readcloud);
  };

  
  ~MTTDInput(){};
  
private:
  
  
  /// Data port to read the non-environment pointcloud msg by ROS node
  ReadDataPort<sensor_msgs::PointCloud> _cloud_data;
  
  /// Data port to write the number of points to be cluster (connected to VBCluster)
  WriteDataPort<int > _numMeasurementsObjects;
  
  /// Data port to write the points to be cluster (connected to VBCluster)
  WriteDataPort<std::vector<MatrixWrapper::ColumnVector> >    _measurementsObjects;
  
  ///just for debug
  WriteDataPort<sensor_msgs::PointCloud> wport2;
  
  /// Variable where memorize incoming pointcloud data
  sensor_msgs::PointCloud _readcloud;
  
  ///Helper variable
  std::vector<MatrixWrapper::ColumnVector> _measurements;
  MatrixWrapper::ColumnVector _measurement;
};

ORO_CREATE_COMPONENT(MTTDInput)

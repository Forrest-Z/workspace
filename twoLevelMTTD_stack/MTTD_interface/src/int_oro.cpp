#include <rtt/TaskContext.hpp>
#include <rtt/Activities.hpp>
#include <rtt/Ports.hpp>
#include <ocl/ComponentLoader.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
//#include "laserdata.hpp"

using namespace RTT;
using namespace std_msgs;

class Component1 : public RTT::TaskContext{
public:
  Component1(const std::string& name):
    TaskContext(name,PreOperational),
    rport1("ReadPort1"),
    rport2("ReadPort2"),
    rport3("ReadPort3"),
    rport4("ReadPort4"),
    rport5("ReadPort5"),
    wport1("WritePort1",0.0),
    wport2("WritePort2",std::vector<double>(6,0.0)),
    wport3("WritePort3"),
    wport4("WritePort4")
  {
    this->ports()->addPort(&rport1);
    this->ports()->addPort(&rport2);
    this->ports()->addPort(&rport3);
    this->ports()->addPort(&rport4);
    this->ports()->addPort(&rport5);
    this->ports()->addPort(&wport1);
    this->ports()->addPort(&wport2);
    this->ports()->addPort(&wport3);
    this->ports()->addPort(&wport4);
    
  };
  void updateHook(){
    float_.data++;
    
    rport4.Get(readlaser_);
    rport5.Get(readcloud_);
    if(float_.data ==8000)
    {
      std::cout << readlaser_.header.frame_id << std::endl;
      std::cout <<  "Ranges\n" << std::endl;
      for(int i=0;i<readlaser_.ranges.size();i++)
	std::cout << readlaser_.ranges[i] << "\t" ;
      std::cout << "\n" << std::endl;
      std::cout << "angle_min: " << readlaser_.angle_min << std::endl;
      std::cout << "angle_max: " << readlaser_.angle_max << std::endl;
      std::cout << "range_min: " << readlaser_.range_min << std::endl;
      std::cout << "range_max: " << readlaser_.range_max << std::endl;
      
      //Scrivo su porta
      wport4.Set(readlaser_);
      //****** PointCloud    
      std::cout <<  "******** PointCloud ********\n" << std::endl;
      std::cout << readcloud_.header.frame_id << std::endl;
      for(int i=0;i<readcloud_.points.size();i++)
	std::cout << "[" <<readcloud_.points[i].x << ","<< readcloud_.points[i].y <<","<< readcloud_.points[i].z << "]" "\t" ;
      std::cout << "\n" << std::endl;
    }
    
  /*  
    rport3.Get(text_);
    if(float_.data == 8000)
      std::cout << text_.data << std::endl;
    */
    wport1.Set(0.1);
    wport3.Set(float_);
  };

  
  ~Component1(){};
private:
  std::vector<double> array_;
  Float64 float_;
  ReadDataPort<double> rport1;
  ReadDataPort<std::vector<double> > rport2;
  ReadDataPort<String> rport3;
  ReadDataPort<sensor_msgs::LaserScan> rport4;
  ReadDataPort<sensor_msgs::PointCloud> rport5;
  WriteDataPort<double > wport1;
  WriteDataPort<std::vector<double> > wport2;
  WriteDataPort<Float64 > wport3;
  WriteDataPort<sensor_msgs::LaserScan> wport4;
  
  //Added (Enea Scioni)
  std_msgs::String text_;
  sensor_msgs::LaserScan readlaser_;
  sensor_msgs::PointCloud readcloud_;
};

ORO_CREATE_COMPONENT(Component1)

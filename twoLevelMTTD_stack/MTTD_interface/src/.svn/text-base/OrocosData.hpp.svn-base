#include <string>
#include <vector>

/**************************************************
*	typedef for Orocos
*
**************************************************/

/// Standard ROS Header
typedef struct{
  unsigned seq;
  std::string frame_id;
} Header;

/// geometry_msgs/Point
typedef struct{
  float x;
  float y;
  float z;
} Point;

/// ChannelFloat (sensor_msgs/PointCloud)
typedef struct{
  std::string name;
  std::vector<float> values;
} ChannelFloat;

/// LaserData (sensor_msgs/LaserScan)
typedef struct{
  Header header;
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float range_min;
  float range_max;
  std::vector<float> ranges;
  std::vector<float> intensities;
  
} LaserData;

/// CloudData (sensor_msgs/PointCloud)
typedef struct{
  Header header;
  std::vector<Point> points;
  std::vector<ChannelFloat> channels;
} CloudData;

/// MTTD_msgs/PeopleCoordinates
typedef struct{
  Header header;
  std::vector<double> filter_id;
  std::vector<Point> coordinates;
}PeopleData;

/// MTTD_msgs & person_msgs/PositionMeasurement (people stack)
typedef struct{
  Header header;
  std::string name;
  std::string object_id;
  Point pos;
  double reliability;
  double covariance[9];
  bool initialization;
} MeasurementData;

/// MTTD_msgs/People , PositionMeasurement vector
typedef struct{
  std::vector<MeasurementData> people;
  int num;
} PeopleData2;

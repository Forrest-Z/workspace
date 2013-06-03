#include <string>
#include <vector>


typedef struct{
  unsigned seq;
  std::string frame_id;
} Header;

typedef struct{
  float x;
  float y;
  float z;
} Point;

typedef struct{
  std::string name;
  std::vector<float> values;
} ChannelFloat;

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

typedef struct{
  Header header;
  std::vector<Point> points;
  std::vector<ChannelFloat> channels;
} CloudData;

typedef struct{
  Header header;
  std::vector<double> filter_id;
  std::vector<Point> coordinates;
}PeopleData;
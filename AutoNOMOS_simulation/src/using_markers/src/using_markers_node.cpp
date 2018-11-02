#include "using_markers.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "using_markers");
  UsingMarkers OBJECT;
  
  ros::spin();
  return 0;
}

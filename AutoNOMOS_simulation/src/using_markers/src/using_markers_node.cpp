#include "using_markers.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "using_markers");
  using_markers OBJECT;
  
  ros::spin();
  return 0;
}

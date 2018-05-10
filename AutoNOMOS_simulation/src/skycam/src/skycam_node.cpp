#include "skycam.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "skycam");
  skycam OBJECT;
  
  ros::spin();
  return 0;
}

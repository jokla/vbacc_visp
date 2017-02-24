#include <ros/ros.h>

#include "vbacc.h"
#include <visp_bridge/3dpose.h>
#include <visp3/core/vpHomogeneousMatrix.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv, "vbacc" );

  ros::NodeHandle n(std::string("~"));

  vbacc *node = new vbacc(n);

  node->spin();

  delete node;

  return 0;
}





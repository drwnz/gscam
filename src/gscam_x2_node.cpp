
#include <ros/ros.h>
#include <gscam_x2/gscam_x2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh, nh_private("~");

  gscam_x2::GSCamX2 gscam_x2_driver(nh, nh_private);
  gscam_x2_driver.run();

  return 0;
}


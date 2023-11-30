#include "teensycommunucation/teensycommunucation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teensycommunucation");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  Teensycommunucation teensycommunucation (nh, nh_private);
  
  ros::spin();
  return 0;
}
#include "pneutrunk_tools/continuum_model.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContinuumModel>());
  rclcpp::shutdown();
  return 0;
}
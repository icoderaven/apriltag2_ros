#include <apriltag2_ros/TagDetectorRosWrapper.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh("~");
  std::string params_file;
  nh.param<std::string>("params_file", params_file, "../config/params.yaml");
  TagDetectorRosWrapper wrapper(params_file, nh);
  ros::spin();
  return EXIT_SUCCESS;
}

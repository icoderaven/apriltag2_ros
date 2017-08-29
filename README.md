# apriltag2_ros
Illustrative ROS package to utilize the apriltag2 (https://github.com/icoderaven/apriltag2_catkin) package using OpenCV

Requires Eigen, OpenCV, and libYAML

Teh tag_detector_node only serves to detec tags in an incoming image stream, and outputs a list of tags detected in the image with their coordinates in the custom message, which has been retained from previous packages for compatibility.

Modify any of the options in the YAML config file in the config directory.
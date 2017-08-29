#pragma once

#include <iostream>

#include <apriltag2_ros/apriltag_params.h>
#include <apriltag2_ros/Apriltag.h>
#include <apriltag2_ros/Apriltags.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <apriltag2/apriltag.h>
#include <apriltag2/tag36h11.h>
#include <apriltag2/tag36h10.h>
#include <apriltag2/tag36artoolkit.h>
#include <apriltag2/tag25h9.h>
#include <apriltag2/tag25h7.h>

class TagDetectorRosWrapper {
 public:
  // Minimal constructor
  TagDetectorRosWrapper(const std::string &config_file);
  // Constructor for standalone node
  TagDetectorRosWrapper(const std::string &config_file, const ros::NodeHandle &nh);
  ~TagDetectorRosWrapper();

 private:
  void readConfig(const std::string &config_file);
  void initSubscribers();
  void initPublishers();
  void callback(const sensor_msgs::ImageConstPtr &image_msg);
  apriltag2_ros::Apriltag detectionToApriltagMsg(const apriltag_detection_t *detection);

  apriltag_params params_;

  apriltag_detector_t *detector_;
  apriltag_family_t *family_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Subscriber> sub_;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher debug_img_pub_, detections_pub_;
};
#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

struct apriltag_params
{
  std::string family; //Tag family to use
  double tag_size;
  cv::Mat K;
  cv::Mat D;
  bool do_debug;      //Enable publishing of drawn overlay
  int threads;        //Use this many CPU threads
  double decimate;    //Decimate input image by this factor
  double blur;        //Apply low-pass blur to input
  bool refine_edges;  //Spend more time trying to align edges of tags
  bool refine_decode; //Spend more time trying to decode tags
  bool refine_pose;   //Spend more time trying to precisely localize tags
  Eigen::MatrixXf cam_in_body, cam_in_body_inv;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  apriltag_params()
      : family("tag36h11"),
        tag_size(1.0),
        K(cv::Matx33d::eye()),
        D(cv::Mat::zeros(4, 1, CV_64FC1)),
        do_debug(false),
        threads(4),
        decimate(1.0),
        blur(0.0),
        refine_edges(true),
        refine_decode(false),
        refine_pose(false),
        cam_in_body(Eigen::Matrix4f::Identity()),
        cam_in_body_inv(Eigen::Matrix4f::Identity())
  {
  }
  void read_config(std::string params_file)
  {
    std::cout << "[AprilTagParams] Reading from file " << params_file << "\n";
    try
    {
      YAML::Node params = YAML::LoadFile(params_file);
      YAML::Node apriltag_params_node = params["apriltag_tracker_params"];
      if (!apriltag_params_node)
      {
        std::cerr << "[AprilTagParams] Could not read apriltag_tracker_params!";
        exit(-1);
      }
      else
      {
        if (apriltag_params_node["tag_size"])
        {
          tag_size = apriltag_params_node["tag_size"].as<double>();
        }
        if (apriltag_params_node["family"])
        {
          family = apriltag_params_node["family"].as<std::string>();
        }
        if (apriltag_params_node["do_debug"])
        {
          do_debug = apriltag_params_node["do_debug"].as<bool>();
        }
        if (apriltag_params_node["threads"])
        {
          threads = apriltag_params_node["threads"].as<int>();
        }
        if (apriltag_params_node["decimate"])
        {
          decimate = apriltag_params_node["decimate"].as<double>();
        }
        if (apriltag_params_node["blur"])
        {
          blur = apriltag_params_node["blur"].as<double>();
        }
        if (apriltag_params_node["refine_edges"])
        {
          refine_edges = apriltag_params_node["refine_edges"].as<bool>();
        }
        if (apriltag_params_node["refine_decode"])
        {
          refine_decode = apriltag_params_node["refine_decode"].as<bool>();
        }
        if (apriltag_params_node["refine_pose"])
        {
          refine_pose = apriltag_params_node["refine_pose"].as<bool>();
        }
        if (apriltag_params_node["K"])
        {
          for (int i = 0; i < 3; ++i)
          {
            for (int j = 0; j < 3; ++j)
            {
              K.at<double>(i, j) = apriltag_params_node["K"][i * 3 + j].as<double>();
            }
          }
        }
        if (apriltag_params_node["D"])
        {
          ;
          for (int i = 0; i < 4; ++i)
          {
            D.at<double>(i) = apriltag_params_node["D"][i].as<double>();
          }
        }
        YAML::Node cam_in_body_node = apriltag_params_node["cam_in_body"];
        if (cam_in_body_node)
        {
          for (int i = 0; i < 4; ++i)
          {
            for (int j = 0; j < 4; ++j)
            {
              cam_in_body(i, j) =
                  cam_in_body_node[i * 4 + j].as<float>();
            }
          }
          cam_in_body_inv = cam_in_body.inverse();
        }
      }
    }
    catch (const std::runtime_error &e)
    {
      std::cerr << e.what() << std::endl;
      exit(-1);
    }
  }

  friend std::ostream &operator<<(std::ostream &o, const apriltag_params &params)
  {
    o << "tag_size: " << params.tag_size << "\nK: "
      << params.K << "\nD: " << params.D << "\n"
      << "\nfamily: " << params.family
      << "\ndo_debug: " << params.do_debug
      << "\nthreads: " << params.threads
      << "\ndecimate: " << params.decimate
      << "\nblur: " << params.blur
      << "\nrefine edges: " << params.refine_edges
      << "\nrefine decode: " << params.refine_decode
      << "\nrefine pose: " << params.refine_pose
      << "\ncam_in_body: \n"
      << params.cam_in_body << "\n";
    return o;
  }
};
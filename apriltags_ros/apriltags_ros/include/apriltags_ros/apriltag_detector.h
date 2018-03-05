#ifndef APRILTAG_DETECTOR_H
#define APRILTAG_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <AprilTags/TagDetector.h>
#include <tf/transform_broadcaster.h>

namespace apriltags_ros{


class AprilTagDescription{
 public:
  AprilTagDescription(int id, double size, std::string &frame_name):id_(id), size_(size), frame_name_(frame_name){}
  double size(){return size_;}
  int id(){return id_;}
  std::string& frame_name(){return frame_name_;}
 private:
  int id_;
  double size_;
  std::string frame_name_;
};


class AprilTagDetector{
 public:
  AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~AprilTagDetector();
 private:
  void imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info);
  std::map<int, AprilTagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

 private:
  std::map<int, AprilTagDescription> descriptions_;
  std::string sensor_frame_id_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detections_pub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_pub_;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_;

  // Global variables
  // TODO: make these variables into the class
  std::vector<AprilTags::TagDetection>	detections;
  int count_tag_loss;
  int count_idle_status;
  // ROI
  int Cx;
  int Cy;
  cv::Rect roi_rect; // (0,0,1,1);
  // Speed estimation, pixel/sample
  double speed_x;
  double speed_y;

  // Parameters
  // ROI
  int x_border; // To reduce the effectness region in the odriginal image in x-direction. Double-sided
  int y_border; // To reduce the effectness region in the odriginal image in y-direction. Double-sided
  int ROI_height; // 300 // 220; // 150; // 300;
  int ROI_width;  // 300 // 220; // 150; // 300;
  // ROI for initial scan
  int ROI_height_scan; // For initial scan // ROI_height/2*3;
  int ROI_width_scan;  // For initial scan // ROI_width/2*3;
  // Resize
  double resize_scale; // 1.0 // 2.0; // 1.2;
  // Speed filter
  double speed_filterRatio; // 0.3 // 0.5;
  // Average light
  int lightSense_Ker_halfSize; // 10;
  int average_light_level; // 127
  // Sharpen
  double sharpen_retain_ratio; // 0.2;
  double sharpen_enhancement_ratio; // 12.0; // 6.0
  //
};



}


#endif

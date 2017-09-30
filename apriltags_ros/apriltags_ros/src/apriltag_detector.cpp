#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

//
#include <iostream>
using std::cout;
//

namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  // Get the tag_family from param
  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  // pnh.param<bool>("projected_optics", projected_optics_, false);

  // Get tagCodes
  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  // AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;
  // tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(tag_codes));
  image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
  image_pub_ = it_.advertise("tag_detections_image", 1);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
}
AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
}

// Utilities
////////////////////////////
cv::Rect convert_centerPoint_to_ROI(int Cx, int Cy, int height, int width, int nRow, int nCol){
    int half_height = height/2;
    int half_width = width/2;
    //
    int x_L = Cx - half_width;
    int y_L = Cy - half_height;
    int x_H = x_L + width;
    int y_H = y_L + height;

    // Fix range, x
    if (x_L < 0){
        x_H -= x_L;
        x_L = 0;
    }else if(x_H > nCol){
        x_L -= x_H - nCol;
        x_H = nCol;
    }
    // Fix range, y
    if (y_L < 0){
        y_H -= y_L;
        y_L = 0;
    }else if(y_H > nRow){
        y_L -= y_H - nRow;
        y_H = nRow;
    }

    // x, y, width, height
    cv::Rect rect(x_L, y_L, (x_H - x_L), (y_H - y_L));
    return rect;
}
//////////////////////////// Utilities


// Global variables
// TODO: make these variables into the class
std::vector<AprilTags::TagDetection>	detections;
cv::Rect roi_rect;
int count_tag_loss = 0;
int count_idle_status = 0;
// Parameters
int ROI_height = 300; // 220; // 150; // 300;
int ROI_width = 300; // 220; // 150; // 300;
double resize_scale = 1.0; // 1.2;
//


void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& cam_info){
  // Get the (raw) image
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);


  // Algorithm of dynamic ROI amd up-sampling
  /////////////////////////
  int nRow = gray.rows;
  int nCol = gray.cols;
  //
  int Cx, Cy; // Position of the center
  int ROI_height_set = ROI_height;
  int ROI_width_set = ROI_width;
  //
  if (detections.size() == 0 && count_tag_loss >= 6){
    // Empty, chose the center area
    /*
    // Fixed at the center
    Cx = nCol/2;
    Cy = nRow/2;
    //
    ROI_height_set = nRow;
    ROI_width_set = nCol;
    */
    //
    ROI_height_set = nRow/2;
    ROI_width_set = nCol/2;
    //
    switch (count_idle_status){
      case 0: // Center
        Cx = nCol/2;
        Cy = nRow/2;
        break;
      case 1: // Left-up
        Cx = 0;
        Cy = 0;
        break;
      case 2: // Right-up
        Cx = nCol;
        Cy = 0;
        break;
      case 3: // Right-down
        Cx = nCol;
        Cy = nRow;
        break;
      case 4: // Left-down
        Cx = 0;
        Cy = nRow;
        //
        break;
      default:
        Cx = nCol/2;
        Cy = nRow/2;
        break;
    }
    //
    if (count_idle_status >= 4){
      count_idle_status = 0;
    }else{
      count_idle_status++;
    }
    //
  }else{
    //
    if (detections.size() == 0){
      // Loss the tag for a short time
      // Leave the Cx and Cy unchanged
      count_tag_loss++;
      // Cx = (the latest one)
      // Cy = (the latest one)
    }else{
      // At least one tag in the former frame
      count_tag_loss = 0;
      Cx = roi_rect.x + detections[0].cxy.first/resize_scale;
      Cy = roi_rect.y + detections[0].cxy.second/resize_scale;
    }
  }


  // Calculate the ROI(s)
  // cv::Rect roi_rect = convert_centerPoint_to_ROI(nCol/2, nRow/2, 300, 300, nRow, nCol);
  // cv::Rect roi_rect = convert_centerPoint_to_ROI(Cx, Cy, 300, 300, nRow, nCol);
  roi_rect = convert_centerPoint_to_ROI(Cx, Cy, ROI_height_set, ROI_width_set, nRow, nCol);
  // cv::Mat gray_roi = gray(roi_rect).clone();

  // Histogram Equalization for solving the lighting problem
  // cv::Mat gray_roi_enhanced;
  // gray(roi_rect).convertTo(gray_roi_enhanced, -1, 2.0, -128 ); // I_out = 2.0*I_in + (-128) // -1 stands for the same type
  // GaussianBlur(gray(roi_rect), gray_roi_enhanced, cv::Size(5,5), 2.0); // sigma = 2.0
  // cout << "gray_roi_enhanced.rows = " << gray_roi_enhanced.rows << ", gray_roi_enhanced.cols = " << gray_roi_enhanced.cols << "\n";

  //
  // double resize_scale = 2.0;
  // Size size_new( int(roi_rect.width*resize_scale), int(roi_rect.height*resize_scale) );//the dst image size, Size_(_Tp _width, _Tp _height);
  cv::Mat gray_roi_resize;
  // resize(gray_roi, gray_roi_resize, cv::Size(0,0), resize_scale, resize_scale); // interpolation = CV_INTER_LINEAR or CV_INTER_NEAREST
  resize(gray(roi_rect), gray_roi_resize, cv::Size(0,0), resize_scale, resize_scale, cv::INTER_LANCZOS4); // interpolation = INTER_LANCZOS4, INTER_CUBIC, NTER_AREA, INTER_LINEAR, or INTER_NEAREST
  // resize( gray_roi_enhanced, gray_roi_resize, cv::Size(0,0), resize_scale, resize_scale, cv::INTER_LANCZOS4); // interpolation = INTER_LANCZOS4, INTER_CUBIC, NTER_AREA, INTER_LINEAR, or INTER_NEAREST
  // cout << "Size of ROI: (" << gray_roi.rows << " x " << gray_roi.cols << ")\n";

  // test, print the Cropped image
  // cv::cvtColor(gray_roi, cv_ptr->image, CV_GRAY2BGR);
  cv::cvtColor(gray_roi_resize, cv_ptr->image, CV_GRAY2BGR);
  //

  /////////////////////////


  // Extract the tags
  ///////////////////////////////
  // std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray_roi);
  // detections = tag_detector_->extractTags(gray_roi);
  detections = tag_detector_->extractTags(gray_roi_resize);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  //
  /*
  if (detections.size() > 0){
    cout << "x: " << detections[0].cxy.first << ", y: " << detections[0].cxy.second << "\n";
  }
  */
  //


  // use projected focal length and principal point
  // these are the correct values

  // Bounderies of ROI
  double roi_offset_x = roi_rect.x; // cam_info->roi.x_offset;
  double roi_offset_y = roi_rect.y; // cam_info->roi.y_offset;
  /*
  // Focal length in pixel-coordinate
  double fx = cam_info->P[0]; // cam_info->P[0]*1.0, resize 1.0x in x (column)
  double fy = cam_info->P[5]; // cam_info->P[5]*1.0, resize 1.0x in y (raw)
  // Principle points in pixel-coordinate
  double px = (cam_info->P[2] - roi_offset_x);
  double py = (cam_info->P[6] - roi_offset_y);
  */
  // Focal length in pixel-coordinate
  double fx = cam_info->P[0]*resize_scale; // cam_info->P[0]*1.0, resize 1.0x in x (column)
  double fy = cam_info->P[5]*resize_scale; // cam_info->P[5]*1.0, resize 1.0x in y (raw)
  // Principle points in pixel-coordinate
  double px = (cam_info->P[2] - roi_offset_x)*resize_scale;
  double py = (cam_info->P[6] - roi_offset_y)*resize_scale;
  //


  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = cv_ptr->header;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    detection.draw(cv_ptr->image); // TODO: Try using the ROI of the original image instead of using the new image
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0,0,3,3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0,3);
    tag_pose.pose.position.y = transform(1,3);
    tag_pose.pose.position.z = transform(2,3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = cv_ptr->header;

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose.pose);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  detections_pub_.publish(tag_detection_array);
  pose_pub_.publish(tag_pose_array);
  image_pub_.publish(cv_ptr->toImageMsg());
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}

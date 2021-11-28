#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif



cv_bridge::CvImagePtr cv_ptr;
cv::Mat out_img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    out_img = cv_ptr->image.clone();
    cv::Size img_size = out_img.size();
    ROS_INFO("cv_ptr->image.size = %d, %d", img_size.height, img_size.width);
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}

int main(int argc, char **argv) {
  if (argc != 2){ROS_INFO("please enter output file name!!"); return -1;}
  std::string file_dir = "../../../../data/";
  std::string filename(argv[1]);
  file_dir = file_dir + filename;
  ros::init(argc, argv, "snapshot");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/hik_cam_node/hik_camera", 10, imageCallback);
  while (true)
  {
    cv::Size img_size = out_img.size();
    if (img_size.width != 0){break;}
    ros::spinOnce();
  }
  cv::imwrite(file_dir, out_img);
  return 0;
}


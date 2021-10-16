#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 

static const std::string OPENCV_WINDOW = "Image window";

void colorPcCallback(const sensor_msgs::PointCloud2& msg)

{
    ROS_INFO("received color pc");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "topic_test");
    // 创建节点句柄
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/hik_cam_node/undist_camera", 10, imageCallback);
    ros::Subscriber color_pc_sub = nh.subscribe("/livox/color_lidar", 10, colorPcCallback);
    ROS_INFO("start listening topic /livox/color_lidar, /hik_cam_node/undist_camera");
 
    ros::spin();
    return 0;

}


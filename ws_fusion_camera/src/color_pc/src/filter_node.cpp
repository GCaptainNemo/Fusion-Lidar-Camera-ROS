#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;
bool IS_FILTER = true;

ros::Publisher pub_pcl_out1;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCallback(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() == 0) return;   
  PointCloudXYZ pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      // PointXYZINormal
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      pt.intensity = livox_msg->points[i].reflectivity;
      if (IS_FILTER && livox_msg->points[i].tag != 16)
      {
        // 去除噪点（1. 基于能量判断的噪点 2. 基于空间位置判断的噪点）
        continue;
      }
      pcl_in.push_back(pt);
    }
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  //   ROS_INFO("livox1 republish %u points at time %f buf size %ld",
  //   pcl_in.size(),
  //           timestamp.toSec(), livox_data.size());

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();
}

int main(int argc, char** argv) {
  // if (argc != 2){ROS_ERROR("need IS_FILTER as argument"); return -1;}
  if (strcmp(argv[1], "true") == 0){IS_FILTER = true; ROS_INFO("filter pointcloud");}
  else if(strcmp(argv[1], "false") == 0){IS_FILTER = false;ROS_INFO("don't filter pointcloud");}
  
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCallback);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_repub", 100);
  ros::spin();
}


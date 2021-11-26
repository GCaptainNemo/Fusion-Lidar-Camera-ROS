#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
// multi-sensor synchronized
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pthread.h>


// ///////////////////////////////////////////////////////////////////////
// topic: /livox/color_lidar
// frame: sensor_frame
// ///////////////////////////////////////////////////////////////////////



static bool IS_IMAGE_CORRECTION = true;




// /* 自定义的PointXYZRGBIL（pcl没有PointXYZRGBIL、PointXYZRGBI结构体）*/
// struct PointXYZRGBIL
// {
// 	PCL_ADD_POINT4D;
// 	PCL_ADD_RGB;
// 	uint32_t label;
// 	PCL_ADD_INTENSITY;
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(
// 	PointXYZRGBIL,
// 	(float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(uint32_t, label, label)(float, intensity, intensity))


// typedef PointXYZRGBIL PointType;

typedef pcl::PointXYZRGB PointType;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncPolicy;   
message_filters::Subscriber<sensor_msgs::Image> * image_sync_sub;
message_filters::Subscriber<sensor_msgs::PointCloud2> * livox_sync_sub;
message_filters::Synchronizer<syncPolicy> *sync_;

class ImageLivoxFusion
{
public:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    ros::Subscriber livox_sub;
    image_transport::Publisher image_sync_pub;
    ros::Publisher livox_sync_pub;
    ros::Publisher livox_pub;

    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::PointCloud out_pointcloud;
    sensor_msgs::PointCloud2 colored_msg; 
      

public:
    cv::Mat transform_matrix;    /**< from globol to image coordinate */
    cv::Mat intrinsic_matrix;    /**< from local to image coordinate  */
    cv::Mat extrinsic_matrix;    /**< from global to local coordinate */
    cv::Mat dist_matrix;         /**< dist parameters  */
    int img_height;
    int img_width;

public:
  ImageLivoxFusion():it(nh)
  {
    ROS_INFO("------------ intialize ----------\n");
    this->set_param();
    // ////////////////////////////////////////////////////////////////////////////////////////////
    // 多消息异步回调
    // ////////////////////////////////////////////////////////////////////////////////////////////

    image_sub = it.subscribe("/hik_cam_node/hik_camera", 100, &ImageLivoxFusion::imageCallback, this);
    livox_sub = nh.subscribe("/livox_repub", 100, &ImageLivoxFusion::livoxCallback, this);
   
    image_sync_pub = it.advertise("/hik_cam_node/undist_camera", 10);
    livox_sync_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/lidar_sync", 10);
    livox_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/color_lidar", 10);

    // ////////////////////////////////////////////////////////////////////////////////////////////
    // 多消息同步回调：需要传感器时间戳一致或相近
    // ////////////////////////////////////////////////////////////////////////////////////////////

    image_sync_sub = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/hik_cam_node/undist_camera",1);
    livox_sync_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "livox/lidar_sync",1);
    // multi sensor synchronized
    sync_ = new message_filters::Synchronizer<syncPolicy> (syncPolicy(10), *image_sync_sub, *livox_sync_sub);
    // image_pub = it.advertise("/hik_cam_node/undist_camera", 10);    
    // livox_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/color_lidar", 10);
    sync_->registerCallback(boost::bind(&ImageLivoxFusion::integral_callback, this, _1, _2));
    ROS_INFO("START LISTENING\n");
  };

  ~ImageLivoxFusion()
  {
  };

    void set_param();
    void livoxCallback(const sensor_msgs::PointCloud2Ptr & msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void integral_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &pc_msg);
};



void ImageLivoxFusion::set_param() 
{
  // extrinsic matrix parameters
  XmlRpc::XmlRpcValue param_list;
  std::vector<double> Extrin_matrix;
  if(!nh.getParam("/color_pc_sync_node/CameraExtrinsicMat/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Extrin_matrix.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  
  // Intrinsic matrix parameters
  std::vector<double> Intrinsic;
  if(!nh.getParam("/color_pc_sync_node/CameraMat/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        Intrinsic.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }

  // 5 distortion parameters
  std::vector<double> dist;
  if(!nh.getParam("/color_pc_sync_node/DistCoeff/data", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {    
        dist.push_back(double(tmp_value));
        ROS_INFO("PARAME SIZE = %f", double(tmp_value));
      }
  }
  
  // img size
  std::vector<int> img_size;
  if(!nh.getParam("/color_pc_sync_node/ImageSize", param_list))
      ROS_ERROR("Failed to get extrinsic parameter.");
  for (size_t i = 0; i < param_list.size(); ++i) 
  {
      XmlRpc::XmlRpcValue tmp_value = param_list[i];
      if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {    
        img_size.push_back(int(tmp_value));
        ROS_INFO("PARAME SIZE = %d", int(tmp_value));
      }
  }
  img_width = img_size[0];
  img_height = img_size[1];
  // convert cv::Mat
  cv::Mat dist_array(5, 1, CV_64F, &dist[0]);
  this->dist_matrix = dist_array.clone();

  
  cv::Mat Int(3, 3, CV_64F, &Intrinsic[0]);
  this->intrinsic_matrix = Int.clone();

  
  cv::Mat ext_(4, 4, CV_64F, &Extrin_matrix[0]);
  cv::Mat invRt = ext_(cv::Rect(0, 0, 3, 3));
  cv::Mat R = invRt.t();
  cv::Mat invT = -R * ext_(cv::Rect(3, 0, 1, 3));
  cv::hconcat(R, invT, this->extrinsic_matrix);
  // transform matrix: from global coordinate to image coordinate
  this->transform_matrix = Int * this->extrinsic_matrix;
}



void ImageLivoxFusion::livoxCallback(const sensor_msgs::PointCloud2Ptr & msg)
{
    msg->header.stamp = ros::Time::now();
    livox_sync_pub.publish(*msg);

}


void ImageLivoxFusion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // image correction
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    sensor_msgs::ImagePtr linshi_msg;
    if(IS_IMAGE_CORRECTION)
    {
        
        cv::Mat image_color;
        cv::undistort(cv_ptr->image, image_color, this->intrinsic_matrix, this->dist_matrix);
        cv_ptr->image = image_color.clone();
    }
    linshi_msg = cv_ptr->toImageMsg();
    linshi_msg->header.stamp = ros::Time::now();
    image_sync_pub.publish(linshi_msg);

  }


void ImageLivoxFusion::integral_callback(const sensor_msgs::ImageConstPtr &img_msg, 
                                          const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{

    // ROS_INFO("in call back");
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_color = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::fromROSMsg(*pc_msg, *raw_pcl_ptr);
    const int size = raw_pcl_ptr->points.size();
    pcl::PointCloud<PointType>::Ptr pc_xyzrgb(new pcl::PointCloud<PointType>);
    for (int i = 0; i < size; i++)
    {
        PointType pointRGB;
        pointRGB.x = raw_pcl_ptr->points[i].x;
        pointRGB.y = raw_pcl_ptr->points[i].y;
        pointRGB.z = raw_pcl_ptr->points[i].z;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(transform_matrix * pos);
        float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
        float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));

        // Trims viewport according to image size
        if (pointRGB.x >= 0)
        {
            if (x >= 0 && x < img_width && y >= 0 && y < img_height)
            {
            //  imread BGR（BITMAP）
            int row = int(y);
            int column = int(x);
            pointRGB.r = image_color.at<cv::Vec3b>(row, column)[2];
            pointRGB.g = image_color.at<cv::Vec3b>(row, column)[1];
            pointRGB.b = image_color.at<cv::Vec3b>(row, column)[0];
            
            // pointRGB.intensity = linshi_raw_pcl_ptr->points[i].intensity; //继承之前点云的intensity
            pc_xyzrgb->push_back(pointRGB);
            }
        }
    }
    pc_xyzrgb->width = 1;
    pc_xyzrgb->height = pc_xyzrgb->points.size();
    sensor_msgs::PointCloud2 colored_msg; 
    pcl::toROSMsg(*pc_xyzrgb,  colored_msg);  // 将点云转化为ROS消息发布
    colored_msg.header.frame_id = "sensor_frame";
    colored_msg.header.stamp = ros::Time::now(); 
    livox_pub.publish(colored_msg); 

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusion");
  if (argc != 2){ROS_ERROR("need is_correct as argument"); return -1;}; 
  
  if (strcmp(argv[1], "true") == 0){IS_IMAGE_CORRECTION = true; ROS_INFO("correct image");}
  else {IS_IMAGE_CORRECTION = false;ROS_INFO("don't correct image");}
  ImageLivoxFusion ic;
  ros::MultiThreadedSpinner spinner(4);  //节点多线程
  spinner.spin();   
  // ros::spin();
  return 0;
}

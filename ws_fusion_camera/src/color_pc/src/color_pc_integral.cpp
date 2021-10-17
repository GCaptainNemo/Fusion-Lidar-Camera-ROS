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
#include <pthread.h>


// ///////////////////////////////////////////////////////////////////////
// topic: /livox/color_lidar
// frame: sensor_frame
// ///////////////////////////////////////////////////////////////////////



static bool IS_IMAGE_CORRECTION = true;
// std::mutex mut_image;
std::mutex mut_pc;


//livox点云消息包含xyz和intensity
pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
pcl::PointCloud<pcl::PointXYZI>::Ptr linshi_raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr integral_color_pcl_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 

// pcl::PointCloud<pcl::PointXYZI> linshi_raw_pcl_ptr; 

cv::Mat image_color;
cv::Mat linshi_image_color;

// 发布判断
bool is_rec_image = false;
bool is_rec_lidar = false;

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


class ImageLivoxFusion
{
public:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  ros::Subscriber livox_sub;
  image_transport::Publisher image_pub;
  ros::Publisher livox_pub;
  cv_bridge::CvImagePtr cv_ptr;
  // message_filters::Subscriber<sensor_msgs::Image> image_sub;
  // message_filters::Subscriber<sensor_msgs::PointCloud2> livox_sub;
  // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> * sync;
  sensor_msgs::PointCloud out_pointcloud;
  sensor_msgs::PointCloud2 colored_msg; 
      
  pthread_t  tids1_;

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
    // it = image_transport::ImageTransport(nh);
    this->set_param();
    // Subscribe to input video feed and publish output video feed
    image_sub = it.subscribe("/hik_cam_node/hik_camera", 100, &ImageLivoxFusion::imageCallback, this);
    livox_sub = nh.subscribe("/livox/lidar", 100, &ImageLivoxFusion::livoxCallback, this);
   
    image_pub = it.advertise("/hik_cam_node/undist_camera", 10);    
    livox_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/color_lidar", 10);
    int ret1 = pthread_create(&tids1_, NULL,  publish_thread, this); //**point5**：线程创建，函数参
     // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/hik_cam_node/hik_camera",10);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> livox_sub(nh, "/livox/lidar",10);
    // multi sensor synchronized
    // typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncPolicy;
    // sync = new TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> (image_sub, livox_sub, 10);
    // Synchronizer<syncPolicy> sync(syncPolicy(10), image_sub, livox_sub);  
    // 指定一个回调函数，就可以实现两个话题数据的同步获取
    // sync->registerCallback(boost::bind(&ImageLivoxFusion::integral_callback, this, _1, _2));

    ROS_INFO("START LISTENING\n");
    
    
  };

  ~ImageLivoxFusion()
  {
  };

  void set_param();
  void livoxCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void integral_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &pc_msg);
  static void * publish_thread(void *  this_sub);
};

void * ImageLivoxFusion::publish_thread(void * args)
{
  ImageLivoxFusion * this_sub = (ImageLivoxFusion *) args; //**point3**：将void*入参转为类对象ImageLivoxFusion* 指针
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  
    if (is_rec_image && is_rec_lidar)
    {
      mut_pc.lock();
      pcl::copyPointCloud (*raw_pcl_ptr, *linshi_raw_pcl_ptr);
      // linshi_raw_pcl_ptr = raw_pcl_ptr;
      linshi_image_color = image_color.clone();
      mut_pc.unlock();
      
      pcl::PointCloud<PointType>::Ptr pc_xyzrgb(new pcl::PointCloud<PointType>);
      const int size = linshi_raw_pcl_ptr->points.size();
      for (int i = 0; i < size; i++)
      {
        // project get the photo coordinate
        // pcl::PointXYZRGB pointRGB;
        PointType pointRGB;

        pointRGB.x = linshi_raw_pcl_ptr->points[i].x;
        pointRGB.y = linshi_raw_pcl_ptr->points[i].y;
        pointRGB.z = linshi_raw_pcl_ptr->points[i].z;
        double a_[4] = { pointRGB.x, pointRGB.y, pointRGB.z, 1.0 };
        cv::Mat pos(4, 1, CV_64F, a_);
        cv::Mat newpos(this_sub->transform_matrix * pos);
        float x = (float)(newpos.at<double>(0, 0) / newpos.at<double>(2, 0));
        float y = (float)(newpos.at<double>(1, 0) / newpos.at<double>(2, 0));

        // Trims viewport according to image size
        if (pointRGB.x >= 0)
        {
          if (x >= 0 && x < this_sub->img_width && y >= 0 && y < this_sub->img_height)
          {
            //  imread BGR（BITMAP）
            int row = int(y);
            int column = int(x);
            pointRGB.r = linshi_image_color.at<cv::Vec3b>(row, column)[2];
            pointRGB.g = linshi_image_color.at<cv::Vec3b>(row, column)[1];
            pointRGB.b = linshi_image_color.at<cv::Vec3b>(row, column)[0];
           
            // pointRGB.intensity = linshi_raw_pcl_ptr->points[i].intensity; //继承之前点云的intensity
            pc_xyzrgb->push_back(pointRGB);
          }
        }
      }
      *integral_color_pcl_ptr = (*integral_color_pcl_ptr) + (*pc_xyzrgb);
      integral_color_pcl_ptr->width = 1;
      integral_color_pcl_ptr->height = integral_color_pcl_ptr->points.size();

      pcl::toROSMsg(*integral_color_pcl_ptr,  this_sub->colored_msg);  // 将点云转化为ROS消息发布
      this_sub->colored_msg.header.frame_id = "sensor_frame";
      this_sub->colored_msg.header.stamp = ros::Time::now();; // 时间戳和/livox/lidar 一致
      this_sub->livox_pub.publish(this_sub->colored_msg); 
      loop_rate.sleep();
    }
  }
}


void ImageLivoxFusion::set_param() 
{
  // extrinsic matrix parameters
  XmlRpc::XmlRpcValue param_list;
  std::vector<double> Extrin_matrix;
  if(!nh.getParam("/integral/CameraExtrinsicMat/data", param_list))
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
  if(!nh.getParam("/integral/CameraMat/data", param_list))
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
  if(!nh.getParam("/integral/CameraMat/data", param_list))
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
  if(!nh.getParam("/integral/ImageSize", param_list))
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



void ImageLivoxFusion::livoxCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{

  pcl::fromROSMsg(*msg, *raw_pcl_ptr);	
  is_rec_lidar = true;
}


void ImageLivoxFusion::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // image correction
  if(IS_IMAGE_CORRECTION)
  {
    cv::undistort(cv_ptr->image, image_color, this->intrinsic_matrix, this->dist_matrix);
    cv_ptr->image = image_color;
  }
  else
    image_color = cv_ptr->image;
  
  is_rec_image = true;
  image_pub.publish(cv_ptr->toImageMsg());
}


void ImageLivoxFusion::integral_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  // /////////////////////////////////////////////////////////////////////////////////////////////////
  // camera
  // /////////////////////////////////////////////////////////////////////////////////////////////////
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  // image correction
  cv::undistort(cv_ptr->image, image_color, this->intrinsic_matrix, this->dist_matrix);
  cv_ptr->image = image_color.clone();
  
  image_pub.publish(cv_ptr->toImageMsg());
  ROS_INFO("FINISH IMAGE\n");
  // /////////////////////////////////////////////////////////////////////////////////////////////////
  // lidar
  // /////////////////////////////////////////////////////////////////////////////////////////////////
  // sensor_msgs::PointCloud out_pointcloud;
	// sensor_msgs::convertPointCloud2ToPointCloud(*pc_msg, out_pointcloud);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  // this->project_get_rgb(&out_pointcloud, undist_image, pc_xyzrgb);
  // pc_xyzrgb->width = 1;
  // pc_xyzrgb->height = pc_xyzrgb->size();
  // ROS_INFO("size = %d \n", pc_xyzrgb->size());
  // pcl::toROSMsg( *pc_xyzrgb,  colored_msg);  //将点云转化为消息才能发布
  // colored_msg.header.frame_id = "sensor_frame";//帧id改成和velodyne一样的
  // livox_pub.publish(colored_msg); 
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "integral");
  if (argc != 2){ROS_ERROR("need is_correct as argument"); return -1;}; 
  
  if (strcmp(argv[1], "true") == 0){IS_IMAGE_CORRECTION = true; ROS_INFO("correct image");}
  else {IS_IMAGE_CORRECTION = false;ROS_INFO("don't correct image");}
  ImageLivoxFusion ic;
  ros::MultiThreadedSpinner spinner(10);  //节点多线程
  spinner.spin();   
  return 0;
}


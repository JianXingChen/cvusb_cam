#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
using namespace cv;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

cv::VideoCapture cap(0);

  if(!cap.isOpened())
  {
  ROS_ERROR("Open usb_cam failed");
    return 1;
  }
  cv::Mat                 frame1;
  sensor_msgs::ImagePtr   msg;
  std_msgs::Header            header;
nh.param("camera_frame_id", header.frame_id, std::string("mono_camera"));
  while (nh.ok()) {
    ros::Time();
    header.stamp=ros::Time::now();
    cap >> frame1;
   cv::cvtColor(frame1, frame1, CV_RGB2GRAY);
    if(!frame1.empty()) {
      msg = cv_bridge::CvImage(header, "mono8", frame1).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }
  }
}

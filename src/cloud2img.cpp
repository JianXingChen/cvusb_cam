#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include<set>
#include<sensor_msgs/PointCloud2.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/Image.h>
#include<iostream>
#include<boost/timer.hpp>

using namespace std;
typedef pcl::PointXYZRGBA PointRgbT;
typedef pcl::PointCloud<PointRgbT> PointCloudRgb;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
void pointCallback (const sensor_msgs::PointCloud2ConstPtr& point_in)
{
 // ROS_INFO("seq = %d : COME IN",point_in->header.seq);

   cv::Mat cvImage(point_in->height, point_in->width,0);
   std::stringstream imgname;
   PointCloudRgb  rgbCloud;
   PointCloudT   xyzCloud;
   PointCloudT   xyzCloudf;
   std::vector<int> mapping;

   pcl::fromROSMsg(*point_in,rgbCloud);
   pcl::copyPointCloud(rgbCloud, xyzCloud);
   pcl::removeNaNFromPointCloud(xyzCloud, xyzCloudf, mapping);
   std::set<int> smapping(mapping.begin(), mapping.end());

      for (int m = 0; m < xyzCloud.height; m++)
      {
              for (int n=0; n <xyzCloud.width; n++)
               {
                      if ( smapping.find(m*xyzCloud.width+n) != smapping.end())
                         {
                           cvImage.at<uchar>(m, n)=xyzCloud.points[m*xyzCloud.width+n].z*(8192/256);
                           }
                  }
      }
        imgname<<"/home/m/catkin_ws/"<<point_in->header.seq<<"_"<<point_in->header.frame_id<<".jpg";// save path
        cv::imwrite(imgname.str(),cvImage);
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "cloud2img");
        ros::NodeHandle nh;
        std::string scloudname;
        if (!nh.getParam("point2img_node/scloudname", scloudname))
        {
        ROS_ERROR("Failed to get  param 'point2img_node/scloudname', set  'point2img_node/scloudname'to defult value ");
        nh.param<std::string>("point2img_node/scloudname", scloudname, std::string("/kinect2/sd/points"));
        }
        ros::Subscriber   sub = nh.subscribe(scloudname,  50,pointCallback);
        ros::spin();

}

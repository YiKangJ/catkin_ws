#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  std::string cloud_topic = priv_nh_.param<std::string>("camera_depth_points", "camera/depth/points");
  std::string base_link = priv_nh_.param<std::string>("base_link", "base_link");
  std::string camera_link = priv_nh_.param<std::string>("camera_link", "camera_link");
 
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("camera_transformed_depth_points", 10);

  /*
   * LISTEN FOR POINTCLOUD
   */
  std::string topic = nh.resolveName(cloud_topic);
  ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic "<< topic);

  while (ros::ok())
  {
   
    sensor_msgs::PointCloud2::ConstPtr recent_cloud =
                 ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

    /*
     * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
     */
    tf::TransformListener listener;
    tf::StampedTransform stransform;
    try
    {
      listener.waitForTransform(base_link, recent_cloud->header.frame_id,  ros::Time::now(), ros::Duration(6.0));
      listener.lookupTransform(base_link, recent_cloud->header.frame_id,  ros::Time(0), stransform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    sensor_msgs::PointCloud2Ptr transformed_cloud(new sensor_msgs::PointCloud2);
//    sensor_msgs::PointCloud2::ConstPtr recent_cloud =
//                 ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);
    pcl_ros::transformPointCloud(base_link, stransform, *recent_cloud, *transformed_cloud);

    /*
     * PUBLISH TRANSFORMED_CLOUD
     */
    pcl_pub.publish(transformed_cloud);
  }

}

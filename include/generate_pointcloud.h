#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "fstream"


//reference: http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

// To get your laser scan as a set of 3D Cartesian (x,y,z) points, 
// you'll need to convert it to a point cloud message. 
// The sensor_msgs/PointCloud message looks like this:
// Header header
// geometry_msgs/Point32[] points # Vector of 3d points (x,y,z)
// ChannelFloat32[] channels # Space for extra information like color.

// transformLaserScanToPointCloud function uses tf to transform your laser scan into a point cloud in another (preferably fixed) frame.
// If your robot is moving, choose a stationary map frame. 
// That way, point clouds from one time are comparable to point clouds at another time (or any other data in your system).


class LaserScanToPointCloud{

 public:
  
  LaserScanToPointCloud(ros::NodeHandle n);

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
  
};
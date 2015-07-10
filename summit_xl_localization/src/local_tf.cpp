#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "tf/message_filter.h"
#include <tf/transform_listener.h>
#include "message_filters/subscriber.h"
#include <math.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h" 

class LocalTF
{
public:
  LocalTF(ros::NodeHandle h) : nh_(h), tf_listener(),  target_frame_("base_footprint_fake")
  {
    sub_=nh_.subscribe("/mavros/imu/data", 5, &LocalTF::imuCallback,this);
    pub_=nh_.advertise<sensor_msgs::Imu>("/mavros/imu_data_new", 5);

    tf_listener.waitForTransform("/base_footprint_fake", "/fcu",
                              ros::Time::now(), ros::Duration(3.0));
  } 

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_ptr) 
  {
    sensor_msgs::Imu imu_out;

    tf::Stamped<tf::Quaternion> orient_in, orient_out;
    tf::quaternionMsgToTF(imu_ptr->orientation, orient_in);
    orient_in.frame_id_ = imu_ptr->header.frame_id;
    ROS_INFO("ERROR");
    tf_listener.transformQuaternion(target_frame_, orient_in, orient_out);
    tf::quaternionTFToMsg(orient_out, imu_out.orientation);
 
    tf::Stamped<tf::Vector3> vel_in, vel_out;
    tf::vector3MsgToTF(imu_ptr->angular_velocity, vel_in);
    vel_in.frame_id_ = imu_ptr->header.frame_id;
    tf_listener.transformVector(target_frame_, vel_in, vel_out);
    tf::vector3TFToMsg(vel_out, imu_out.angular_velocity);
   
    tf::Stamped<tf::Vector3> accel_in, accel_out;
    tf::vector3MsgToTF(imu_ptr->linear_acceleration, accel_in);
    accel_in.frame_id_ = imu_ptr->header.frame_id;
    tf_listener.transformVector(target_frame_, accel_in, accel_out);
    tf::vector3TFToMsg(accel_out, imu_out.linear_acceleration);
 
    imu_out.header.stamp = imu_ptr->header.stamp;
    imu_out.header.frame_id = target_frame_;

    pub_.publish(imu_out);
  }


private:
  tf::TransformListener tf_listener;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;

  std::string target_frame_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "local_tf"); //Init ROS

  ros::NodeHandle nh;  
  LocalTF ltf(nh);
  ros::spin();

  return 0;
};



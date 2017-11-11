//删除rplidar获得激光数据中的一部分数据

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <std_msgs/Float32.h>

#define DISCARDED_NUM 90    

class laser_data_process_node
{
private:
  ros::NodeHandle node_;
  ros::Subscriber laser_sub;
  ros::Publisher laser_pub;
  sensor_msgs::LaserScan laser_send;
  
public:  
  explicit laser_data_process_node(const ros::NodeHandle& nh):
  node_(nh)
  {
    //human_sub_ = node_.subscribe<people_msgs::PositionMeasurementArray>("/people_tracker_measurements", 1000, boost::bind(&human_trajectory_node::humanCallBack,this,_1));
    //marker_pub_ = node_.advertise<visualization_msgs::Marker>("visualization_marker",10);
    laser_sub = node_.subscribe<sensor_msgs::LaserScan>("/myscan", 1000, boost::bind(&laser_data_process_node::laserCallBack, this, _1));
    laser_pub = node_.advertise<sensor_msgs::LaserScan>("/scan", 10);
  }
  
  void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    //std::cout<<"called!!!"<<std::endl;
    //std::cout<<msg->ranges.size()<<std::endl;
    float inf = 1.0 / 0.0;
    //std::cout<<a<<std::endl;
    
    laser_send = *msg;
    
    int medium = 0;
    for(int i=0;i<360;i++)
    {
      if(laser_send.ranges[i]>4)
      {
	laser_send.ranges[i] = 4;
      }
    }
    for(int i=medium-DISCARDED_NUM; i<medium+DISCARDED_NUM; i++)
    {
      if(i<0)
      {
	laser_send.ranges[i+360] = 4;
      }
      else{
      laser_send.ranges[i] = 4;
      }
    }
    
    
    laser_pub.publish(laser_send);
  }
  
  
  
  ~laser_data_process_node()
  {};
  

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_data_process");
  ros::NodeHandle n;
  laser_data_process_node laser_data_process_node(n);
  ros::spin();
  return 1;
}
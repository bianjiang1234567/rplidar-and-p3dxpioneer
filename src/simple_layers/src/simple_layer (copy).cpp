#include <simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  //ros::NodeHandle nh("~/" + name_);
  //int argc = 0;
  //char** argv =0;
  //ros::init(argc,argv,"Layer_node");
  //ros::init(argc,argv,"simple_layer_namespace");
 // ros::init();
  ros::NodeHandle nh("~/" + name_);
 // nh.param("update_steps", update_steps, 10);
 // nh.param("update_freq", update_freq, 10.0);
    ros::Subscriber sub_ =nh.subscribe("/mychatter",1000,&SimpleLayer::getTargets,this);

  //std::string name1 = "~/"+name_;
  //std::cout<<name1<<"!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

  std::cout<<"123456!!!!!!!!!!!!!!!!!!!!1234"<<std::endl;
  current_ = true;
  

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
 // ros::Subscriber sub_ =nh.subscribe("mychatter",1000,&SimpleLayer::getTargets,this);
  //ros::Subscriber sub_ =nh.subscribe<geometry_msgs::Point>("/mychatter",1000,boost::bind(&SimpleLayer::getTargets,this,_1));
  //ros::Subscriber sub_ =nh.subscribe<nav_msgs::Odometry>("RosAria/pose",1000,boost::bind(&SimpleLayer::poseMessageReceived,this,_1));
  //ros::Subscriber sub_ =nh.subscribe("move_base/goal",1000,&SimpleLayer::batteryChargeStateMessageReceived,this);
  std::cout<<"123456!!!!!!!!!!!!!!!!!!!!"<<std::endl;

  //nh.subscribe("RosAria/battery_charge_state", 1000, &batteryChargeStateMessageReceived) ; //inform charge state
 //ros::spin();
}

/*
void SimpleLayer::batteryChargeStateMessageReceived(const move_base_msgs/MoveBaseActionGoal& msg)
{
	// Right now this feature is not included in the pioneer-3 robot
  	//ROS_INFO_STREAM("The battery state of charge is "<<msg);
	std::cout<<"called!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
}
*/
void SimpleLayer::getTargets(const geometry_msgs::Point::ConstPtr& mypoint)
{
  //mark_x_=mypoint.x;
  //mark_y_=mypoint.y;
  std::cout<<"called!!!!!!!!!!!!!!!!!!!!"<<std::endl;
  int x;
}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = origin_x + cos(origin_yaw);
  mark_y_ = origin_y + sin(origin_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

}

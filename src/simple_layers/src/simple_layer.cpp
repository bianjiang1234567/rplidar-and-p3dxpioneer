/*#include <simple_layers/simple_layer.h>
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


void SimpleLayer::batteryChargeStateMessageReceived(const move_base_msgs/MoveBaseActionGoal& msg)
{
	// Right now this feature is not included in the pioneer-3 robot
  	//ROS_INFO_STREAM("The battery state of charge is "<<msg);
	std::cout<<"called!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
}

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
*/

// PedestrianLayer.cpp --- 
// 
// Filename: PedestrianLayer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Fri Jun 20 15:32:53 2014 (+0100)
// Version: Wed Jun 25 13:45:52 2014 (+0100)
// Last-Updated: 
//           By: 
//     Update #: 4
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// The MIT License (MIT)
// 
// Copyright (c) 2014 Federico Boniardi
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// 

// Code:


#include <simple_layers/simple_layer.h>
//#include "target_to_pedestrian/TargetToPedestrian.h"
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace simple_layer_namespace {

double l2distance(std::pair<double,double> x1, std::pair<double,double> x2)
{
  return std::sqrt(x1.first * x1.first + x2.second * x2.second);
}

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  std::string pedestrians_topic;
  double costmap_update_freq, resolution, inflation_radius;
    
  current_ = true;

  if( !nh.getParam("pedestrians_topic", pedestrians_topic) )
    pedestrians_topic = "/mychatter";
  nh.param("/move_base/local_costmap/pedestrian_layer/update_steps", update_steps_, 1);
  nh.param("/move_base/local_costmap/pedestrian_layer/update_freq", update_freq_, 1.0);
  nh.param("/move_base/local_costmap/pedestrian_layer/decay_constant", decay_constant_, 10.0);
  nh.param("/move_base/local_costmap/pedestrian_layer/pedestrian_radius", pedestrian_inflation_radius_, 0.5);
  nh.param("/move_base/local_costmap/resolution", resolution, 0.05);
  nh.param("/move_base/local_costmap/inflation_radius", inflation_radius, 0.5);
  nh.param("/move_base/local_costmap/update_frequency", costmap_update_freq, 5.0);

  window_radius_ = int((0.5*inflation_radius+0.5*pedestrian_inflation_radius_)/resolution);
  
  update_costs_ = false;

  if( update_freq_ > costmap_update_freq ) {
    ROS_WARN("update rate of 'pedestrian_layer' is greater than 'update_frequency' of local_costmap. update_freq will be set to %f ", 0.2*costmap_update_freq);
    update_ratio_ = 5;
  } else {
    update_ratio_ = int(costmap_update_freq/update_freq_);
  }

  count_ = -1;
  
  sub_ = nh.subscribe("/mychatter", 1000, &SimpleLayer::getTargets, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  mark_x_=0;
  mark_y_=0;
  mark_x2_=0;
  mark_y2_=0;
  
}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
/*
void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
  
  
  
  /*
  if ( !enabled_ )
    return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);
  
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
  
  if( ++count_ % update_ratio_ == 0 ) {
    count_ %= update_steps_;
    clearing_ = mark_;
    mark_.clear();

    /*for(unsigned int i=0; i<pedestrian_.size(); ++i) {
      double mark_x = pedestrian_[i].pose.x  + update_steps_ * pedestrian_[i].velocity * std::cos(pedestrian_[i].pose.theta) / update_freq_ ;
      double mark_y = pedestrian_[i].pose.y  + update_steps_ * pedestrian_[i].velocity * std::sin(pedestrian_[i].pose.theta) / update_freq_ ;
      std::pair<double, double> mark(mark_x, mark_y);
      if( l2distance(mark, std::pair<double, double>(robot_x, robot_y)) > 0.40 ) {
        mark_.push_back(mark);
        *min_x = std::min(*min_x, mark_x);
        *min_y = std::min(*min_y, mark_y);
        *max_x = std::max(*max_x, mark_x);
        *max_y = std::max(*max_y, mark_y);
      }
    }
    //update_costs_ = true;
 // }
}
*/

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

 // mark_x_ = origin_x + cos(origin_yaw);
 // mark_y_ = origin_y + sin(origin_yaw)-2;

  //mark_x_ = 1;
  //mark_y_ = -3;
  
  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
  std::cout<<"updateBounds!!!!!!!!!!!!!!!!!"<<std::endl;
  std::cout<<"minx="<<*min_x<<std::endl;
  std::cout<<"miny="<<*min_y<<std::endl;
  std::cout<<"maxx="<<*max_x<<std::endl;
  std::cout<<"maxy="<<*max_y<<std::endl;
  std::cout<<"mark_x_="<<mark_x_<<std::endl;
  std::cout<<"mark_y_="<<mark_y_<<std::endl;
}

/*
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                      int min_i, int min_j, int max_i, int max_j)
{
  if( !enabled_ )
      return;

  if( update_costs_ ) {
    unsigned int mx, cx;
    unsigned int my, cy;
    for(unsigned int i=0; i<clearing_.size(); ++i) {
      if( master_grid.worldToMap(clearing_[i].first, clearing_[i].second, cx, cy) ) { 
        for(int i=(-1)*window_radius_; i<window_radius_+1; ++i) 
          for(int j=(-1)*window_radius_; j<window_radius_+1; ++j) 
            master_grid.setCost(cx-i, cy-j, FREE_SPACE);
      }
    }
    for(unsigned int i=0; i<mark_.size(); ++i) {
      if( master_grid.worldToMap(mark_[i].first, mark_[i].second, mx, my) ) {
        for(int i=(-1)*window_radius_; i<window_radius_+1; ++i) 
          for(int j=(-1)*window_radius_; j<window_radius_+1; ++j) 
            master_grid.setCost(mx-i, my-j, this->pedestrian_cost(mx,my,i,j));
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
    update_costs_ = false;
  }
  
  
}
*/


void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  
  //unsigned int mark_x2_=mark_x_+1;
  //unsigned int mark_y2_=mark_y_+6;
  
  double k1= (double)(mark_y2_-mark_y_)/(mark_x2_-mark_x_);
  
  double b1= mark_y_-k1*mark_x_;
  
  double k2= (double)(mark_x2_-mark_x_)/(mark_y2_-mark_y_);
  
  double b2= mark_x_-k2*mark_y_;
  
  std::cout<<"K1="<<k1<<std::endl;
  std::cout<<"K2="<<k2<<std::endl;
  std::cout<<"b1="<<b1<<std::endl;
  std::cout<<"b2="<<b2<<std::endl;
  
   std::cout<<"mark_x_="<<mark_x_<<std::endl;
   std::cout<<"mark_y_="<<mark_y_<<std::endl;
   std::cout<<"mark_x2_="<<mark_x2_<<std::endl;
   std::cout<<"mark_y2_="<<mark_y2_<<std::endl;
  
  double dis= sqrt((mark_x2_-mark_x_)*(mark_x2_-mark_x_)+(mark_y2_-mark_y_)*(mark_y2_-mark_y_));
  if(dis<10 && abs(mark_x2_-mark_x_)>=abs(mark_y2_-mark_y_))
  {
    for (int i=0; i<= (int)abs(mark_x2_-mark_x_); i++ )
    {
      if(master_grid.worldToMap(mark_x_+i, k1*(mark_x_+i)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.1, k1*(mark_x_+i+0.1)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.2, k1*(mark_x_+i+0.2)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.3, k1*(mark_x_+i+0.3)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.4, k1*(mark_x_+i+0.4)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.5, k1*(mark_x_+i+0.5)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.6, k1*(mark_x_+i+0.6)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.7, k1*(mark_x_+i+0.7)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(mark_x_+i+0.8, k1*(mark_x_+i+0.8)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      std::cout<<"COME IN COME IN !!!!!"<<std::endl;
      }
      if(master_grid.worldToMap(mark_x_+i+0.9, k1*(mark_x_+i+0.9)+b1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
  }
  if(dis<10 && abs(mark_x2_-mark_x_)<abs(mark_y2_-mark_y_))
  {
    for(int i=0; i<= (int)abs(mark_y2_-mark_y_);i++)
    {
      if(master_grid.worldToMap(k2*(mark_y_+i)+b2, mark_y_+i, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.1)+b2, mark_y_+i+0.1, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.2)+b2, mark_y_+i+0.2, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.3)+b2, mark_y_+i+0.3, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.4)+b2, mark_y_+i+0.4, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.5)+b2, mark_y_+i+0.5, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.6)+b2, mark_y_+i+0.6, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.7)+b2, mark_y_+i+0.7, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.8)+b2, mark_y_+i+0.8, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
      if(master_grid.worldToMap(k2*(mark_y_+i+0.9)+b2, mark_y_+i+0.9, mx, my)){
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
  }
  //if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    //master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  //}
}




//void SimpleLayer::getTargets(const geometry_msgs::Point::ConstPtr& tgts_msg)
void SimpleLayer::getTargets(const visualization_msgs::Marker::ConstPtr& tgts_msg)
{
  // Target estimations are in world frame. To convert to map frame swap x=y,y=-x
  /*pedestrian_.clear();
  for(unsigned int i=0; i<tgts_msg->identities.size(); ++i) {
    target_to_pedestrian::PedestrianEstimation ped;
    ped.id = tgts_msg->identities.at(i);
    ped.pose.x = tgts_msg->positions.at(i).y; 
    ped.pose.y = -tgts_msg->positions.at(i).x; 
    ped.velocity = std::sqrt(std::pow(tgts_msg->velocities.at(i).x, 2) +
                             std::pow(tgts_msg->velocities.at(i).y, 2));
    ped.pose.theta = (ped.velocity > 0 ? std::asin(tgts_msg->velocities.at(i).y/ped.velocity) : 0); // reverse the angle
    ped.std_deviation_x = std::pow(tgts_msg->standardDeviations.at(i).y, 2);
    ped.std_deviation_y = std::pow(tgts_msg->standardDeviations.at(i).x, 2);
    pedestrian_.push_back(ped);
  
  }*/
   //mark_x_=tgts_msg->x;
   //mark_y_=tgts_msg->y;
   mark_x_=tgts_msg->points[0].x;
   mark_y_=tgts_msg->points[0].y;
   mark_x2_=tgts_msg->points[1].x;
   mark_y2_=tgts_msg->points[1].y;
   std::cout<<"called!!!!!!!!!!!!"<<std::endl;
   std::cout<<"mark_x_="<<mark_x_;
   std::cout<<"mark_y_="<<mark_y_;
   std::cout<<"mark_x2_="<<mark_x2_;
   std::cout<<"mark_y2_="<<mark_y2_;
   
}

int  SimpleLayer::pedestrian_cost(unsigned int mx, unsigned int my, int i, int j)
{
  double d = std::sqrt(double(i*i+j*j)); 
  return int(std::exp(-0.001*decay_constant_*d*d)*(INSCRIBED_INFLATED_OBSTACLE-1)*(d <= window_radius_)); 
}

}
// 
// PedestrianLayer.cpp ends here

// Drive.h: interface for the CDrive class.
/***********************************************************************/
/*  Project         -     Robot driver                                 */
/*  File Name       -     drive.h                                      */
/*  Description     -     interface for the CDrive class.              */
/***********************************************************************/
#ifndef  DRIVE_H_
#define	 DRIVE_H_

#include "SerialPort.h"	
#include <ros/time.h>
#include <ros/ros.h>

#define DEBUG_DRIVE		1
#define USING_ODOM_COMBINED	1
#define LONG_TIME_NO_COMMAND_CHECKER	1

union UcFloConv
{
	unsigned char dat[4];
	float f;
};


class CDrive  
{
public:
	CDrive();
	virtual ~CDrive();


public:
	ros::NodeHandle nh_;
	ros::Subscriber velocity_sub_;
	ros::Publisher odom_pub;
	ros::Timer timer_;

	CSerialPort m_spPort;
	//unsigned char sendBuf[32];
	char sendBuf[];
	unsigned char recBuf[9]; // mainly for odometry data
	UcFloConv u2f;  // unsigned char ch[4] -> float 
};


#endif 	


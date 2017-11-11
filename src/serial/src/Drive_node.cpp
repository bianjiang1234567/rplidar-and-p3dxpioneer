/***********************************************************************/
/*  Project         -     Robot Drive                                       */
/*  File Name       -     drive.cpp                                         */
/*  Purpose         -     Implement the driving function of robot platform. */
/****************************************************************************/

#include "Drive.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>




const char PORT[] = "/dev/ttyUSB0";


CDrive::CDrive()
{

	if(m_spPort.OpenPort( PORT ) < 0) //
	{
		fprintf(stderr, "Fail Open Serial Port!\n");
		exit(0);
	}

	if(m_spPort.SetPortPars(115200, 8, 1, 'N'))
		printf(">>>Serial Port: 115200, 8N1 setted! \n");

	sendBuf[0] = 'j';  //cm/s -> rpm
	sendBuf[3] = 'j';
	sendBuf[4] = 'j';
	sendBuf[1] = 'j';
	sendBuf[2] = 'k';
	sendBuf[5] = 0;
	
	printf("%s", sendBuf);
	m_spPort.SendData(sendBuf, 5);
}


CDrive::~CDrive()
{
	m_spPort.ClosePort();
	printf("Serial port closed! \n");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Diff_Drive_Node");

	CDrive arm_raise();
	
	
	
}




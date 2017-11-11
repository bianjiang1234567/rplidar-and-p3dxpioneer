// SerialPort.cpp: implementation of the CSerialPort class.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <paths.h>
#include <sysexits.h>
#include <termios.h>
#include <sys/param.h>
#include "SerialPort.h"


CSerialPort::CSerialPort()
{
	hCom = -1;
	m_bOpened = false;
}


CSerialPort::~CSerialPort()
{
	ClosePort();
}


int CSerialPort::OpenPort(const char * dev)
{
	hCom = open(dev, O_RDWR | O_NOCTTY | O_NDELAY ); 
	if (-1 == hCom)   
	{       
		printf ("> [SerialComm] Can't Open Serial Port on [%s]...\n", dev);
		return -1;       
	}

	m_bOpened = true;
	return hCom;
}


bool CSerialPort::IsPortOpened()
{
	return m_bOpened;
}


int CSerialPort::fd()
{
	return hCom;
}


void CSerialPort::ClosePort()
{
	if( m_bOpened )
	{
		if(close(hCom) != -1);
			printf("Serial port /dev/ttyUSB%d closed! \n,", hCom);
		m_bOpened = false;
	}
}


bool CSerialPort::SetPortPars(int baud, unsigned char databits, unsigned char stopbits, char parity)
{
	if(!m_bOpened)
		return false;
//  get current options for the port:
	struct termios Opt;
	tcgetattr(hCom, &Opt);

// setting baud rate:	
	switch(baud)
	{
		case 9600	: cfsetispeed(&Opt, B9600)	; cfsetospeed(&Opt, B9600);  break;
		case 19200	: cfsetispeed(&Opt, B19200)	; cfsetospeed(&Opt, B19200);  break;
		case 38400	: cfsetispeed(&Opt, B38400)	; cfsetospeed(&Opt, B38400);  break;
		case 57600	: cfsetispeed(&Opt, B57600)	; cfsetospeed(&Opt, B57600);  break;
		case 115200	: cfsetispeed(&Opt, B115200); cfsetospeed(&Opt, B115200);  break;
		default     : cfsetispeed(&Opt, B9600)	; cfsetospeed(&Opt, B9600);  break;
	}

// setting data bit	
	Opt.c_cflag &= ~CSIZE;// Mask the character size bits 
	switch (databits) /*????????λ??*/
	{  
        case 7: Opt.c_cflag |= CS7; break;
		case 8: Opt.c_cflag |= CS8; break;
		default:Opt.c_cflag |= CS8; break;
	}

// setting stop bit 
	switch (stopbits)
	{  
        case 1: Opt.c_cflag &= ~CSTOPB; break;
        case 2: Opt.c_cflag |= CSTOPB;  break;
		default:Opt.c_cflag &= ~CSTOPB; break; 
	}

// setting parity
	switch (parity)
	{  
        case 'O':
		case 'o':	
            Opt.c_cflag |= (PARENB | PARODD); /* ????Ϊ??Ч??*/ 
            Opt.c_iflag |= (INPCK | ISTRIP); /* Enable parity checking and strip the parity bit*/
            break;
        case 'E':
		case 'e':	
            Opt.c_cflag |= PARENB;     /* Enable parity */   
            Opt.c_cflag &= ~PARODD;   /* ת??ΪżЧ??*/    
            Opt.c_iflag |= (INPCK | ISTRIP); /* Enable parity checking */
            break;
		default:
		case 'n':
        case 'N':
			Opt.c_cflag &= ~PARENB;
			Opt.c_iflag &= ~INPCK;
		   	break;
	}

	Opt.c_cflag |= (CLOCAL | CREAD); //CREAD: Enable receiver,CLOCAL: Local line - do not change "owner" of port
	//	Opt.c_cflag |= CNEW_RTSCTS; // ENABLE hardware flow;
	//  Opt.c_cflag &= ~CNEW_RTSCTS; // disable hardware flow;

	Opt.c_lflag &= ~(ICANON | ECHO | ECHOE| ECHOK | ECHONL | ISIG);  // Original Input
	Opt.c_oflag &= ~OPOST;   /*Original Output*/
    Opt.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);
	
	fcntl(hCom, F_SETFL, 0); //set read() as BLOCKing behavior
	Opt.c_cc[VMIN] = 0;  /* define the minimum bytes data to be readed*/
	Opt.c_cc[VTIME] = 1;

	// Write Opt struct Back, setting all
	tcsetattr(hCom, TCSANOW, &Opt); 
	printf ("Serial port set to %d baud rate %d%c%d ...\n", baud, databits, parity, stopbits);

	return true;
}


int CSerialPort::SendData(char *szBuffer, int len)
{
	int nwrite;
	if(!IsPortOpened())
		return -1;

	while(len > 0)
	{
		nwrite = write(hCom, szBuffer, len);
		if(nwrite < 1)
		{
			printf("[WriteComm] error due to %d:%s\n", errno, strerror(errno));
			return false;
		}
		len -= nwrite;
		szBuffer += nwrite;
	}
	return len;
}


int CSerialPort::SendData(unsigned char *szBuffer, int len)
{
	int nwrite;
	if(!IsPortOpened())
		return -1;

	while(len > 0)
	{
		nwrite = write(hCom, szBuffer, len);
		if(nwrite < 1)
		{
			fprintf(stderr, "error due to %d:%s\n", errno, strerror(errno));
			return false;
		}
		len -= nwrite;
		szBuffer += nwrite;
	}
	return len;
}

int CSerialPort::SendData(const char *szBuffer, int len)
{
	int nwrite;
	if(!IsPortOpened())
		return -1;

	while(len > 0)
	{
		nwrite = write(hCom, szBuffer, len);
		if(nwrite < 1)
		{
			fprintf(stderr, "Error due to %d:%s\n", errno, strerror(errno));
			return false;
		}
		len -= nwrite;
		szBuffer += nwrite;
	}
	return len;
}


/*
bool CSerialPort::ReadAllAndPrint()
{
	int n = 0;
	int ret = 0;
	ioctl(hCom, FIONREAD, &n);
	ret = read(hCom, pRecDat, n);
	pRecDat[n] = 0;

	printf("%s", pRecDat);
	return ret;
}
*/


int CSerialPort::ReadData(unsigned char *pData, int len)
{
	int n = 0;
	int nread = 0;
	int readtimes = 0;

	int retval;
	struct timeval	tv;
	tv.tv_sec = 3;
	tv.tv_usec = 0;

	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(hCom, &rfds);


	retval = select(hCom + 1, &rfds, NULL, NULL, &tv);
	if (retval == -1)
		perror("select()");
	else if (retval)
	{
	
		do{
			n = read(hCom, &pData[nread], len);
			if(n < 0)
			{
				fprintf(stderr, "CSerialPort::ReadData: %s(errno: %d)\n", strerror(errno), errno);
				return n;
			}
			else if(n == 0)
			{
				if(readtimes++ >= 1000)
				{
					fprintf(stderr, "read 1000 time\n");
					return -1;
				}
				continue;
			}
			else
			{
				nread += n;
				len -= n;
			}
		}while(len);
		
	}	
	else
		fprintf(stderr, "ReadData: no data after 3 second\n");

	return nread;
}


int CSerialPort::ReadData(char *pData, int len)
{
	int n = 0;
	int nread = 0;
	int readtimes = 0;
	int retval;
	struct timeval	tv;
	tv.tv_sec = 3;
	tv.tv_usec = 0;

	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(hCom, &rfds);
	if(!m_bOpened)
	{
		printf("Serial port is not opened!\n");
		return false;
	}

	retval = select(hCom + 1, &rfds, NULL, NULL, &tv);
	if (retval == -1)
		perror("select()");
	else if (retval)
	{
		do{
			n = read(hCom, &pData[nread], len);
			if(n < 0)
			{
				fprintf(stderr, "CSerialPort::ReadData: %s(errno: %d)\n", strerror(errno), errno);
				return n;
			}
			else if(n == 0)
			{
				if(readtimes++ >= 1000)
				{
					fprintf(stderr, "read 1000 time\n");
					return -1;
				}
				continue;
			}
			else
			{
				nread += n;
				len -= n;
			}
		}while(len);
	}	
	else
		fprintf(stderr, "ReadData: no data after 3 second\n");
	return nread;
}

bool CSerialPort::ClearInputBuffer()
{
	tcflush(hCom, TCIOFLUSH);
	return true;
}

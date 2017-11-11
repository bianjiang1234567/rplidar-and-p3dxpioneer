#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#define	BAUDRATE	115200
#define DATABITS	8
#define PARITY		'N'
#define STOPBITS	1


class CSerialPort  
{
public:
	CSerialPort();
	virtual ~CSerialPort();

	int  OpenPort(const char *dev);
	void ClosePort();
	int  SendData(char *szBuffer, int len);
	int  SendData(unsigned char *szBuffer, int len);
	int  SendData(const char *, int);
	bool SetPortPars(int BaudRate, unsigned char ByteSize, unsigned char StopBit, char Parity);

	int  ReadData(unsigned char *pData, int len);
	int  ReadData(char *pData, int nBufLen);

	bool ClearInputBuffer();
	bool IsPortOpened();
	int  fd();
	
	bool ReadAllAndPrint();

private:	
	int	hCom;
	bool m_bOpened;
};

#endif

#pragma once
/******************** (C) COPYRIGHT 2021 Shenzhen LeiShen Intelligent System Co., Ltd. *****************

* File Name         : LSLidarDemo.h
* Author            : LSJ
* Version           : v2.1.0
* Date              : 2023/09
* Modify Record		: 
* Description       : Initial Version


*******************************************************************************
* Version           : v2.1.1
* Date              : 2023/10
* Modify Record		: C16 v4.0 bug
* Description       : C16 v4.0 fixed the incorrect display of coordinate conversions


*******************************************************************************
* Version           : v2.1.2
* Date              : 2024/02
* Modify Record		: Change the function that gets a frame of data
* Description       : std::shared_ptr<std::vector<MuchLidarData>> getLidarPerFrameDate();   
					  (Change) ->
                      bool getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info);

*******************************************************************************

* *******************************************************************************
* Version           : v2.1.3
* Date              : 2024/03
* Modify Record		: All set functions add message feedback
* Description       : example:
					  bool setLidarRotateSpeed(int SpeedValue);			
					  (Change) ->
                     bool setLidarRotateSpeed(int SpeedValue, std::string& InfoString); 

*******************************************************************************/

#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <cmath>
#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <regex>
#include <memory>

#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
typedef uint64_t u_int64;
#else
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef unsigned __int64 u_int64;

#define timegm _mkgmtime
#endif

#define PI 3.1415926
#define ConversionUnit	100.0			//unit conversion: convert the centimeter to meter

typedef struct _Point_XYZ
{
	float x1 = 0.0;
	float y1 = 0.0;
	float z1 = 0.0;

	float x2 = 0.0;
	float y2 = 0.0;
	float z2 = 0.0;
}m_PointXYZ;

typedef struct _MuchLidarData
{
	float X = 0.0;
	float Y = 0.0;
	float Z = 0.0;
	int ID = 0;
	float H_angle = 0.0;
	float V_angle = 0.0;
	float Distance = 0.0;
	int Intensity = 0;
	u_int64 Mtimestamp_nsce = 0;
}MuchLidarData;

typedef struct _UTC_Time
{
	int year = 2000;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
}UTC_Time;

typedef struct _LidarStateParam
{
	std::string LidarIP = "-1,-1,-1,-1";			//Lidar IP
	std::string ComputerIP = "-1,-1,-1,-1";			//Lidar destination IP（computer IP）
	std::string GatewayIP = "-1,-1,-1,-1";			//Lidar Gateway IP（Gateway IP）
	std::string SubnetMaskIP = "-1,-1,-1,-1";		//Lidar Subnet Mask IP（Subnet Mask IP）
	int DataPort = -1;								//Lidar data pakcet port
	int DevPort = -1;								//Lidar device pakcet port

	float ReceiverTemperature = -1.0;				//Receiver plate temperature    Unit:°
	float ReceiverHighVoltage = -1.0;				//Receiving plate high voltage  Unit:volt
	float MotorSpeed = -1;							//motor speed					Unit:revolution
	int	  PTP_State = -1;							//PPS State:					1:Losed;	0: Not lost 
	int	  GPS_State = -1;							//PPS State:					1:Losed;	0: Not lost 
	int	  PPS_State = -1;							//PPS State:					1:Losed;	0: Not lost 
	int   StandbyMode = -1;							//Standby Mode					0:Normal	1: Standby
	int   Clock_Source = -1;						//Clock Source:					0:GPS;		1: PTP_L2;	 2:Reserved;		3:PTP_UDPV4
	int   PhaseLockedSwich = -1;					//Phase Locked Swich:			0:Closing;	1: Opening
	int   PhaseLockedState = -1;					//Phase Locked State:			0:Unlocked; 1: Locked
	double RunningTime = -1;						//Running Time					Unit: hour
}LidarStateParam;

typedef std::function<void(std::shared_ptr<std::vector<MuchLidarData>>, int, std::string)> FunDataPrt;

class GetLidarData
{
public:
	GetLidarData();
	~GetLidarData();

	bool isFrameOK = false;
	int isDormant = 0;                                                   //whether lidar is in low power mode or not, 0: normal mode, 1:low power mode
	bool isFrames = true;                                               //frame synchronization judgement，true: normal
	bool isHighVoltage = true;                                          //whether voltage is normal, true: normal
	bool islidarDevCome = false;													//judge whether obtain lidar device packet

	double cosAngleValue[360000];										//Pre-calculate the Angle values of cos and sin	
	double sinAngleValue[360000];

	std::string mDataInfoString;										//Message feedback of data packet
	std::string mDevInfoString;											//Message feedback of dev packet
	bool isSuccessfulFlag;												//Whether the data is successfully obtained

	/*
		 Function: Get data packet status
		 parameter:none
		 return value: std::string：Returns the status information of the data packet 
	*/
	std::string getDataPacketState();


	/*
		 Function: Get dev packet status
		 parameter:none
		 return value: std::string：Returns the status information of the dev packet
	*/
	std::string getDevPacketState();		

	/*
	 Function: Obtain Lidar parameter status
	 parameter: LidarStateParam: Lidar status parameter feedback;			InfoString:Message feedback
	 return value: true, operation succeeded; false: operation failed, View InfoString
	*/
	virtual bool getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString);

	/*
		Function: Initializes the data packet port, device packet port, and destination IP address
		parameter 1: mDataPort	   Data packet port		2368 (by default)
		parameter 2: mDevPort	  Device packet port	2369 (by default)
		parameter 3: mDestIP	 Destination IP 		"192.168.1.102"(by default)
		parameter 3: mGroupIp		Multicast IP	    "224.1.1.102"(by default)
		return calue: none
		
	*/
	void setPortAndIP(uint16_t mDataPort = 2368, uint16_t mDevPort = 2369, std::string mDestIP = "192.168.1.102", std::string mGroupIp = "224.1.1.102");
	

	/*
		Function: Set the radar fixed IP, radar IP, destination IP packet port and device port
		parameter 1:   mFixedLidarIP	Radar fixed IP			"10.10.10.254"   (by default)
		parameter 2:   mLidarIP		    Lidar IP				 "192.168.1.200"  (by default)
		parameter 3:   mDestIP		    Destination IP			 "192.168.1.102"  (by default)
		parameter 4:   mDataPort	    Data packet port	     2368			  (by default)
		parameter 5:   mDevPort		    Device packet port	     2369			  (by default)
		返回值: 无
	**************  Only CX6S3 is currently available  *********/
	virtual void setChangeLidarIP(std::string mFixedLidarIP = "10.10.10.254", std::string mLidarIP = "192.168.1.200", std::string mDestIP = "192.168.1.102", uint16_t mDataPort = 2368, uint16_t mDevPort = 2369);

	/*
		Function: transmit callback function, obtain lidar data
		parameter 1: FunDataPrt*	 introduce callback function pointer, obtain lidar data
		return calue: none
	*/
	void setCallbackFunction(FunDataPrt*);												//set data point callback function
	
	/*
		 Function: star parsing lidar data
		 parameter:none
		 return calue: none
	 */
	void LidarStart();
	
	/*
	 Function: stop parsing lidar data 
	 parameter:none
	 return calue: none
	*/
	void LidarStop();
	
	/*
		 Function: start offline data parsing of network data packet 
		 parameter:none
		 return value: none 
	*/
	void LidarOfflineDataStar();

	/*
	 Function:obtain lidar data of one frame 
	 parameter: preFrameData:  obtain point cloud data of one frame; Info:information
	 return value: true, operation succeeded; false: operation failed
	*/
	bool getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info);
	
	virtual void LidarRun() = 0;
	void CollectionDataArrive(void* pData, uint16_t len);								//transmit UDP data here

	int NegativeToPositive(float);														//Negative numbers are converted to positive numbers
public:

#pragma region //set lidar parameters
	 unsigned char dataDev[1206];														//receive device packet
	 unsigned char Rest_UCWP_buff[1206];												//modify configuration packet
	 std::string ip_sa;																	//send lidar IP
	 int m_SpeedValue = 0;																//set lidar rotate speed
	 bool is_speedFlag = false;															//whether enable rotating speed detection, true: enable
	 bool is_speedNormal = true;														//whether rotating speed is normal, true: normal

	 /*
	 Function: mofify lidar rotating speed
	 parameter: SpeedValue				rotating speed value: support 300， 600，1200;  InfoString:Message feedback
	 return value: true, operation succeeded; false: operation failed
	 */
	virtual bool setLidarRotateSpeed(int SpeedValue, std::string& InfoString);						//set lidar rotating speed
	
	/*
	Function: mofify lidar IP
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.200";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setLidarIP(std::string IPString, std::string& InfoString);							//set lidar IP
	
	/*
	Function: mofify lidar destination IP（computer IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.102";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setComputerIP(std::string IPString, std::string& InfoString);						//set computer IP

		/*
	Function: mofify lidar Gateway IP（Gateway IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "192.168.1.254";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setGatewayIP(std::string IPString, std::string& InfoString);						//set Gateway IP

			/*
	Function: mofify lidar Subnet Mask IP（Subnet Mask IP）
	parameter: IPString	input the IPv4 address to modify, i.e. "255.255.255.0";  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setSubnetMaskIP(std::string IPString, std::string& InfoString);					//set Subnet Mask IP
	
	/*
	Function: modify lidar data pakcet port
	parameter: PortNum, input lidar data port,0~65536 can be input by default（a value larger than 2000 is recommended）;  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setDataPort(int PortNum, std::string& InfoString);									//set data pakcet port
	
	/*
	Function:modify the device packet port
	parameter: PortNum, input lidar data port,0~65536 can be input by default（a value larger than 2000 is recommended）;  InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setDevPort(int PortNum, std::string& InfoString);									//set device packet
	
	/*
	Function: modify the Lidar's motor state
	Parameter: StateValue, input the mode of Lidar's motor, 0 means: running (by default) , 1: stationary;	InfoString:Message feedback
	return value: true, operation succeeded; false: operation failed
	*/
	virtual bool setLidarRotateState(int StateValue, std::string& InfoString);						//set lidar motor state: running or stationary 
	
	/*
	Function:modify Lidar's timing source selection
	Parameter:selectValue, input the source selection, by default, 0 means GPS, 1 means PTP, 2 means NTP;	InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool setLidarSoureSelection(int StateValue, std::string& InfoString);					//set timing source

	/*
	Function:modify Lidar's low power mode, which means the lidar only sends device packet without data packet)
	Parameter:selectValue, input the mode, by default, 0:normal operation mode, 1:low power mode;			 InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool  setLidarWorkState(int StateValue, std::string& InfoString);						//lidar state: in low power mode or not
	
	/*
	Function: send UDP packet, Lidar data
	Parameter: none
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendPackUDP();																		//set UDP packet
	
	/*
	Function:  The second IP control switch
	Parameter: 0 indicates enable and 1 indicates disable
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendSecondIP(int switchIP, std::string& InfoString);								 //The second IP control switch is available only to cx6s3

	/*
	Function:  Cue light control
	Parameter: 1 indicates enable and 0 indicates disable	;										InfoString:Message feedback
	Returned value: true: operation succeeded; false: operation failed
	*/
	virtual bool sendSecondCueLight(int switchIP, std::string& InfoString);							//Available only for CX6S3

	bool setLidarParam();																			//set package
	bool sendPacketToLidar(unsigned char* packet, const char* ip_data, u_short portdata);			//send packet via socket
#pragma endregion 

protected:
	bool isQuit = false;																		//a mark that lidar quits
	std::queue<unsigned char *> allDataValue;													//data cache queue
	std::mutex m_mutex;																			//lock

	u_int64 timestamp_nsce;																		//timestamp shorter than 1 second in the packet(unit: nanosecond)
	u_int64 allTimestamp;																		//save the timestamp in the packet
	u_int64 lastAllTimestamp = 0;																//save the timestamp of the previous packet
	double PointIntervalT_ns;																	//time interval between each point in the packet, unit is nanosecond
	
	int m_messageCount = 0;
	int m_DistanceIsNotZero = 0;	

	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePrt_Get;						// obtain data
	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePrt_Send;						// send data
	
	UTC_Time m_UTC_Time;																		//obtain the GPS information of device packet
	void clearQueue(std::queue<unsigned char *>& m_queue);										//clear queue
	void sendLidarData();																		//packaging and send data
	FunDataPrt *callback = NULL;																//callback function pointer--callback data packet
	std::string time_service_mode = "gps";                                                     //timing method
	
	void messFunction(std::string strValue, int gValue);
	//Check whether the source IP matches or not
	bool checkDefaultIP(std::vector<std::string> m_DefaultIP, std::string& InfoString);
	//Check whether the destination IP matches or not
	bool checkDestIP(std::vector<std::string> m_DestIP, std::string& InfoString);
	
	//start thread to obtain data packet
	void getDataSock();		
	//start thread to obtain device packet
	void getDevSock();    
	
	//start rotating speed judgement after a while
	void sleepTime();
	//start rotating speed judgement 
	void startSleepThread();

private:
	std::shared_ptr<std::vector<MuchLidarData>> LidarPerFrameDatePer;
	bool isSendUDP = true;

	int dataPort = 2368;															//data packet port
	int devPort = 2369;																//device packet port
	std::string computerIP = "192.168.1.102";										//Lidar destination IP (computer IP)
	std::string groupIp = "224.1.1.102";											//Define Multicast IP

#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
	int fd_;
#else
	WORD wVerRequest;
	WSADATA wsaData;
#endif
};

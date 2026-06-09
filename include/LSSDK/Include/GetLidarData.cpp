#include "GetLidarData.h"

GetLidarData::GetLidarData() {

    for (long int FF = 0; FF < 360000; FF++)
    {
        cosAngleValue[FF] = cos(FF / 1000.0 * PI / 180);
        sinAngleValue[FF] = sin(FF / 1000.0 * PI / 180);
    }

	LidarPerFrameDatePrt_Get = std::make_shared<std::vector<MuchLidarData>>();
}

GetLidarData::~GetLidarData() {
}


void GetLidarData::setPortAndIP(uint16_t mDataPort, uint16_t mDevPort, std::string mDestIP, std::string mGroupIp)
{
	dataPort = mDataPort;
	devPort = mDevPort;
	computerIP = mDestIP;
    groupIp = mGroupIp;
}


void GetLidarData::setChangeLidarIP(std::string mFixedLidarIP, std::string mLidarIP, std::string mDestIP, uint16_t mDataPort, uint16_t mDevPort)
{
	std::string str = "This version of Lidar does not support 'setChangeLidarIP()'!!!";
	messFunction(str, 0);
    return;
}

bool GetLidarData::sendSecondIP(int switchIP, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'sendSecondIP()'!!!";
    return false;
}

bool GetLidarData::sendSecondCueLight(int switchIP, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'sendSecondCueLight()'!!!";
    return false;
}

void GetLidarData::setCallbackFunction(FunDataPrt *callbackValue) {
    callback = callbackValue;
}

void GetLidarData::LidarStart() {
    isQuit = false;
    std::thread t1(&GetLidarData::LidarRun, this);
    t1.detach();


    std::thread m_DataSockT(&GetLidarData::getDataSock, this);
    m_DataSockT.detach();

    std::thread m_DevSockT(&GetLidarData::getDevSock, this);
    m_DevSockT.detach();

}

void GetLidarData::LidarOfflineDataStar()
{
	isQuit = false;
	std::thread t1(&GetLidarData::LidarRun, this);
	t1.detach();
}
//obtain data packet port number
void GetLidarData::getDataSock()
{
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
    //create socket
    int m_sock = socket(2, 2, 0);			//create sock
#else
	WORD wVerRequest = MAKEWORD(1, 1);
	WSADATA wsaData;
	WSAStartup(wVerRequest, &wsaData);
		//create socket
    SOCKET m_sock = socket(2, 2, 0);			//create sock
#endif


	//initialize UDP communication 
	//create socket
	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	//define address
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(dataPort);
	//sockAddr.sin_addr.s_addr = inet_addr(computerIP.c_str());
	inet_pton(AF_INET, computerIP.c_str(), &sockAddr.sin_addr);
	
	int value = 10 * 1024 * 1024;
	int tmpCode = 0;
	tmpCode = ::setsockopt(m_sock, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));
	
	ip_mreq multiCast;

#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
    multiCast.imr_interface.s_addr = INADDR_ANY;
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.s_addr);

    //设置超时时间  2秒
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
#else

    multiCast.imr_interface.S_un.S_addr = INADDR_ANY;		         //The IP address of a local network device interface。
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.S_un.S_addr);

    //设置超时时间  2秒
    struct timeval tv;
    tv.tv_sec = 2000;
    tv.tv_usec = 0;
#endif


    setsockopt(m_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&multiCast, sizeof(multiCast));
    
    //bind socket
    int retVal = ::bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));

    if (-1 == retVal)
    {
        std::string str = "Bind mDataPort failed!!!\n";
        messFunction(str, 10001);
    }

    if (setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0)
    {
        std::string str = "mDataPort Set SO_RCVTIMEO timeout failed !!!\n";
        messFunction(str, 10002);
    }

    struct sockaddr_in addrFrom;

#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
	socklen_t len = sizeof(sockaddr_in);
#else
	int len = sizeof(sockaddr_in);
#endif
	//receive data
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (true)
	{
        if (isQuit) {
            std::string str = "Exit to obtain network data packet!!!\n";
            messFunction(str, 10009);
            break;
        }

		//Bind the socket to receive content
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
		recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
#else
		recvLen = ::recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (SOCKADDR*)&addrFrom, &len);
		
		
#endif
        if (-1 == recvLen)
        {
            std::string str = "The mDataPort failed to obtain data!!!\n";
            messFunction(str, 10003);
        }
        else if(0 == recvLen)
        {
            std::string str = "mDataPort recvfrom timeout !!!\n";
            messFunction(str, 10004);
        }
		else if (recvLen > 0)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			CollectionDataArrive(data, recvLen);												//transmit data to class
		}
		

	}
}

//Obtain the port number of the device packet
void GetLidarData::getDevSock()
{
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
    //create socket
    int m_sock = socket(2, 2, 0);			//create sock
#else
	WORD wVerRequest = MAKEWORD(1, 1);
	WSADATA wsaData;
	WSAStartup(wVerRequest, &wsaData);
		//create socket
    SOCKET m_sock = socket(2, 2, 0);			//create sock
#endif

	//initialize UDP communication //
	//create socket
	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	//define address
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(devPort);											//obtain the port number of the device package
	//sockAddr.sin_addr.s_addr = inet_addr(computerIP.c_str());
	inet_pton(AF_INET, computerIP.c_str(), &sockAddr.sin_addr);
	
	//int iMode = 1;
	//ioctlsocket(m_sock, FIONBIO, (u_long FAR*)&iMode);   //Non-blocking Setting
	
	ip_mreq multiCast;
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
    multiCast.imr_interface.s_addr = INADDR_ANY;		         //IP address of a local network device interface。
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.s_addr);

    //设置超时时间  2秒
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
#else
    multiCast.imr_interface.S_un.S_addr = INADDR_ANY;		         //IP address of a local network device interface
    inet_pton(AF_INET, groupIp.data(), &multiCast.imr_multiaddr.S_un.S_addr);

    //设置超时时间  2秒
    struct timeval tv;
    tv.tv_sec = 2000;
    tv.tv_usec = 0;
#endif

    setsockopt(m_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&multiCast, sizeof(multiCast));
    
    //Bind the socket
    int retVal = ::bind(m_sock, (struct sockaddr *)&sockAddr, sizeof(sockAddr));

    if (-1 == retVal)
    {
        std::string str = "Bind mDevPort failed!!!\n";
        messFunction(str, 10011);
    }

    if (setsockopt(m_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv)) < 0)
    {
        std::string str = "mDevPort Set SO_RCVTIMEO timeout failed !!!\n";
        messFunction(str, 10012);
    }

    struct sockaddr_in addrFrom;
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
	socklen_t len = sizeof(sockaddr_in);
#else
	int len = sizeof(sockaddr_in);
#endif
	//receive data
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (true)
	{
        if (isQuit) {
            std::string str = "Exit to obtain network data!!!\n";
            messFunction(str, 10019);
            break;
        }

		//Bind the socket to receive content
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
		recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
#else
		recvLen = ::recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (SOCKADDR*)&addrFrom, &len);
#endif
		
        if (-1 == recvLen)
        {
            std::string str = "The mDevPort failed to obtain data!!!\n";
            messFunction(str, 10013);
        }
        else if (0 == recvLen)
        {
            std::string str = "mDevPort recvfrom timeout !!!\n";
            messFunction(str, 10014);
        }
        else if (recvLen > 0)
        {
            u_char data[1212] = { 0 };
            memcpy(data, recvBuf, recvLen);
            CollectionDataArrive(data, recvLen);												//transmit data to class
        }


	}
}

std::string GetLidarData::getDataPacketState()
{
    m_mutex.lock();
    std::string mDataInfoStringT = mDataInfoString;
    m_mutex.unlock();

    return mDataInfoStringT;
}

std::string GetLidarData::getDevPacketState()
{
    m_mutex.lock();
    std::string mDevInfoStringT = mDataInfoString;
    m_mutex.unlock();
 
    return mDevInfoStringT;
}

bool GetLidarData::getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString)
{
    if (true == islidarDevCome)
    {
        m_mutex.lock();
        //Save the device packet
        unsigned char pktdata[1206];												//modify configuration packet
        memcpy(pktdata, dataDev, 1206);
        m_mutex.unlock();

        //Lidar IP
        std::string ip_value = 
            std::to_string(pktdata[10]) + "." +
            std::to_string(pktdata[11]) + "." +
            std::to_string(pktdata[12]) + "." +
            std::to_string(pktdata[13]);
        mLidarStateParam.LidarIP = ip_value;

        //Lidar destination IP（computer IP）
        ip_value =
            std::to_string(pktdata[14]) + "." +
            std::to_string(pktdata[15]) + "." +
            std::to_string(pktdata[16]) + "." +
            std::to_string(pktdata[17]);
        mLidarStateParam.ComputerIP = ip_value;

        //Lidar Gateway IP（Gateway IP）
        ip_value =
            std::to_string(pktdata[32]) + "." +
            std::to_string(pktdata[33]) + "." +
            std::to_string(pktdata[34]) + "." +
            std::to_string(pktdata[35]);
        mLidarStateParam.GatewayIP = ip_value;

        //Lidar Subnet Mask IP（Subnet Mask IP）
        ip_value =
            std::to_string(pktdata[36]) + "." +
            std::to_string(pktdata[37]) + "." +
            std::to_string(pktdata[38]) + "." +
            std::to_string(pktdata[39]);
        mLidarStateParam.SubnetMaskIP = ip_value;

        mLidarStateParam.DataPort = pktdata[24] * 256 + pktdata[25];
        mLidarStateParam.DevPort  = pktdata[26] * 256 + pktdata[27];

        return true;
    }
    else
    {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

void GetLidarData::LidarStop() {
    isQuit = true;
}


void GetLidarData::sendLidarData() {

    if (m_DistanceIsNotZero < 20)
    {
        messFunction("Data error!!! All lidar distanceValue are 0!", 10032);
    }
    else
    {
        m_mutex.lock();
        isSuccessfulFlag = true;
        mDataInfoString = "Obtaining data successfully!";
        m_mutex.unlock();
    }
    m_DistanceIsNotZero = 0;

	if (callback) {
		LidarPerFrameDatePrt_Send = LidarPerFrameDatePrt_Get;
		(*callback)(LidarPerFrameDatePrt_Send, isSuccessfulFlag, mDataInfoString);
	}
    m_mutex.lock();
	LidarPerFrameDatePer = LidarPerFrameDatePrt_Get;
	LidarPerFrameDatePrt_Get.reset(new std::vector<MuchLidarData>); 
	isFrameOK = true;
    m_mutex.unlock();
}

void GetLidarData::CollectionDataArrive(void *pData, uint16_t len) {
    if (len >= 1206) {
        unsigned char *dataV = new unsigned char[1212];
        memset(dataV, 0, 1212);
        memcpy(dataV, pData, len);
        m_mutex.lock();
        allDataValue.push(dataV);
        m_mutex.unlock();

        if ((dataV[0] == 0x00 || dataV[0] == 0xa5) && dataV[1] == 0xff && dataV[2] == 0x00 && dataV[3] == 0x5a) {

            m_mutex.lock();
            memcpy(dataDev, pData, 1206);
            ip_sa = std::to_string(dataDev[10]) + "." +
                    std::to_string(dataDev[11]) + "." +
                    std::to_string(dataDev[12]) + "." +
                    std::to_string(dataDev[13]);
    
            islidarDevCome = true;
            m_mutex.unlock();
        }
    }
}

int  GetLidarData::NegativeToPositive(float value)
{
    int valueT = value * 1000;
    if (valueT >= 0)
    {
        return (valueT > 360000 ? valueT % 360000 : valueT);
    }
    else
    {
        return (valueT < -360000 ? (valueT % -360000) + 360000 : valueT + 360000);
    }
}

void GetLidarData::clearQueue(std::queue<unsigned char *> &m_queue) {
    std::queue<unsigned char *> empty;
    swap(empty, m_queue);
}

bool GetLidarData::getLidarPerFrameDate(std::shared_ptr<std::vector<MuchLidarData>>& preFrameData, std::string& Info)
{
    m_mutex.lock();
    isFrameOK = false;
    preFrameData = std::move(LidarPerFrameDatePer);
    Info = mDataInfoString;
    m_mutex.unlock();
    return isSuccessfulFlag;

}

#pragma region //Set radar parameters to send device packet 

//set the rotate speed
bool GetLidarData::setLidarRotateSpeed(int SpeedValue, std::string& InfoString) {

	m_SpeedValue = SpeedValue;
	
	if (setLidarParam()) {
	    //set the rotate speed
		Rest_UCWP_buff[8] = SpeedValue / 256;	
		Rest_UCWP_buff[9] = SpeedValue % 256;
	} else {
        InfoString = "Equipment package is not update!!!";
	    return false;
	}
	startSleepThread();
	return true;
}
//Turn on the speed judgment after a period of time
void GetLidarData::sleepTime()
{
	is_speedFlag = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	is_speedFlag = true;
}

//Start the speed judgement thread
void GetLidarData::startSleepThread()
{
	std::thread t1_Start(&GetLidarData::sleepTime, this);
	t1_Start.detach();
}

//set lidar IP
bool GetLidarData::setLidarIP(std::string LidarIPValue, std::string& InfoString) {
    std::regex ipv4(
            "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
    if (!regex_match(LidarIPValue, ipv4)) {
        InfoString = "The IP format entered is incorrect, please check the input parameters";
        return false;
    }
    if (setLidarParam()) {
        //set lidar IP
        std::string::size_type defailtIP_pos;
        std::vector<std::string> IP_result;
        IP_result.clear();
        LidarIPValue = LidarIPValue + ".";                                                     //Easily obtain the last piece of data
        for (size_t i = 0; i < LidarIPValue.size(); i++)                                     //cut out defaultIP lineedit
        {
            defailtIP_pos = LidarIPValue.find(".", i);
            if (defailtIP_pos < LidarIPValue.size()) {
                std::string s = LidarIPValue.substr(i, defailtIP_pos - i);
                IP_result.emplace_back(std::move(s));
                i = defailtIP_pos;// +pattern.size() - 1;
            }
        }

        if (IP_result.size() < 4) {
            InfoString = "Please enter the full Lidar IP address!!!Failed to set the Lidar IP address!!!";
            return false;
        } else if (IP_result.size() == 4 && IP_result[3] == "") {
            InfoString =  "Please enter the full Lidar IP address!!!Failed to set the Lidar IP address!!!";
            return false;
        }
    
        if (!checkDefaultIP(IP_result, InfoString)) {
            InfoString =  "Failed to set the Lidar IP address!!!";
            return false;
        }
    
        Rest_UCWP_buff[10] = atoi(IP_result[0].c_str());
        Rest_UCWP_buff[11] = atoi(IP_result[1].c_str());
        Rest_UCWP_buff[12] = atoi(IP_result[2].c_str());
        Rest_UCWP_buff[13] = atoi(IP_result[3].c_str());
        InfoString = "Successfully set!";
        return true;
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

//Check whether the lidar IP settings are in compliance with the requireement
bool GetLidarData::checkDefaultIP(std::vector<std::string> m_DefaultIP, std::string& InfoString) {
    int HeadDefaultIPValue = stoi(m_DefaultIP[0]);
    int endDefaultIPValue = stoi(m_DefaultIP[3]);
    if (HeadDefaultIPValue == 0 || HeadDefaultIPValue == 127 ||
        (HeadDefaultIPValue >= 224 && HeadDefaultIPValue <= 255)
            ) {
        std::string str =
                "The Lidar IP cannot be set to " + std::to_string(HeadDefaultIPValue) + std::string(".x.x.x!!!");
        InfoString = str;
        messFunction(str, 0);
        return false;
    } else if (endDefaultIPValue == 255) {
        std::string str = "The Lidar IP cannot be set to broadcast(x.x.x.255)!!!";
        InfoString = str;
        messFunction(str, 0);
        return false;
    }
    return true;
}

//Check whether the destination IP settings are in compliance with the requireement
bool GetLidarData::checkDestIP(std::vector<std::string> m_DestIP, std::string& InfoString) {
    int HeadDefaultIPValue = stoi(m_DestIP[0]);
    int endDefaultIPValue = stoi(m_DestIP[3]);
    if (HeadDefaultIPValue == 0 || HeadDefaultIPValue == 127 ||
        (HeadDefaultIPValue >= 240 && HeadDefaultIPValue <= 255)
            ) {
        std::string str =
                "The Dest IP cannot be set to " + std::to_string(HeadDefaultIPValue) + std::string(".x.x.x!!!");
        InfoString = str;
        messFunction(str, 0);
        return false;
    }
    return true;
}

//set computer IP
bool GetLidarData::setComputerIP(std::string ComputerIPValue, std::string& InfoString) {
    std::regex ipv4(
            "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
    if (!regex_match(ComputerIPValue, ipv4)) {
        InfoString = "The IP format entered is incorrect, please check the input parameters";
        return false;
    }

    if (setLidarParam()) {
        //set computer IP
        std::vector<std::string> IP_result;
        IP_result.clear();
        std::string::size_type DestIP_pos;
        ComputerIPValue = ComputerIPValue + ".";
        for (size_t i = 0; i < ComputerIPValue.size(); i++)                                     //cut out of destIP lineedit 
        {
            DestIP_pos = ComputerIPValue.find(".", i);
            if (DestIP_pos < ComputerIPValue.size()) {
                std::string s = ComputerIPValue.substr(i, DestIP_pos - i);
                IP_result.emplace_back(std::move(s));
                i = DestIP_pos;
            }
        }
    
        if (IP_result.size() < 4) {
            InfoString = "Please enter the full Dest IP address!!! Failed to set the Dest(Computer) IP address!!!";
            return false;
    
        } else if (IP_result.size() == 4 && IP_result[3] == "") {
            InfoString = "Please enter the full Dest IP address!!!Failed to set the Dest(Computer) IP address!!!";
            return false;
        }
        if (!checkDestIP(IP_result, InfoString)) {
            InfoString = "Failed to set the computer IP address!!!";
            return false;
        }
        Rest_UCWP_buff[14] = atoi(IP_result[0].c_str());
        Rest_UCWP_buff[15] = atoi(IP_result[1].c_str());
        Rest_UCWP_buff[16] = atoi(IP_result[2].c_str());
        Rest_UCWP_buff[17] = atoi(IP_result[3].c_str());
    
        return true;
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

bool GetLidarData::setGatewayIP(std::string IPString, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setGatewayIP()'!!!";
    return false;
}

bool GetLidarData::setSubnetMaskIP(std::string IPString, std::string& InfoString)
{
    InfoString = "This version of Lidar does not support 'setSubnetMaskIP()'!!!";
    return false;
}

//set data packet port
bool GetLidarData::setDataPort(int DataPort, std::string& InfoString) {
    if (setLidarParam()) {
        //check port
        int devPort = Rest_UCWP_buff[26] * 256 + Rest_UCWP_buff[27];
        if (DataPort < 1025 || DataPort > 65535 || DataPort == devPort) {
            InfoString = "DataPort range 1025-65535 and DataPort and devport cannot be equal, please check the input parameters";
            return false;
        } else {
            //set data packet port
            Rest_UCWP_buff[24] = DataPort / 256;
            Rest_UCWP_buff[25] = DataPort % 256;
            return true;
        }

    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

//set device packet port
bool GetLidarData::setDevPort(int DevPort, std::string& InfoString) {
    if (setLidarParam()) {
        //check port
        int dataPort = Rest_UCWP_buff[24] * 256 + Rest_UCWP_buff[25];
        if (DevPort < 1025 || DevPort > 65535 || DevPort == dataPort) {
            InfoString = "DataPort range 1025-65535 and DataPort and devport cannot be equal, please check the input parameters";
            return false;
        } else {
            //set device packet port
            Rest_UCWP_buff[26] = DevPort / 256;
            Rest_UCWP_buff[27] = DevPort % 256;
            return true;
        }
    } else {
        InfoString = "Equipment package is not update!!!";
        return false;
    }
}

bool GetLidarData::setLidarRotateState(int RotateState, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarRotateState()'!!!";
	return false;
}

bool GetLidarData::setLidarSoureSelection(int StateValue, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarSoureSelection()'!!!";
    return false;
}

bool GetLidarData::setLidarWorkState(int LidarState, std::string& InfoString) {
    InfoString = "This version of Lidar does not support 'setLidarWorkState()'!!!";
    return false;
}

bool GetLidarData::setLidarParam() {
    if (isSendUDP == false) {
        return true;
    }

    if (true == islidarDevCome && true == isSendUDP) {
        islidarDevCome = false;
        isSendUDP = false;
        m_mutex.lock();
        //Save the device packet before send the configuration packet
        memcpy(Rest_UCWP_buff, dataDev, 1206);
        m_mutex.unlock();

        for (int i = 52; i < 60; i++) {
            Rest_UCWP_buff[i] = 0x00;
        }
    
        for (int i = 160; i < 168; i++) {
            Rest_UCWP_buff[i] = 0x00;
        }
    
        Rest_UCWP_buff[0] = 0xAA;                           //merge UCWP with ACWP, the UCWP identification header is the first 8 bytes
        Rest_UCWP_buff[1] = 0x00;
        Rest_UCWP_buff[2] = 0xFF;
        Rest_UCWP_buff[3] = 0x11;
        Rest_UCWP_buff[4] = 0x22;
        Rest_UCWP_buff[5] = 0x22;
        Rest_UCWP_buff[6] = 0xAA;
        Rest_UCWP_buff[7] = 0xAA;
        return true;
    } else {
        std::string str = "Equipment package is not update!!!";
        messFunction(str, 0);
        return false;
    }

}

bool GetLidarData::sendPacketToLidar(unsigned char *packet, const char *ip_data, u_short portNum) {
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
    struct sockaddr_in addrSrv{};
    //create socket UDP
    int socketid = socket(2, 2, 0);
#define SOCKET_ERROR -1
#else
    //initialize socket 
    WORD  request;
    WSADATA  ws;
    request = MAKEWORD(1, 1);
    int err = WSAStartup(request, &ws);
    if (err != 0)
    {
        return false;
    }
    if (LOBYTE(ws.wVersion) != 1 || HIBYTE(ws.wVersion) != 1)
    {
        WSACleanup();
        return false;
    }
    SOCKADDR_IN addrSrv;
    //create socket UDP
    SOCKET socketid = socket(2, 2, 0);
#endif

    addrSrv.sin_family = AF_INET;
    addrSrv.sin_port = htons(portNum);
    inet_pton(AF_INET, ip_data, &addrSrv.sin_addr);
    
    int sd = sendto(socketid, (const char *) packet, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
    
    if (sd != SOCKET_ERROR) {
        printf("send successfully,send:%dchars\n", sd);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
#if defined(__linux__) || defined(__aarch64__) || defined(__x86_64__)
        (void) ::close(socketid);
#else
        closesocket(socketid);
#endif
    } else {
        printf("Failure to send\n");
        return false;
    }

    isSendUDP = true;
    return true;
}

bool GetLidarData::sendPackUDP() {
    return sendPacketToLidar(Rest_UCWP_buff, ip_sa.c_str(), 2368);
}

void GetLidarData::messFunction(std::string strValue, int gValue) {
    std::cout << "Code = " << gValue << " : " << strValue.c_str() << std::endl;

    m_mutex.lock();
    if ((10000 <= gValue  &&  gValue <= 10009) || (10030 <= gValue && gValue <= 10039))
    {
        isSuccessfulFlag = false;
        mDataInfoString = strValue;   
    }

    if ((10010 <= gValue && gValue <= 10019))
    {
        mDevInfoString = strValue;
    }
    m_mutex.unlock();

}

#pragma endregion

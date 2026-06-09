#include "GetLidarData_LS.h"
#define DEG2RAD(x) ((x)*0.017453293)

GetLidarData_LS::GetLidarData_LS() {

	m_DeadZoneOffset = 6.37f;
	m_fPutTheMirrorOffAngle[0] = 1.5f;
	m_fPutTheMirrorOffAngle[1] = -0.5f;
	m_fPutTheMirrorOffAngle[2] = 0.5f;
	m_fPutTheMirrorOffAngle[3] = -1.5f;
	
	m_fDistanceAcc = 0.001f;
	m_fH_AngleAcc = 0.01f;
	m_fV_AngleAcc = 0.0025f;
	
	count = 0;
}

GetLidarData_LS::~GetLidarData_LS() {
}

void GetLidarData_LS::LidarRun() {
    while (true) {
        if (isQuit) {
            clearQueue(allDataValue);
            break;
        }
        if (!allDataValue.empty()) {
            unsigned char data[1206] = {0};
            m_mutex.lock();
            memcpy(data, allDataValue.front(), 1206);
            delete allDataValue.front();
            allDataValue.pop();
            m_mutex.unlock();

            //obtain the header which is a5 ff 00 5a in a device packet, obtain GPS time and offset value of the optical prism
            if ((data[0] == 0x00 || data[0] == 0xa5) && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
    		{
    			m_HorizontalPoints = (data[184] << 8) + data[185];
    
    			if (64 == data[231] || 65 == data[231])
    			{
    				m_StackFrame = 2;
    			}
    			else if (192 == data[231])
    			{
    				m_StackFrame = 1;
    				m_fV_AngleAcc = 0.01;
    				m_fDistanceAcc = 0.004f;
    				m_fPutTheMirrorOffAngle[0] = 1.5;
    				m_fPutTheMirrorOffAngle[1] = 0.5;
    				m_fPutTheMirrorOffAngle[2] = -0.5;
    				m_fPutTheMirrorOffAngle[3] = -1.5;
    				m_DeadZoneOffset = 10.82;
    			}
    			else
    			{
    				m_fDistanceAcc = 0.001f;
    				m_fH_AngleAcc = 0.01f;
    				m_fV_AngleAcc = 0.0025f;
    				m_StackFrame = 1;
    
    				m_DeadZoneOffset = 6.37;
    				m_fPutTheMirrorOffAngle[0] = 1.5;
    				m_fPutTheMirrorOffAngle[1] = -0.5;
    				m_fPutTheMirrorOffAngle[2] = 0.5;
    				m_fPutTheMirrorOffAngle[3] = -1.5;
    			}
    
    			int majorVersion = data[1202];
    			int minorVersion1 = data[1203] / 16;
    			int minorVersion2 = data[1203] % 16;
    
    			//v1.1 :0.01   //after v1.2  ： 0.0025
    			if (majorVersion > 1 || (1 == majorVersion && minorVersion1 > 1)) {
    				m_fV_AngleAcc = 0.0025;
    			}
    			else {
    				m_fV_AngleAcc = 0.01;
    			}
    
                continue;
            }
    
			uint64_t timestamp_nsce =
				(static_cast<uint64_t>(data[1200]) << 24) +
				(static_cast<uint64_t>(data[1201]) << 16) +
				(static_cast<uint64_t>(data[1202]) << 8) +
				static_cast<uint64_t>(data[1203]);

    		if (0xff == data[1194])
    		{    //ptp timing
    			//std::cout << "ptp";
				uint64_t timestamp_s =
					(static_cast<uint64_t>(data[1195]) << 32) +
					(static_cast<uint64_t>(data[1196]) << 24) +
					(static_cast<uint64_t>(data[1197]) << 16) +
					(static_cast<uint64_t>(data[1198]) << 8) +
					static_cast<uint64_t>(data[1199]);

    			allTimestamp = timestamp_s * 1000000000 + timestamp_nsce;
    		}
    		else
    		{     //gps timing
    			//std::cout << "gps";
    			m_UTC_Time.year		= data[1194] + 2000;
    			m_UTC_Time.month	= data[1195];
    			m_UTC_Time.day		= data[1196];
    			m_UTC_Time.hour		= data[1197];
    			m_UTC_Time.minute	= data[1198];
    			m_UTC_Time.second	= data[1199];
    
    			struct tm t{};
    			t.tm_sec = m_UTC_Time.second;
    			t.tm_min = m_UTC_Time.minute;
    			t.tm_hour = m_UTC_Time.hour;
    			t.tm_mday = m_UTC_Time.day;
    			t.tm_mon = m_UTC_Time.month - 1;
    			t.tm_year = m_UTC_Time.year - 1900;
    			t.tm_isdst = 0;
    			//auto timestamp_s = mktime(&t);
    			auto timestamp_s = static_cast<uint64_t>(timegm(&t)); //s
    			if (-1 == timestamp_s) {
    				perror("parse error");
    			}
    			allTimestamp = timestamp_s * 1000000000 + timestamp_nsce;
    		}
    
    		if (192 == data[1204] && m_HorizontalPoints != 0)
    		{
    			m_StackFrame = 1;
    			m_fV_AngleAcc = 0.01f;
    			m_fDistanceAcc = 0.004f;
    			m_fPutTheMirrorOffAngle[0] = 1.5f;
    			m_fPutTheMirrorOffAngle[1] = 0.5f;
    			m_fPutTheMirrorOffAngle[2] = -0.5f;
    			m_fPutTheMirrorOffAngle[3] = -1.5f;
    			m_DeadZoneOffset = 10.82f;
    		}
    			
            int lidar_EchoModel = data[1205];
    
            //Double Echo
            if (2 == lidar_EchoModel) {
                handleDoubleEcho(data);
            }
                //Single Echo
            else {
                handleSingleEcho(data);
            }
            lastAllTimestamp = allTimestamp;
#pragma endregion
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}



//echo mode is up to the lidar model.
//Obtain single echo data
void GetLidarData_LS::handleSingleEcho(unsigned char *data) {

#pragma region//single echo
	PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 149;
	for (int i = 0; i < 1192; i = i + 8)
	{
		if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc && data[i + 4] == 0xdd)
		{
			count++;
		}
		if (count == 1)
		{
			if (m_StackFrame > 1)
			{
				tempPointCloud = *LidarPerFrameDatePrt_Get;

				LidarPerFrameDatePrt_Get->insert(LidarPerFrameDatePrt_Get->begin(), PointCloudLastData.begin(), PointCloudLastData.end());
				PointCloudLastData = std::move(tempPointCloud);
			}
	
			sendLidarData();
			count = 0;
			m_messageCount = 0;
			continue;
		}
		if (LidarPerFrameDatePrt_Get->size() > 3000000)
		{
			if (1 == m_messageCount)
			{
				continue;
			}

			std::string str = "Frame synchronization failure!!!";
			messFunction(str, 10031);
			m_messageCount = 1;
			continue;
		}
		if (count == 0)
		{
			float tDistance = (data[i + 4] << 16) + (data[i + 5] << 8)  + data[i + 6];
			m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : ((abs(tDistance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero);
			if (tDistance <= 0)
				continue;

			//horizontal angle
			float tfAngle_H = (data[i] << 8) + data[i + 1];
			if (tfAngle_H > 32767) {
				tfAngle_H = (tfAngle_H - 65536);
			}
			tfAngle_H = tfAngle_H * m_fH_AngleAcc;


			//vertical angle+channel No.
			int tTempAngle = data[i + 2];
			int tChannelID = tTempAngle >> 6;			//left shift for 6 bits, channel No.
			int tSymmbol = (tTempAngle >> 5) & 0x01;	//eft shift for 5 bits,sign bit
			float fAngle_V = 0.0;
			if (1 == tSymmbol)							// ign bit 0：positive number 1：negative number
			{
				int iAngle_V = (data[i + 2] << 8) + data[i + 3];
				fAngle_V = iAngle_V | 0xc000;					//high order of negative number are all "1"
				if (fAngle_V > 32767)
				{
					fAngle_V = (fAngle_V - 65536);				//convert to negative number
				}
			}
			else
			{
				int iAngle_Hight = tTempAngle & 0x3f;
				fAngle_V = (iAngle_Hight << 8) + data[i + 3];
			}
			fAngle_V *= m_fV_AngleAcc;
	
			tDistance *= m_fDistanceAcc;
			int intensity = data[i + 7];


			m_PointXYZ m_point_1 = XYZ_calculate(tChannelID, tfAngle_H, fAngle_V, tDistance, -1);
	
			MuchLidarData m_DataT;
			m_DataT.X = m_point_1.x1;
			m_DataT.Y = m_point_1.y1;
			m_DataT.Z = m_point_1.z1;
			m_DataT.ID = tChannelID;
			m_DataT.H_angle = tfAngle_H;
			m_DataT.V_angle = fAngle_V;
			m_DataT.Distance = tDistance;
			m_DataT.Intensity = intensity;
			m_DataT.Mtimestamp_nsce = (allTimestamp - PointIntervalT_ns * (148 - i / 8));
			LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT));
		}
	}

#pragma endregion

}

//Obtain double echo data
void GetLidarData_LS::handleDoubleEcho(unsigned char *data) {
#pragma region//double echo 
	PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 99;
	for (int i = 0; i < 1188; i = i + 12)
	{
		if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc && data[i + 4] == 0xdd)
		{
			count++;
		}
		if (count == 1)
		{
			if (m_StackFrame > 1)
			{
				tempPointCloud = *LidarPerFrameDatePrt_Get;

				LidarPerFrameDatePrt_Get->insert(LidarPerFrameDatePrt_Get->begin(), PointCloudLastData.begin(), PointCloudLastData.end());
				PointCloudLastData = std::move(tempPointCloud);
			}
	
			sendLidarData();
			count = 0;
			m_messageCount = 0;
			continue;
		}
		if (LidarPerFrameDatePrt_Get->size() > 3000000)
		{
			if (1 == m_messageCount)
			{
				continue;
			}

			std::string str = "Frame synchronization failure!!!";
			messFunction(str, 10031);
			m_messageCount = 1;
			continue;
		}
		if (count == 0)
		{
			float tDistance = (data[i + 4] << 16) + (data[i + 5] << 8) + data[i + 6];
			m_DistanceIsNotZero = m_DistanceIsNotZero > 30 ? m_DistanceIsNotZero : (abs(tDistance - 0) > 0.00000001) ? ++m_DistanceIsNotZero : m_DistanceIsNotZero;
			if (tDistance <= 0)
				continue;
			//horizontal angle
			float tfAngle_H = (data[i] << 8) + data[i + 1];
			if (tfAngle_H > 32767) {
				tfAngle_H = (tfAngle_H - 65536);
			}
			tfAngle_H = tfAngle_H * m_fH_AngleAcc;
	
			//vertical angle+channel No.
			int tTempAngle = data[i + 2];
			int tChannelID = tTempAngle >> 6; //left shift for 6 bits, channel No.
			int tSymmbol = (tTempAngle >> 5) & 0x01; //left shift for 5 bits,sign bit
			float tfAngle_V = 0.0;
			if (1 == tSymmbol) // sign bit 0：positive number 1：negative number
			{
				int iAngle_V = (data[i + 2]<< 8) + data[i + 3];
				tfAngle_V = iAngle_V | 0xc000;					//high order of negative number are all "1"
				if (tfAngle_V > 32767)
				{
					tfAngle_V = (tfAngle_V - 65536);				//convert to negative number
				}
			}
			else
			{
				int iAngle_Hight = tTempAngle & 0x3f;
				tfAngle_V = (iAngle_Hight << 8) + data[i + 3];
			}
			tfAngle_V *= m_fV_AngleAcc;
	
			//1st echo data
			tDistance *= m_fDistanceAcc;
			int intensity = data[i + 7];
	
			//2nd echo data
			float tDistance_2 = ((data[i + 8] << 16) + (data[i + 9] << 8) + data[i + 10]) * m_fDistanceAcc;
			int intensity_2 = data[i + 11];


			m_PointXYZ m_point = XYZ_calculate(tChannelID, tfAngle_H, tfAngle_V, tDistance, tDistance_2);
			MuchLidarData m_DataT;
			m_DataT.X = m_point.x1;
			m_DataT.Y = m_point.y1;
			m_DataT.Z = m_point.z1;
			m_DataT.ID = tChannelID;
			m_DataT.H_angle = tfAngle_H;
			m_DataT.V_angle = tfAngle_V;
			m_DataT.Distance = tDistance;
			m_DataT.Intensity = intensity;
			m_DataT.Mtimestamp_nsce = (allTimestamp - PointIntervalT_ns * (98 - i / 12));
			LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT));


			MuchLidarData m_DataT2;
			m_DataT2.X = m_point.x2;
			m_DataT2.Y = m_point.y2;
			m_DataT2.Z = m_point.z2;
			m_DataT2.ID = tChannelID;
			m_DataT2.H_angle = tfAngle_H;
			m_DataT2.V_angle = tfAngle_V;
			m_DataT2.Distance = tDistance_2;
			m_DataT2.Intensity = intensity_2;
			m_DataT2.Mtimestamp_nsce = (allTimestamp - PointIntervalT_ns * (98 - i / 12));
			LidarPerFrameDatePrt_Get->emplace_back(std::move(m_DataT2));
		}
	}

#pragma endregion
}

//Three-dimensional coordinate conversion
m_PointXYZ GetLidarData_LS::XYZ_calculate(int tChannelID, float& fAngle_H, float& fAngle_V, float tDistance, float tDistance2)
{
	m_PointXYZ point;
	//distortion formula
	//mirror angle
	float fPutTheMirrorOffAngle = m_fPutTheMirrorOffAngle[tChannelID];

	//Galvanometer offset angle = actual vertical angle / 2  - offset value
	double fGalvanometrtAngle = 0;
	//fGalvanometrtAngle = (((fAngle_V + 0.05) / 0.8) + 1) * 0.46 + 6.72;
	fGalvanometrtAngle = fAngle_V + m_DeadZoneOffset;     //note: 7.26 is vertical angle offset rotation)
	
	double fAngle_R0 = cosAngleValue[NegativeToPositive(30)] * cosAngleValue[NegativeToPositive(fPutTheMirrorOffAngle)] * cosAngleValue[NegativeToPositive(fGalvanometrtAngle)] -
		sinAngleValue[NegativeToPositive(fGalvanometrtAngle)] * sinAngleValue[NegativeToPositive(fPutTheMirrorOffAngle)];
	
	double fSinV_angle = 2 * fAngle_R0 * sinAngleValue[NegativeToPositive(fGalvanometrtAngle)] + sinAngleValue[NegativeToPositive(fPutTheMirrorOffAngle)];
	double fCosV_angle = sqrt(1 - fSinV_angle * fSinV_angle);
	
	double fSinCite = (2 * fAngle_R0 * cosAngleValue[NegativeToPositive(fGalvanometrtAngle)] * sinAngleValue[NegativeToPositive(30)] -
		cosAngleValue[NegativeToPositive(fPutTheMirrorOffAngle)] * sinAngleValue[NegativeToPositive(60)]) / fCosV_angle;
	double fCosCite = sqrt(1 - fSinCite * fSinCite);
	
	double fSinCite_H = sinAngleValue[NegativeToPositive(fAngle_H)] * fCosCite + cosAngleValue[NegativeToPositive(fAngle_H)] * fSinCite;
	double fCosCite_H = cosAngleValue[NegativeToPositive(fAngle_H)] * fCosCite - sinAngleValue[NegativeToPositive(fAngle_H)] * fSinCite;
	
	fAngle_H = (asin(fSinCite_H) * 180 / PI);
	fAngle_V = (asin(fSinV_angle) * 180 / PI);
	// todo 
	point.x1 = float(tDistance * fCosV_angle * fSinCite_H);
	point.y1 = float(tDistance * fCosV_angle * fCosCite_H);
	point.z1 = float(tDistance * fSinV_angle);
	
	if (tDistance2 > 0)
	{
		point.x2 = float(tDistance2 * fCosV_angle * fSinCite_H);
		point.y2 = float(tDistance2 * fCosV_angle * fCosCite_H);
		point.z2 = float(tDistance2 * fSinV_angle);
	}
	
	return point;
}


bool GetLidarData_LS::getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString)
{
	if (true == islidarDevCome)
	{
		GetLidarData::getLidarParamState(mLidarStateParam, InfoString);
		m_mutex.lock();
		//Save the device packet
		unsigned char pktdata[1206];												//modify configuration packet
		memcpy(pktdata, dataDev, 1206);
		m_mutex.unlock();

		float fRXTemperature = ((pktdata[80] << 8) + pktdata[81]) * 0.061035 - 50;
		float fHV1 = ((pktdata[84] << 8) + pktdata[85]) * 0.00061035 * 51 - 2.5;

		float mMotorSpeed = 
			(pktdata[94] << 24) + 
			(pktdata[95] << 16) + 
			(pktdata[96] << 8) + 
			pktdata[97];

		mMotorSpeed = (mMotorSpeed < 1e-6) ? 0 : 60 / (mMotorSpeed * 0.000000032);

		double mRunningTime = 
			(pktdata[120] << 24) + 
			(pktdata[121] << 16) + 
			(pktdata[122] << 8) + 
			(pktdata[123]);

		mLidarStateParam.ReceiverTemperature = fRXTemperature;
		mLidarStateParam.ReceiverHighVoltage = fHV1;
		mLidarStateParam.MotorSpeed = mMotorSpeed;
		mLidarStateParam.PTP_State = pktdata[86];
		mLidarStateParam.GPS_State = pktdata[92];
		mLidarStateParam.PPS_State = pktdata[93]; 

		mLidarStateParam.Clock_Source = pktdata[44]; 
		mLidarStateParam.StandbyMode = pktdata[101];
		mLidarStateParam.PhaseLockedSwich = pktdata[102];
		mLidarStateParam.PhaseLockedState = pktdata[103];
		mLidarStateParam.RunningTime = mRunningTime / 60;

		return true;
	}
	else
	{
		InfoString = "Equipment package is not update!!!";
		return false;
	}

}

#pragma region //#pragma region //set  lidar parameters 
bool GetLidarData_LS::setLidarRotateSpeed(int SpeedValue, std::string& InfoString)
{
	if (setLidarParam())
	{
		if (SpeedValue != 0 && SpeedValue != 1 && SpeedValue != 2) {
			InfoString = "StateValue can only be equal to 0 or 1 or 2, please check the input parameters";
			return false;
		}
		Rest_UCWP_buff[100] = SpeedValue;
		return true;
	}
	else {
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

bool GetLidarData_LS::setLidarRotateState(int RotateState, std::string& InfoString) {

	InfoString = "The Lidar does not support this function (setLidarRotateState)!!";
	return false;
}


bool GetLidarData_LS::setLidarSoureSelection(int StateValue, std::string& InfoString) {
    if (setLidarParam()) {
        if (StateValue != 0 && StateValue != 1) {
			InfoString = "StateValue can only be equal to 0 or 1, please check the input parameters";
            return false;
        }
        Rest_UCWP_buff[44] = StateValue;
        return true;
    } else {
		InfoString = "Equipment package is not update!!!";
        return false;
    }
}

bool GetLidarData_LS::setLidarWorkState(int LidarState, std::string& InfoString) {
	if (setLidarParam()) {
		if (LidarState != 0 && LidarState != 1) {
			InfoString = "LidarState can only be equal to 0 or 1, please check the input parameters";
			return false;
		}
		Rest_UCWP_buff[101] = LidarState;
		return true;
	}
	else {
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

bool GetLidarData_LS::setGatewayIP(std::string IPString, std::string& InfoString)
{
	std::regex ipv4(
		"\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
	if (!regex_match(IPString, ipv4)) {
		InfoString = "The IP format entered is incorrect, please check the input parameters";
		return false;
	}

	if (setLidarParam()) {
		//set Gateway IP
		std::vector<std::string> IP_result;
		IP_result.clear();
		std::string::size_type DestIP_pos;
		IPString = IPString + ".";
		for (size_t i = 0; i < IPString.size(); i++)                                     //cut out of destIP lineedit 
		{
			DestIP_pos = IPString.find(".", i);
			if (DestIP_pos < IPString.size()) {
				std::string s = IPString.substr(i, DestIP_pos - i);
				IP_result.emplace_back(std::move(s));
				i = DestIP_pos;
			}
		}

		if (IP_result.size() < 4) {
			InfoString = "Please enter the full Gateway IP address!!! Failed to set the Gateway IP address!!!";
			return false;

		}
		else if (IP_result.size() == 4 && IP_result[3] == "") {
			InfoString = "Please enter the full Gateway IP address!!!Failed to set the Gateway IP address!!!";
			return false;
		}

		Rest_UCWP_buff[32] = atoi(IP_result[0].c_str());
		Rest_UCWP_buff[33] = atoi(IP_result[1].c_str());
		Rest_UCWP_buff[34] = atoi(IP_result[2].c_str());
		Rest_UCWP_buff[35] = atoi(IP_result[3].c_str());

		return true;
	}
	else {
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

bool GetLidarData_LS::setSubnetMaskIP(std::string IPString, std::string& InfoString)
{
	std::regex ipv4(
		"\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
	if (!regex_match(IPString, ipv4)) {
		InfoString = "The IP format entered is incorrect, please check the input parameters";
		return false;
	}

	if (setLidarParam()) {
		//set Gateway IP
		std::vector<std::string> IP_result;
		IP_result.clear();
		std::string::size_type DestIP_pos;
		IPString = IPString + ".";
		for (size_t i = 0; i < IPString.size(); i++)                                     //cut out of destIP lineedit 
		{
			DestIP_pos = IPString.find(".", i);
			if (DestIP_pos < IPString.size()) {
				std::string s = IPString.substr(i, DestIP_pos - i);
				IP_result.emplace_back(std::move(s));
				i = DestIP_pos;
			}
		}

		if (IP_result.size() < 4) {
			InfoString = "Please enter the full Subnet Mask IP address!!! Failed to set the  Subnet Mask IP address!!!";
			return false;

		}
		else if (IP_result.size() == 4 && IP_result[3] == "") {
			InfoString = "Please enter the full  Subnet Mask IP address!!!Failed to set the  Subnet Mask IP address!!!";
			return false;
		}

		Rest_UCWP_buff[36] = atoi(IP_result[0].c_str());
		Rest_UCWP_buff[37] = atoi(IP_result[1].c_str());
		Rest_UCWP_buff[38] = atoi(IP_result[2].c_str());
		Rest_UCWP_buff[39] = atoi(IP_result[3].c_str());

		return true;
	}
	else {
		InfoString = "Equipment package is not update!!!";
		return false;
	}
}

#pragma endregion 

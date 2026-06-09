#pragma once

#include "GetLidarData.h"

class GetLidarData_LS : public GetLidarData
{
public:
	GetLidarData_LS();
	~GetLidarData_LS();

	void LidarRun() override;
	
	virtual bool setLidarRotateSpeed(int SpeedValue, std::string& InfoString) override;					//Set frame rate 0 is the standard frame rate, 1 is 50% frame rate, and 2 is 25% frame rate
	bool setLidarRotateState(int StateValue, std::string& InfoString) override;							//Lidar rotate/stationary 
	bool setLidarSoureSelection(int StateValue, std::string& InfoString) override;						//time source selection
	bool setLidarWorkState(int StateValueV, std::string& InfoString) override;							//Lidar state, low power mode or not

	virtual bool setGatewayIP(std::string IPString, std::string& InfoString) override;					//set Gateway IP
	virtual bool setSubnetMaskIP(std::string IPString, std::string& InfoString) override;				//set Subnet Mask IP

	virtual bool getLidarParamState(LidarStateParam& mLidarStateParam, std::string& InfoString) override;

	int m_StackFrame = 1;
private:
	int count = 0;
	float m_fDistanceAcc = 0.001f;										//Distance accuracy: 1mm(0.001m)
	float m_fH_AngleAcc = 0.01f;										//horizontal angle accuracy: 0.01°
	float m_fV_AngleAcc = 0.0025f;										//vertical  angle accuracy为: 0.0025°

	int m_HorizontalPoints = 0;													
	float m_fPutTheMirrorOffAngle[4];									//The offset angle varies according to different channels
	float m_DeadZoneOffset = 6.37f;										//vertical  angle Offset rotation
	
	std::vector<MuchLidarData> PointCloudLastData;
	std::vector<MuchLidarData> tempPointCloud;
	void handleSingleEcho(unsigned char* data);									//single echo handling
	void handleDoubleEcho(unsigned char* data);									//double echo handling
	m_PointXYZ XYZ_calculate(int, float&, float&, float, float tDistance2);	//Coordinate conversion formula()  tDistance2： <=0, then he data of the second echo is not calculated
};


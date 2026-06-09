#pragma once

#include <iostream>
#include "GetLidarData.cpp"

#define GetLidarData_LS_MACRO												// predefined, fill in the lidar model that is to be used

#pragma region // HS1; CH16; CH32; CH32_WideAngle; CH64; CH120; CH128
//create class of HS1
#ifdef GetLidarData_HS1_MACRO
#include "GetLidarData_HS1.h"
#include "GetLidarData_HS1.cpp"													//.cpp file is not added to the project of *.vcxproj  and is added here; otherwise, there will be repeated definition if it is added to both two projects
GetLidarData* m_GetLidarData = new GetLidarData_HS1;							//create class of HS1
#endif

//create class of CH16
#ifdef GetLidarData_CH16_MACRO
#include "GetLidarData_CH16.h"
#include "GetLidarData_CH16.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH16;						//创create class of CH16
#endif

//create class of CH32
#ifdef GetLidarData_CH32_MACRO
#include "GetLidarData_CH32.h"
#include "GetLidarData_CH32.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH32;						//create class of CH32
#endif

//create class of CH32_WideAngle
#ifdef GetLidarData_CH32_WideAngle_MACRO
#include "GetLidarData_CH32_WideAngle.h"
#include "GetLidarData_CH32_WideAngle.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH32_WideAngle;			//create class of CH32_WideAngle
#endif

//create class of CH64
#ifdef GetLidarData_CH64_MACRO
#include "GetLidarData_CH64.h"
#include "GetLidarData_CH64.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH64;						//create class of CH64
#endif

//create class of CH120
#ifdef GetLidarData_CH120_MACRO
#include "GetLidarData_CH120.h"
#include "GetLidarData_CH120.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH120;						//create class of CH120
#endif

//create class of CH128
#ifdef GetLidarData_CH128_MACRO
#include "GetLidarData_CH128.h"
#include "GetLidarData_CH128.cpp"
	 GetLidarData* m_GetLidarData = new GetLidarData_CH128;						//create class of CH128
#endif
#pragma endregion

#pragma region// CH128x1; CH16x1; CH128S1; CX128S2; CX126S3; CX1S3; CX3S3; CX6S3; CX12S3; CXMS3; CB64S1; CB64S1_A; CH1W; CH1W_B; CH256; CVS4
//create class of CH128x1
#ifdef GetLidarData_CH128x1_MACRO
#include "GetLidarData_CH128x1.h"
#include "GetLidarData_CH128x1.cpp"
		 GetLidarData* m_GetLidarData = new GetLidarData_CH128x1;				//create class of CH128x1
#endif

// create class of CH16x1
#ifdef GetLidarData_CH16x1_MACRO
#include "GetLidarData_CH16x1.h"
#include "GetLidarData_CH16x1.cpp"
		 GetLidarData* m_GetLidarData = new GetLidarData_CH16x1;			//create class of CH16x1
#endif

// create class of CH128S1
#ifdef GetLidarData_CH128S1_MACRO
#include "GetLidarData_CH128S1.h"
#include "GetLidarData_CH128S1.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CH128S1;			// create class of CH128S1
#endif

// create class of CX128S2
#ifdef GetLidarData_CX128S2_MACRO
#include "GetLidarData_CX128S2.h"
#include "GetLidarData_CX128S2.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX128S2;			// create class of CX128S2
#endif

//  create class of CX126S3
#ifdef GetLidarData_CX126S3_MACRO
#include "GetLidarData_CX126S3.h"
#include "GetLidarData_CX126S3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX126S3;			//create class of CX126S3
#endif

// create class of CX1S3
#ifdef GetLidarData_CX1S3_MACRO
#include "GetLidarData_CX1S3.h"
#include "GetLidarData_CX1S3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX1S3;			//  create class of CX1S3
#endif

 // create class of CX3S3
#ifdef GetLidarData_CX3S3_MACRO
#include "GetLidarData_CX3S3.h"
#include "GetLidarData_CX3S3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX3S3;			//  create class of CX3S3
#endif

 //  create class of CX6S3, only display the data of S3's channel 19and channel 20. 
#ifdef GetLidarData_CX6S3_MACRO
#include "GetLidarData_CX6S3.h"
#include "GetLidarData_CX6S3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX6S3;			//  create class of CX6S3
#endif

 // create class of CX12S3
#ifdef GetLidarData_CX12S3_MACRO
#include "GetLidarData_CX12S3.h"
#include "GetLidarData_CX12S3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CX12S3;			//  create class of CX12S3
#endif

// create class of CXMS3
#ifdef GetLidarData_CXMS3_MACRO
#include "GetLidarData_CXMS3.h"
#include "GetLidarData_CXMS3.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CXMS3;			//  create class of CXMS3
#endif

//create class of CB64S1
#ifdef GetLidarData_CB64S1_MACRO
#include "GetLidarData_CB64S1.h"
#include "GetLidarData_CB64S1.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CB64S1;				// create class of CB64S1
#endif

 // create class of CB64S1_A
#ifdef GetLidarData_CB64S1_A_MACRO
#include "GetLidarData_CB64S1_A.h"
#include "GetLidarData_CB64S1_A.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CB64S1_A;			// create class of CB64S1_A
#endif

// create class of CH1W
#ifdef GetLidarData_CH1W_MACRO
#include "GetLidarData_CH1W.h"
#include "GetLidarData_CH1W.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CH1W;				// create class of CH1W
#endif

// create class of CH1W_B
#ifdef GetLidarData_CH1W_B_MACRO
#include "GetLidarData_CH1W_B.h"
#include "GetLidarData_CH1W_B.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CH1W_B;				// create class of CH1W_B
#endif

 //create class of CH256
#ifdef GetLidarData_CH256_MACRO
#include "GetLidarData_CH256.h"
#include "GetLidarData_CH256.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CH256;				//create class of CH256
#endif

//create class of CVS4
#ifdef GetLidarData_CVS4_MACRO
#include "GetLidarData_CVS4.h"
#include "GetLidarData_CVS4.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CVS4;				//create class of CVS4
#endif

#pragma endregion

#pragma region // C16_v3_0; C32_v3_0; C16_v4_0; C32_v4_0; C8_v4_0; C1_v4_0; C32_70D_v4_0; C32W_v4_0; CH32R_v4_0; CH32R_v4_0
//  create class of C16_v3_0
#ifdef GetLidarData_C16_v3_0_MACRO
#include "GetLidarData_C16_v3_0.h"
#include "GetLidarData_C16_v3_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C16_v3_0;			// create class of C16_v3_0
#endif

//create class of C32_v3_0
#ifdef GetLidarData_C32_v3_0_MACRO
#include "GetLidarData_C32_v3_0.h"
#include "GetLidarData_C32_v3_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C32_v3_0;			// create class of C32_v3_0
#endif

// create class of C16_v4_0
#ifdef GetLidarData_C16_v4_0_MACRO
#include "GetLidarData_C16_v4_0.h"
#include "GetLidarData_C16_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C16_v4_0;			// create class of C16_v4_0
#endif

// create class of C32_v4_0
#ifdef GetLidarData_C32_v4_0_MACRO
#include "GetLidarData_C32_v4_0.h"
#include "GetLidarData_C32_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C32_v4_0;			//create class of  C32_v4_0
#endif

// create class of C8_v4_0
#ifdef GetLidarData_C8_v4_0_MACRO
#include "GetLidarData_C8_v4_0.h"
#include "GetLidarData_C8_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C8_v4_0;			//create class of C8_v4_0
#endif

// create class of C1_v4_0
#ifdef GetLidarData_C1_v4_0_MACRO
#include "GetLidarData_C1_v4_0.h"
#include "GetLidarData_C1_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C1_v4_0;			//create class of C1_v4_0
#endif

// create class of C32_70D_v4_0
#ifdef GetLidarData_C32_70D_v4_0_MACRO
#include "GetLidarData_C32_70D_v4_0.h"
#include "GetLidarData_C32_70D_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C32_70D_v4_0;		// create class of  C32_70D_v4_0
#endif

// create class of   C32W_v4_0
#ifdef GetLidarData_C32W_v4_0_MACRO
#include "GetLidarData_C32W_v4_0.h"
#include "GetLidarData_C32W_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_C32W_v4_0;		// create class of  C32W_v4_0
#endif

// create class of  C32_90D_v4_0
#ifdef GetLidarData_CH32R_v4_0_MACRO
#include "GetLidarData_CH32R_v4_0.h"
#include "GetLidarData_CH32R_v4_0.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_CH32R_v4_0;		// create class of  CH32R_v4_0
#endif
#pragma endregion

#pragma region// LS; MS06; LS500W1; LS51;
// create class of  LS
#ifdef GetLidarData_LS_MACRO
#include "GetLidarData_LS.h"
#include "GetLidarData_LS.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_LS;				// create class of  LS
#endif

// create class of  MS06
#ifdef GetLidarData_MS06_MACRO
#include "GetLidarData_MS06.h"
#include "GetLidarData_MS06.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_MS06;				// create class of  MS06
#endif

// create class of  LS500W1
#ifdef GetLidarData_LS500W1_MACRO
#include "GetLidarData_LS500W1.h"
#include "GetLidarData_LS500W1.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_LS500W1;				// create class of  LS500W1
#endif

// create class of  LS51
#ifdef GetLidarData_LS51_MACRO
#include "GetLidarData_LS51.h"
#include "GetLidarData_LS51.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_LS51;				// create class of  LS51
#endif
#pragma endregion

#pragma region// N301 
//create class of N301 //6.0 lidar 1.7 protocol
#ifdef GetLidarData_N301_L6_v1_7MACRO      
#include "GetLidarData_N301.h"
#include "GetLidarData_N301.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_N301(4, 4, 7);						//create class of N301 //6.0 lidar 1.7 protocol
#endif

//create class of  N301 //5.0 lidar 1.7 protocol
#ifdef GetLidarData_N301_L5_v1_7MACRO     
#include "GetLidarData_N301.h"
#include "GetLidarData_N301.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_N301(4, 1, 7);					//create class of N301 //5.0 lidar 1.7 protocol
#endif

//create class of  N301 //5.0 lidar 1.6 protocol
#ifdef GetLidarData_N301_L5_v1_6MACRO      
#include "GetLidarData_N301.h"
#include "GetLidarData_N301.cpp"
			 GetLidarData* m_GetLidarData = new GetLidarData_N301(2, 1, 6);						//create class of N301 //5.0 lidar 1.6 protocol
#endif
#pragma endregion
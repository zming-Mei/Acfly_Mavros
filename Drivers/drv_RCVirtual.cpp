#include "Basic.hpp"
#include "FreeRTOS.h"
#include "timers.h"
#include "ReceiverBackend.hpp"
#include "Parameters.hpp"
TaskHandle_t VirtualRCTaskHandle;
static float rc_buf[16];
static void VirtualRC_Server(void* pvParameters);
void cal_virtual_RC();
void init_drv_RCVirtual()
{
	//接收机注册
	ReceiverRegister(SName("Virtual"));
	//cal_virtual_RC();
	xTaskCreate( VirtualRC_Server, "VirtualRC", 1024, NULL, 3, &VirtualRCTaskHandle);
}
void cal_virtual_RC()
{
	struct
	{
		uint8_t reflections[8];
		float minRcs[8];
		float scales[8];
	}__PACKED RCConfig;
	for( uint8_t i = 0; i < 4; ++i )
	{
		RCConfig.reflections[i] = i;
		RCConfig.minRcs[i] = 0;
		RCConfig.scales[i] = 1.0f;
	}
	for( uint8_t i = 4; i < 8; ++i )
	{
		if( i < 16 )
		{
			RCConfig.reflections[i] = i;		
			RCConfig.minRcs[i] = 0;
			RCConfig.scales[i] = 1.0f;
		}
		else
			RCConfig.reflections[i] = 255;
	}
	//更新参数
	PR_RESULT res = UpdateParamGroup( SName("RC_Virtual"), (uint64_t*)&RCConfig, 0, 9 );
}
static void VirtualRC_Server(void* pvParameters)
{
	//准确周期延时
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil( &xLastWakeTime, (1.0/20.0)*configTICK_RATE_HZ );
		rc_buf[0]=50;
		rc_buf[1]=50;
		rc_buf[2]=50;
		rc_buf[3]=50;
		rc_buf[4]=50;
		rc_buf[5]=99;
		rc_buf[6]=50;
		rc_buf[7]=50;
		rc_buf[8]=50;
		rc_buf[9]=50;
		rc_buf[10]=50;
		rc_buf[11]=50;
		rc_buf[12]=50;
		rc_buf[13]=50;
		rc_buf[14]=50;
		rc_buf[15]=50;
		ReceiverUpdate( "Virtual",true, rc_buf, 16, 0.01 );
		static bool calibrated=false;
		if(getInitializationCompleted())
		{
			if(!calibrated)
			{
				cal_virtual_RC();
				calibrated=true;
			}
		}
	}
}

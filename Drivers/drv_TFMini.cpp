#include "Commulink.hpp"
#include "Basic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SensorsBackend.hpp"
#include "MeasurementSystem.hpp"

#define SensorInd 2

struct DriverInfo
{
	uint32_t param;
	Port port;
};
double TF_mini_Z=0;

 bool get_TF_mini_Z( double* result, double TIMEOUT )//原子操作，无需互斥锁
{
	*result = TF_mini_Z;
	return true;
}

typedef struct
{
	int16_t Dist;	//Dist距离（30-1200cm）
	int16_t Strength;	//Strength信号强度（20-2000可信）
	uint8_t Mode;	//Mode测距档位
	uint8_t Rsv;	//预留
}__PACKED _TfMini;
static const unsigned char packet_ID[2] = { 0x59 , 0x59 };

static void TFMini_Server(void* pvParameters)
{
	DriverInfo driver_info = *(DriverInfo*)pvParameters;
	delete (DriverInfo*)pvParameters;
	
	/*状态机*/
		_TfMini  SensorD;
		unsigned char rc_counter = 0;
		unsigned char sum = 0;
	/*状态机*/
	
	while(1)
	{
		uint8_t rdata;
//		TF_mini_Z = 99;
		if( driver_info.port.read( &rdata, 1, 2, 0.5 ) )
		{
			if( rc_counter == 0 )
				sum = 0;
			if( rc_counter < 2 )
			{
				//接收包头
				if( rdata != packet_ID[ rc_counter ] )
					rc_counter = 0;
				else
				{
					sum += rdata;
					++rc_counter;
				}
			}
			else if( rc_counter < 8 )
			{	//接收数据
				( (unsigned char*)&SensorD )[ rc_counter - 2 ] = rdata;
				sum += (unsigned char)rdata;
				++rc_counter;
			}
			else
			{	//校验
				if( sum == rdata )
				{	//校验成功
					if( SensorD.Strength>20 && SensorD.Dist>1 && SensorD.Dist<5000 )
					{
						vector3<double> position;
						position.z = SensorD.Dist;
						//获取倾角
						Quaternion quat;
						get_Airframe_quat( &quat );
						double lean_cosin = quat.get_lean_angle_cosin();
						//更新
						position.z *= lean_cosin;
						PositionSensorUpdatePosition( default_vision_height_sensor_index, position, true );
						
						TF_mini_Z = position.z ;
					}
					else
						PositionSensorSetInavailable( default_vision_height_sensor_index );
				}
				rc_counter = 0;
			}
		}
	}
}

static bool TFMini_DriverInit( Port port, uint32_t param )
{
	//波特率115200
	port.SetBaudRate( 115200, 2, 2 );
	//注册传感器
//	bool res = PositionSensorRegister( SensorInd , \
//																			Position_Sensor_Type_RangePositioning , \
//																			Position_Sensor_DataType_s_z , \
//																			Position_Sensor_frame_ENU , \
//																			0.05 , //延时
//																			0 ,	//xy信任度
//																			0 //z信任度
//																			);
	
	bool res = 	PositionSensorRegister( default_vision_height_sensor_index , \
														Position_Sensor_Type_RangePositioning , \
														Position_Sensor_DataType_s_z , \
														Position_Sensor_frame_ENU , \
														0.034f , //延时
														0 ,	//xy信任度
														50 //z信任度
														);
	if(!res)
		return false;
	DriverInfo* driver_info = new DriverInfo;
	driver_info->param = param;
	driver_info->port = port;
	xTaskCreate( TFMini_Server, "OpticalFlow", 1024, (void*)driver_info, SysPriority_ExtSensor, NULL);
	return true;
}

void init_drv_TFMini()
{
	PortFunc_Register( 52, TFMini_DriverInit );
}
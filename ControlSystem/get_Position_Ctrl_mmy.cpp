#include "Sensors.hpp"
#include "MeasurementSystem.hpp"


bool get_Position_Ctrl_mmy( vector3<double>* result, double TIMEOUT )
{
	Position_Sensor radar;
	vector3<double>hheight;
	GetPositionSensor(default_vision_pos_sensor_index,&radar);
	get_Position(&hheight);
	//GetPositionSensor(default_vision_height_sensor_index,&hheight);
	result->x=radar. position.x;
	result->y=radar. position.y;
	result->z=hheight.z;
	return 1;
}

bool get_VelocityENU_Ctrl_mmy(vector3<double>* result, double TIMEOUT )
{
	Position_Sensor T265;
	GetPositionSensor(default_vision_speed_sensor_index,&T265);
	result->x=T265. velocity.x;
	result->y=T265. velocity.y;
	result->z=T265. velocity.y;;
	return 1;
}

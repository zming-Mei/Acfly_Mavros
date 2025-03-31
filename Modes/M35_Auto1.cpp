#include "Modes.hpp"
#include "M35_Auto1.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "NavCmdProcess.hpp"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"

#include "Basic.hpp"
#include "drv_Uart3.hpp"
#include "Commulink.hpp"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "event_groups.h"
#include "drv_LED.hpp"
static bool takeoff_in_progress=false;
static bool land_in_progress=false;
static bool RC_Control=true;

uint8_t mmy11=1;
uint8_t mmy12=2;
//uint8_t mmy13=12;
//uint8_t mmy14=13;
uint8_t *p11=&mmy11;
uint8_t *p12=&mmy12;
//uint8_t *p13=&mmy13;
//uint8_t *p14=&mmy14;


M35_Auto1::M35_Auto1():Mode_Base( "Offboard", 35 )
{
	
}

void M35_Auto1::get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode )
{
		*mode = AFunc_OffBoard;
}

ModeResult M35_Auto1::main_func( void* param1, uint32_t param2 )
{
	double freq = 50;
/*解锁*/
	set_MSafe_en(false);
	Altitude_Control_Enable();
	Position_Control_Enable();
	bool pos_ena;
	is_Position_Control_Enabled(&pos_ena);
	if(!pos_ena)
	{
		Attitude_Control_Disable();
		set_MSafe_en(true);
		return MR_Err;
	}
	else
	{
		setLedMode(LEDMode_Flying1);
	}
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	uint32_t RollOverProtectCounter = 0;
	bool in_speed_control_xy=false;
	bool in_speed_control_z=false;
	bool in_speed_control_yaw=false;	
	TIME last_XYSpeedTime;
	TIME last_ZSpeedTime;
	TIME last_YAWSpeedTime;
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//任务模式
	bool mode_switched = false;
	#define change_Mode(x) {cMode=x; mode_switched = true;}
	uint16_t current_mission_ind;
	uint8_t MissionButtonZone = 255;
	uint8_t RTLButtonZone = 255;
	uint8_t cMode = AFunc_PosHold;
	if(param1)
	{
		cMode = *(uint8_t*)param1;
		//uint8_t *p15=&cMode;
		//Write_Uart3(p15,2,1,1);//OUT=cMode
	}
	//当前执行任务的序号
	uint16_t mission_ind = 0;
	//任务状态机
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{	
		os_delay(0.02);
		Position_Control_set_XYZAutoSpeed(40);
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			set_MSafe_en(true);
			return MR_Err;
		}
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		bool msg_handled = false;
		
		int32_t msg_param2=0;
		
					/*返回消息处理结果*/
		
		
			if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//地面站加锁
			bool inFlight;
			get_is_inFlight(&inFlight); //上锁 保护
			if( msg.params[0] == 0 &&(!inFlight))//disarm
			{
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK )
				{
					Attitude_Control_Disable();
					set_mav_mode( 
							MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
							PX4_CUSTOM_MAIN_MODE_ALTCTL,
							0 );
					os_delay(1.0);
					
					uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
					const Port* port = get_CommuPort( port_index );
					if( port->write )
					{
						mavlink_message_t msg_sd;
						if( mavlink_lock_chan( port_index, 0.01 ) )
						{
							mavlink_msg_command_ack_pack_chan( 
								get_CommulinkSysId() ,	//system id
								get_CommulinkCompId() ,	//component id
								port_index ,
								&msg_sd,
								msg.cmd,	//command
								MAV_RESULT_ACCEPTED ,	//result
								100 ,	//progress
								0 ,	//param2
								msg.sd_sysid ,	//target system
								msg.sd_compid //target component
							);
							mavlink_msg_to_send_buffer(port->write, 
																				 port->lock,
																				 port->unlock,
																				 &msg_sd, 0, 0.01);
							mavlink_unlock_chan(port_index);
						}
					}
				}
				set_MSafe_en(true);
				return MR_OK;
			}
		}
		if( get_CrashedState() )
		{	//侧翻加锁
			Attitude_Control_Disable();
			set_MSafe_en(true);
			return MR_Err;
		}
		
    
		uint8_t reqMode = cMode;

		if( rc.available )
		{	//接收机可用
			
			//使用遥控器更新飞行模式
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//摇杆没回中不允许自动操作
                RC_Control = true;
				if( is_AFunc_auto(cMode) )
				{
					reqMode = AFunc_PosHold;
					MissionButtonZone = RTLButtonZone = 255;
				}					
			}
			else
			{	//摇杆回中可执行自动操作	
				
				/*判断执行任务*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//按钮按下执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone )
							{	//按钮状态发生变化
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_OffBoard;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//按钮变化执行任务
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
							{	//按钮状态发生变化
								if( cMode != AFunc_OffBoard )
									reqMode = AFunc_OffBoard;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*判断执行任务*/
				
				/*判断返航*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//按钮按下返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone )
							{	//按钮状态发生变化	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//按钮变化返航
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//获取按钮状态
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
							{	//按钮状态发生变化
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*判断返航*/
					
				if( reqMode == 0 )
				{	//有按钮松开重新检测按钮位置
					
					/*判断执行任务*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//按钮按下执行任务
							if( MissionButtonZone>=4 )
								reqMode = AFunc_OffBoard;
						}
					/*判断执行任务*/
						
					/*判断返航*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//按钮按下返航
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*判断返航*/
						
					if( reqMode == 0 )
						reqMode = AFunc_PosHold;
				}
				
			}
		}
		else
		{	//接收机不可用重置遥控状态
			 MissionButtonZone = RTLButtonZone = 255;
			//如果不是自动模式则切换到返航模式
			if( is_AFunc_auto(cMode)==false )
				reqMode = AFunc_RTL;
		}
		
		if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		{	//进出自动模式置位mode_swithced
			if( cMode != reqMode )
			{
				cMode = reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = reqMode;
		if( cMode==AFunc_RTL )
		{	//进入安全模式返航
RTL:
			set_MSafe_en(true);
			enter_MSafe(true);
			/*判断退出模式*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					set_MSafe_en(true);
					return MR_OK;
				}
			/*判断退出模式*/
		}
		else if( cMode==AFunc_OffBoard )
		{	//任务模式
        RC_Control = false ; 
			if( rc.available )
			{
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , 5 );
				if( !sticks_in_neutral )
				{	//摇杆不在中间返回手动模式
					init_NavCmdInf(&navInf);
					reqMode = AFunc_PosHold;
					goto Manual_Mode;
				}
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待			
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{	//进入任务飞行模式
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//mavlink指令接收开启
				//处理消息
				
				if( msg_available)
				{

					switch( msg.cmd )
					{
						
							case MAV_CMD_DO_SET_SERVO:
						{
							if(msg.params[0]==1||msg.params[0]==2)
							 {
								 uint8_t color_number[2];
								 color_number[0]=msg.params[0];
								 color_number[1]=msg.params[1];
//								 if(msg.params[1]==10)
//								 {
//								 set_BuzzerFreq(1500);
//                 set_BuzzerOnOff(true);
//								 }
//								 if(msg.params[1]==11)
//								 {
//									 set_BuzzerOnOff(false);
//								 }
								 Write_Uart3(color_number,2,1,1);
								 msg_handled = true; 
							 }
							 
							 if(msg.params[0]==9)
							 {
								 TIM3->ARR = 1e6 / 50;
								 TIM3->CCR1 = uint32_t(msg.params[1]);
								 msg_handled = true; 
							 }
							 	break;
						}
					
						case MAV_CMD_NAV_TAKEOFF:
						{
							if((!takeoff_in_progress)&&(!land_in_progress))
							{//takeoff起飞
								bool pos_ena;
								is_Position_Control_Enabled(&pos_ena);
								if( pos_ena )
								{
									Position_Control_set_XYLock();
									in_speed_control_z=false;
									Position_Control_Takeoff_HeightRelative(msg.params[6]*100);
									takeoff_in_progress=true;
									msg_handled = true;		
								}
								else
								{
									Attitude_Control_Disable();
									set_MSafe_en(true);
									return MR_Err;
								}
							}
							
						break;
						}
						
						case MAV_CMD_NAV_LAND:
						{	//降落
							if((!takeoff_in_progress)&&(!land_in_progress))
							{
								//Write_Uart3(p11,1,1,1);
								Position_Control_set_TargetVelocityZ(-30);
								Position_Control_set_XYLock();
								in_speed_control_z=false;
								land_in_progress=true;
								msg_handled = true;
	
							}
							break;
						}
						
						case 1://request Z Controller state
						{
							Position_ControlMode mode;
							get_Altitude_ControlMode(&mode);
							msg_param2 = mode;
							msg_handled = true;
							break;
						}
						case 2://request XY Controller state
						{
							Position_ControlMode mode;
							get_Position_ControlMode(&mode);
							msg_param2 = mode;
							msg_handled = true;
							break;
						}
					}
				}
		
		/*返回消息处理结果*/
			if( msg_available )
			{
				uint8_t port_index = msg.cmd_type & CMD_TYPE_PORT_MASK;
				const Port* port = get_CommuPort( port_index );
				if( (msg.cmd_type & CMD_TYPE_MASK) == CMD_TYPE_MAVLINK && port->write )
				{
					mavlink_message_t msg_sd;
					if( mavlink_lock_chan( port_index, 0.01 ) )
					{
						mavlink_msg_command_ack_pack_chan( 
							get_CommulinkSysId() ,	//system id
							get_CommulinkCompId() ,	//component id
							port_index ,
							&msg_sd,
							msg.cmd,	//command
							msg_handled==1 ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED ,	//result
							100 ,	//progress
							msg_param2 ,	//param2
							msg.sd_sysid ,	//target system
							msg.sd_compid //target component
						);
						mavlink_msg_to_send_buffer(port->write, 
																			 port->lock,
																			 port->unlock,
																			 &msg_sd, 0, 0.01);
						mavlink_unlock_chan(port_index);
					}
				}
				msg_available=false;//message replyed
			}
			
		
			bool msg_pos_available;
			ModeMsg_Pos msg_pos;
			msg_pos_available = ModeReceiveMsg_Pos( &msg_pos, 0 );
			
			if(msg_pos_available &&(!takeoff_in_progress)&&(!land_in_progress)) //没有起飞和降落
			{
				switch (msg_pos.coordinate_frame)
				{
        case MAV_FRAME_LOCAL_NED:
        {
           float north=msg_pos.x*100;//m to cm
            float east=msg_pos.y*100;
            float up=-msg_pos.z*100;
            float north_speed=msg_pos.vx*100;//m to cm
            float east_speed=msg_pos.vy*100;				
            float up_vel=-msg_pos.vz*100;
            float yaw=Pi/2-msg_pos.yaw;
						if(yaw>Pi)yaw-=2*Pi;
						if(yaw<-Pi)yaw+=2*Pi;
            float yaw_rate=-msg_pos.yaw_rate;
					
		
							
						if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)))
						{
								Position_Control_set_TargetPositionXY(east,north);
								in_speed_control_xy=false;
						}
						else if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)))
						{
							if( abs(east_speed)<0.01&&abs(north_speed)<0.01)
								Position_Control_set_XYLock();
							else
								Position_Control_set_TargetVelocityXY_AngleLimit(east_speed,north_speed);
							
								last_XYSpeedTime=TIME::now();						
								in_speed_control_xy=true;
						}
							
            if(!(msg_pos.type_mask &	POSITION_TARGET_TYPEMASK_YAW_IGNORE))
            {
                Attitude_Control_set_Target_Yaw(yaw);
								in_speed_control_yaw=false;
            }
            else if(!(msg_pos.type_mask &	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE))
            {
							if(abs(yaw_rate)<0.01)
								Attitude_Control_set_YawLock();
							else
                Attitude_Control_set_Target_YawRate(yaw_rate);
							
								last_YAWSpeedTime=TIME::now();
								in_speed_control_yaw=true;
            }
            if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_Z_IGNORE))
            {
                Position_Control_set_TargetPositionZ(up);
            }
            else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_VZ_IGNORE))
            {
							if(abs(up_vel)<0.01)
								Position_Control_set_ZLock();
							else
                Position_Control_set_TargetVelocityZ(up_vel);
							
							in_speed_control_z=true;
								last_ZSpeedTime=TIME::now();
            }
				
        }
        break;
				
        case MAV_FRAME_LOCAL_ENU:
        {

					 float east=msg_pos.x*100;//m to cm
            float north=msg_pos.y*100;
            float east_speed=msg_pos.vx*100;//m to cm
            float north_speed=msg_pos.vy*100;					
            float up=msg_pos.z*100;
            float up_vel=msg_pos.vz*100;
            float yaw=msg_pos.yaw;
            float yaw_rate=msg_pos.yaw_rate;
            if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)))
            {
                Position_Control_set_TargetPositionXY(east,north);
								in_speed_control_xy=false;
            }
						else if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)))
						{
							if( abs(east_speed)<0.01&& abs(north_speed)<0.01)
								Position_Control_set_XYLock();
							else
								Position_Control_set_TargetVelocityXY_AngleLimit(east_speed,north_speed);
							
							last_XYSpeedTime=TIME::now();
							in_speed_control_xy=true;
						}
            if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_YAW_IGNORE))
            {
                Attitude_Control_set_Target_Yaw(yaw);
								in_speed_control_yaw=false;
            }
            else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE))
            {
							if(abs(yaw_rate)<0.01)
								Attitude_Control_set_YawLock();
							else						
                Attitude_Control_set_Target_YawRate(yaw_rate);
							
								last_YAWSpeedTime=TIME::now();
								in_speed_control_yaw=true;
            }
            if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_Z_IGNORE))
            {
                Position_Control_set_TargetPositionZ(up);
								in_speed_control_z=false;
            }
            else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_VZ_IGNORE))
            {
							if(abs(up_vel)<0.01)
								Position_Control_set_ZLock();
							else
                Position_Control_set_TargetVelocityZ(up_vel);
							
								in_speed_control_z=true;
								last_ZSpeedTime=TIME::now();
            }

        }
        break;
				
        case MAV_FRAME_BODY_FRD://x y z & yaw is relative
					
				case MAV_FRAME_BODY_OFFSET_NED://ardupilot-like support
        {
            float forward=(msg_pos.x)*100;
            float left=-(msg_pos.y)*100;
            float up=-(msg_pos.z)*100;
            float forward_vel=(msg_pos.vx)*100;
            float left_vel=-(msg_pos.vy)*100;
            float up_vel=-(msg_pos.vz)*100;
            float yaw=-msg_pos.yaw;
            float yaw_rate=-msg_pos.yaw_rate;
					
							if((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE))
							{
								if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VX_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_VY_IGNORE)))
								{
									if( abs(forward_vel)<0.01&& abs(left_vel)<0.01 )
										Position_Control_set_XYLock();
									else
										Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(forward_vel,left_vel);
									
									in_speed_control_xy=true;
									last_XYSpeedTime=TIME::now();
								}
							}
							else
							{
								if(abs(forward)<0.01 && abs(left)<0.01 )
									Position_Control_set_XYLock();
								else
									Position_Control_set_TargetPositionXYRelativeBodyheading(forward,left);
								
								in_speed_control_xy=false;
							}
						
						
							if(!(msg_pos.type_mask &	POSITION_TARGET_TYPEMASK_YAW_IGNORE))
							{
								if(abs(yaw)<0.01)
									Attitude_Control_set_YawLock();
								else
									Attitude_Control_set_Target_YawRelative(yaw);
								
									in_speed_control_yaw=false;
							}
							else if(!(msg_pos.type_mask &	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE))
							{
														
								if( abs(yaw_rate)<0.01 )
									Attitude_Control_set_YawLock();
								else
									Attitude_Control_set_Target_YawRate(yaw_rate);
									in_speed_control_yaw=true;
									last_YAWSpeedTime=TIME::now();
							}
						
							if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_Z_IGNORE))
							{
								if(abs(up)<0.01)
									Position_Control_set_ZLock();
								else
									Position_Control_set_TargetPositionZRelative(up);
								
								in_speed_control_z=false;
							}
							else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_VZ_IGNORE))
							{
								if(abs(up_vel)<0.01)
									Position_Control_set_ZLock();
								else
									Position_Control_set_TargetVelocityZ(up_vel);
								
								in_speed_control_z=true;
								last_ZSpeedTime=TIME::now();
							}
							
							
						double body_info[8]={forward/100.0,left/100.0,up/100.0,forward_vel/100.0,left_vel/100.0,up_vel/100.0,yaw,yaw_rate};
						SDLog_Msg_DebugVect( "Body_frame", body_info, 8 );
											
        }			
        break;
				
        case MAV_FRAME_LOCAL_OFFSET_NED:
        {
            float north=(msg_pos.x)*100;
            float east=(msg_pos.y)*100;
            float up=-(msg_pos.z)*100;
            float up_vel=-(msg_pos.vz)*100;
            float yaw=Pi/2-msg_pos.yaw;
						if(yaw>Pi)yaw-=2*Pi;
						if(yaw<-Pi)yaw+=2*Pi;
            float yaw_rate=-msg_pos.yaw_rate;

            if(!((msg_pos.type_mask&POSITION_TARGET_TYPEMASK_X_IGNORE)||(msg_pos.type_mask&POSITION_TARGET_TYPEMASK_Y_IGNORE)))
            {
							if(abs(east)<0.01&&abs(north)<0.01)
									Position_Control_set_XYLock();
							else
                Position_Control_set_TargetPositionXYRelative(east,north);
							
								in_speed_control_xy=false;
            }
            if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_YAW_IGNORE))
            {
                Attitude_Control_set_Target_Yaw(yaw);
								in_speed_control_yaw=false;
            }
            else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE))
            {
							if( abs(yaw_rate)<0.01 )
								Attitude_Control_set_YawLock();
							else
                Attitude_Control_set_Target_YawRate(yaw_rate);
							
								in_speed_control_yaw=true;
								last_YAWSpeedTime=TIME::now();
            }
            if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_Z_IGNORE))
            {
								if(abs(up)<0.01)
									Position_Control_set_ZLock();
								else
									Position_Control_set_TargetPositionZRelative(up);
								
								in_speed_control_z=false;
            }
            else if(!(msg_pos.type_mask&	POSITION_TARGET_TYPEMASK_VZ_IGNORE))
            {
							if(abs(up_vel)<0.01)
								Position_Control_set_ZLock();
							else
                Position_Control_set_TargetVelocityZ(up_vel);
							
								in_speed_control_z=true;
								last_ZSpeedTime=TIME::now();
            }
        }
        break;
				}
				
			}
				
				//设定mavlink模式
				set_mav_mode( 
					MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
					PX4_CUSTOM_MAIN_MODE_OFFBOARD,
					0 );
				
			}
			
			if(takeoff_in_progress)
			{
				Position_ControlMode mode;
				Position_Control_set_XYLock();
				get_Altitude_ControlMode(&mode);
				if( mode == Position_ControlMode_Position )
				{
					takeoff_in_progress=false;
				}
			}
			else if(land_in_progress)  
			{
					Position_Control_set_XYLock();
					Position_Control_set_TargetVelocityZ(-30);
					bool inFlight;
					get_is_inFlight(&inFlight);
					if(!inFlight)//not in air anymore
					{
						land_in_progress=false;
						Attitude_Control_Disable();
						set_MSafe_en(true);
						return MR_OK;
					}
			}
			if(last_XYSpeedTime.get_pass_time()>1&&in_speed_control_xy)//xy速度控制超时
			{
					Position_Control_set_XYLock();
					in_speed_control_xy=false;
			}
			if(last_ZSpeedTime.get_pass_time()>1&&in_speed_control_z)//z速度控制超时
			{
					Position_Control_set_ZLock();
					in_speed_control_z=false;
			}
			if(last_YAWSpeedTime.get_pass_time()>1&&in_speed_control_yaw)//yaw速度控制超时
			{
					Attitude_Control_set_YawLock();
					in_speed_control_yaw=false;
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
            RC_Control = true ;
			if( rc.available )
			{
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							set_MSafe_en(true);
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//手势强制加锁
					if( rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90 )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							set_MSafe_en(true);
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*判断退出模式*/
				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				bool alt_ena;
				is_Altitude_Control_Enabled(&alt_ena);					
				//油门杆控制垂直速度
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
				{
					double thr_stick = remove_deadband( rc.data[0] - 50.0 , 5.0 );
					if( thr_stick > 0 )
						thr_stick *= get_maxVelUp() / 50;
					else
						thr_stick *= get_maxVelDown() / 50;
					Position_Control_set_TargetVelocityZ(thr_stick);
				}
				
				if( pos_ena )
				{
					//设定mavlink模式
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
					
					//俯仰横滚杆控水平速度
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double RPCtrlScale = atan2( get_maxAccXY() / 50.0, GravityAcc );
						double XYCtrlScale = get_maxVelXY() / 50.0;						
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						vector3<double> velocityFLU;
						get_VelocityFLU_Ctrl(&velocityFLU);
						double vel_stick_err_pitch = velocityFLU.x/XYCtrlScale - pitch_sitck_d;
						double vel_stick_err_roll = velocityFLU.y/XYCtrlScale - -roll_sitck_d;
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
							pitch_sitck_d * XYCtrlScale ,\
							-roll_sitck_d * XYCtrlScale , \
							fabs( vel_stick_err_roll  )*RPCtrlScale, \
							fabs( vel_stick_err_pitch )*RPCtrlScale \
						);
					}
				}
				else if(alt_ena)
				{
					//设定mavlink模式
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );
					
					//补偿风力扰动
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					//俯仰横滚杆控俯仰横滚
					double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
	//				Attitude_Control_set_Target_RollPitch( 
	//					( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
	//					( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
	//				);
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*RPCtrlScale,
						( rc.data[2] - 50 )*RPCtrlScale
					);
				}
				
				//偏航杆在中间锁偏航
				//不在中间控制偏航速度
				double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
			}
			else
			{	//无遥控信号进入安全模式
				change_Mode(AFunc_RTL)
				goto RTL;				
			}
		}
	}
	set_MSafe_en(true);
	return MR_OK;
}

bool get_is_takingoff( bool* result, double TIMEOUT )//原子操作，无需互斥锁
{
	*result = takeoff_in_progress;
	return true;
}

bool get_is_landing( bool* result, double TIMEOUT )//原子操作，无需互斥锁
{
	*result = land_in_progress;
	return true;
}
bool get_is_RC_Control( bool* result, double TIMEOUT )//原子操作，无需互斥锁
{
	*result = RC_Control;
	return true;
}
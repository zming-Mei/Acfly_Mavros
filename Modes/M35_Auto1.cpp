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
/*����*/
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
	//��ȡģʽ����
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	
	//����ģʽ
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
	//��ǰִ����������
	uint16_t mission_ind = 0;
	//����״̬��
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	while(1)
	{	
		os_delay(0.02);
		Position_Control_set_XYZAutoSpeed(40);
		if( get_CrashedState() )
		{	//�෭����
			Attitude_Control_Disable();
			set_MSafe_en(true);
			return MR_Err;
		}
		//��ȡ���ջ�
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//��ȡ��Ϣ
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg, 0 );
		bool msg_handled = false;
		
		int32_t msg_param2=0;
		
					/*������Ϣ������*/
		
		
			if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		{	//����վ����
			bool inFlight;
			get_is_inFlight(&inFlight); //���� ����
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
		{	//�෭����
			Attitude_Control_Disable();
			set_MSafe_en(true);
			return MR_Err;
		}
		
    
		uint8_t reqMode = cMode;

		if( rc.available )
		{	//���ջ�����
			
			//ʹ��ң�������·���ģʽ
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , MFunc_cfg.NeutralZone[0] ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , MFunc_cfg.NeutralZone[0] );
			if( !sticks_in_neutral )
			{	//ҡ��û���в������Զ�����
                RC_Control = true;
				if( is_AFunc_auto(cMode) )
				{
					reqMode = AFunc_PosHold;
					MissionButtonZone = RTLButtonZone = 255;
				}					
			}
			else
			{	//ҡ�˻��п�ִ���Զ�����	
				
				/*�ж�ִ������*/
					if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
					{	//��ť����ִ������
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]+4 )
						{			
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-1+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );									
							if( new_MissionButtonZone!=MissionButtonZone )
							{	//��ť״̬�����仯
								if( new_MissionButtonZone>=4 )
									reqMode = AFunc_OffBoard;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
					else if( MFunc_cfg.MissionBt[0]>=12 && MFunc_cfg.MissionBt[0]<=14 )
					{	//��ť�仯ִ������
						if( rc.available_channels >= MFunc_cfg.MissionBt[0]-10+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.MissionBt[0]-11+4];
							uint8_t new_MissionButtonZone = get_RcButtonZone( btn_value, MissionButtonZone );
							if( MissionButtonZone<=5 && new_MissionButtonZone!=MissionButtonZone )
							{	//��ť״̬�����仯
								if( cMode != AFunc_OffBoard )
									reqMode = AFunc_OffBoard;
								else
									reqMode = 0;
							}
							MissionButtonZone = new_MissionButtonZone;
						}
					}
				/*�ж�ִ������*/
				
				/*�жϷ���*/
					if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
					{	//��ť���·���
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-1+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );		
							if( new_RTLButtonZone!=RTLButtonZone )
							{	//��ť״̬�����仯	
								if( new_RTLButtonZone>=4 )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
					else if( MFunc_cfg.RTLBt[0]>=12 && MFunc_cfg.RTLBt[0]<=14 )
					{	//��ť�仯����
						if( rc.available_channels >= MFunc_cfg.RTLBt[0]-10+4 )
						{
							//��ȡ��ť״̬
							double btn_value = rc.data[MFunc_cfg.RTLBt[0]-11+4];
							uint8_t new_RTLButtonZone = get_RcButtonZone( btn_value, RTLButtonZone );	
							if( RTLButtonZone<=5 && new_RTLButtonZone!=RTLButtonZone )
							{	//��ť״̬�����仯
								if( cMode != AFunc_RTL )
									reqMode = AFunc_RTL;
								else
									reqMode = 0;
							}
							RTLButtonZone = new_RTLButtonZone;
						}
					}
				/*�жϷ���*/
					
				if( reqMode == 0 )
				{	//�а�ť�ɿ����¼�ⰴťλ��
					
					/*�ж�ִ������*/
						if( MFunc_cfg.MissionBt[0]>=2 && MFunc_cfg.MissionBt[0]<=4 )
						{	//��ť����ִ������
							if( MissionButtonZone>=4 )
								reqMode = AFunc_OffBoard;
						}
					/*�ж�ִ������*/
						
					/*�жϷ���*/
						if( MFunc_cfg.RTLBt[0]>=2 && MFunc_cfg.RTLBt[0]<=4 )
						{	//��ť���·���
							if( RTLButtonZone>=4 )
								reqMode = AFunc_RTL;
						}
					/*�жϷ���*/
						
					if( reqMode == 0 )
						reqMode = AFunc_PosHold;
				}
				
			}
		}
		else
		{	//���ջ�����������ң��״̬
			 MissionButtonZone = RTLButtonZone = 255;
			//��������Զ�ģʽ���л�������ģʽ
			if( is_AFunc_auto(cMode)==false )
				reqMode = AFunc_RTL;
		}
		
		if( is_AFunc_auto(reqMode) || is_AFunc_auto(cMode) )
		{	//�����Զ�ģʽ��λmode_swithced
			if( cMode != reqMode )
			{
				cMode = reqMode;
				mode_switched = true;
			}
		}
		else
			cMode = reqMode;
		if( cMode==AFunc_RTL )
		{	//���밲ȫģʽ����
RTL:
			set_MSafe_en(true);
			enter_MSafe(true);
			/*�ж��˳�ģʽ*/
				bool inFlight;
				get_is_inFlight(&inFlight);
				if( inFlight==false )
				{
					Attitude_Control_Disable();
					set_MSafe_en(true);
					return MR_OK;
				}
			/*�ж��˳�ģʽ*/
		}
		else if( cMode==AFunc_OffBoard )
		{	//����ģʽ
        RC_Control = false ; 
			if( rc.available )
			{
				bool sticks_in_neutral = 
					in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
					in_symmetry_range_mid( rc.data[3] , 50 , 5 );
				if( !sticks_in_neutral )
				{	//ҡ�˲����м䷵���ֶ�ģʽ
					init_NavCmdInf(&navInf);
					reqMode = AFunc_PosHold;
					goto Manual_Mode;
				}
			}
			
			if( mode_switched )
			{	//���л�������ģʽ
				//����ɲ���ȴ�			
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//�ȴ�ɲ�����
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//ɲ�����
					++navInf.counter2;
					//�ȴ�1���ٽ����������
					if( navInf.counter2 >= 1*freq )
					{	//�����������ģʽ
						mode_switched = false;
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//mavlinkָ����տ���
				//������Ϣ
				
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
							{//takeoff���
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
						{	//����
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
		
		/*������Ϣ������*/
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
			
			if(msg_pos_available &&(!takeoff_in_progress)&&(!land_in_progress)) //û����ɺͽ���
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
				
				//�趨mavlinkģʽ
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
			if(last_XYSpeedTime.get_pass_time()>1&&in_speed_control_xy)//xy�ٶȿ��Ƴ�ʱ
			{
					Position_Control_set_XYLock();
					in_speed_control_xy=false;
			}
			if(last_ZSpeedTime.get_pass_time()>1&&in_speed_control_z)//z�ٶȿ��Ƴ�ʱ
			{
					Position_Control_set_ZLock();
					in_speed_control_z=false;
			}
			if(last_YAWSpeedTime.get_pass_time()>1&&in_speed_control_yaw)//yaw�ٶȿ��Ƴ�ʱ
			{
					Attitude_Control_set_YawLock();
					in_speed_control_yaw=false;
			}
		}
		else
		{	//�ֶ�����ģʽ���������߶�����ƣ�
			Manual_Mode:
            RC_Control = true ;
			if( rc.available )
			{
				/*�ж��˳�ģʽ*/
					//��ȡ����״̬
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 480;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//������Զ�����
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
					//����ǿ�Ƽ���
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
				/*�ж��˳�ģʽ*/
				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				bool alt_ena;
				is_Altitude_Control_Enabled(&alt_ena);					
				//���Ÿ˿��ƴ�ֱ�ٶ�
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
					//�趨mavlinkģʽ
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_POSCTL,
						0 );
					
					//��������˿�ˮƽ�ٶ�
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
					//�趨mavlinkģʽ
					set_mav_mode( 
						MAV_MODE_STABILIZE_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
						PX4_CUSTOM_MAIN_MODE_ALTCTL,
						0 );
					
					//���������Ŷ�
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					//��������˿ظ������
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
				
				//ƫ�������м���ƫ��
				//�����м����ƫ���ٶ�
				double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
			}
			else
			{	//��ң���źŽ��밲ȫģʽ
				change_Mode(AFunc_RTL)
				goto RTL;				
			}
		}
	}
	set_MSafe_en(true);
	return MR_OK;
}

bool get_is_takingoff( bool* result, double TIMEOUT )//ԭ�Ӳ��������軥����
{
	*result = takeoff_in_progress;
	return true;
}

bool get_is_landing( bool* result, double TIMEOUT )//ԭ�Ӳ��������軥����
{
	*result = land_in_progress;
	return true;
}
bool get_is_RC_Control( bool* result, double TIMEOUT )//ԭ�Ӳ��������軥����
{
	*result = RC_Control;
	return true;
}
#pragma once

#include "Modes.hpp"

class M35_Auto1:public Mode_Base 
{
	private:
		
	public:
		M35_Auto1();
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode );
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};
		bool get_is_takingoff( bool* result, double TIMEOUT = -1);

		bool get_is_landing( bool* result, double TIMEOUT = -1);
        
    bool get_is_RC_Control( bool* result, double TIMEOUT = -1 );
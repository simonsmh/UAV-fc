#include "DY_Flight_Log.h"
#include "DY_Tracking.h"

#include "DY_AltCtrl.h"
#include "DY_AttCtrl.h"
#include "DY_FlightCtrl.h"
#include "DY_Math.h"
#include "Drv_time.h"
#include "DY_DT.h"

_DY_flag dy_flag;
u8 DY_ControlConvert_Tracking_Flag=1;
void DY_Flight_Control(void)
{
    /***************·ÉÐÐ¿ØÖÆ***************/
	if(dy_flag.pit_go == 1)
	{
		DY_DT_SendString("PIT Go", sizeof("PIT Go"));
		dy_flag.pit_go = 0;

        dy_pit = dy_ov_pit;
	}
	if(dy_flag.pit_back == 1)
	{
		DY_DT_SendString("PIT Back", sizeof("PIT Back"));
		dy_flag.pit_back = 0;

        dy_pit = -(dy_ov_pit);
	}
	if(dy_flag.rol_left == 1)
	{
		DY_DT_SendString("ROL Left", sizeof("ROL Left"));
		dy_flag.rol_left = 0;

        dy_rol = dy_ov_rol;
	}
	if(dy_flag.rol_right == 1)
	{
		DY_DT_SendString("ROL Right", sizeof("ROL Right"));
		dy_flag.rol_right = 0;

        dy_rol = -(dy_ov_rol);
	}
	if(dy_flag.yaw_ccw == 1)
	{
		DY_DT_SendString("Counter Clockwise", sizeof("Counter Clockwise"));
		dy_flag.yaw_ccw = 0;

        dy_yaw = dy_ov_yaw;
	}
	if(dy_flag.yaw_cw == 1)
	{
		DY_DT_SendString("Clockwise", sizeof("Clockwise"));
		dy_flag.yaw_cw = 0;

        dy_yaw = -(dy_ov_yaw);
	}
	if(dy_flag.up == 1)
	{
		DY_DT_SendString("Height Up", sizeof("Height Up"));
		dy_flag.up = 0;

		dy_height = dy_ov_height;
	}
	if(dy_flag.down == 1)
	{
		DY_DT_SendString("Height Down", sizeof("Height Down"));
		dy_flag.down = 0;

		dy_height = -(dy_ov_height);
	}
	if(dy_flag.stop == 1)
	{
		DY_DT_SendString("OpenMv Stop", sizeof("OpenMv Stop"));
		dy_flag.stop = 0;

		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
	}
	if(dy_flag.land == 1)
	{
		DY_DT_SendString("OpenMv End", sizeof("OpenMv End"));
		dy_flag.land = 0;

		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		one_key_land();
	}
}

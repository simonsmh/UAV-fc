#include "DY_AltCtrl.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "DY_MotionCal.h"
#include "DY_FlightCtrl.h"
#include "DY_MotorCtrl.h"
#include "DY_AttCtrl.h"
#include "DY_LocCtrl.h"

#include "DY_FlightCtrl.h"

static s16 auto_taking_off_speed;

/***************高度控制变量初始化***************/
s16 dy_height = 0;

void Auto_Take_Off_Land_Task()
{
	if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
	{
		auto_taking_off_speed = 0;
	}
	else if (flag.auto_take_off_land == AUTO_TAKE_OFF)
	{
		auto_taking_off_speed = 40;
	}
	else if (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
	{
		if (auto_taking_off_speed > 0)
		{
			auto_taking_off_speed -= 1;
		}
		else
		{
			auto_taking_off_speed = 0;
		}
	}
	else if (flag.auto_take_off_land == AUTO_LAND)
	{
		auto_taking_off_speed = -35 - fs.speed_set_h[Z];
	}
}


_PID_arg_st alt_arg_2;
_PID_val_st alt_val_2;

/*高度环PID参数初始化*/
void Alt_2level_PID_Init()
{
	alt_arg_2.kp = DY_Parame.set.pid_alt_2level[KP];
	alt_arg_2.ki = DY_Parame.set.pid_alt_2level[KI];
	alt_arg_2.kd_ex = 0.00f;
	alt_arg_2.kd_fb = DY_Parame.set.pid_alt_2level[KD];
	alt_arg_2.k_ff = 0.0f;

}

void Alt_2level_Ctrl(float dT_s)
{
	Auto_Take_Off_Land_Task();

/***************OpenMv控制***************/
	if(DY_Debug_Height_Mode == 1)
	{
		fs.speed_set_h[Z] = dy_height;
	}
/*****************************************************/

	fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + auto_taking_off_speed;

	loc_ctrl_2.fb[Z] = wcz_hei_fus.out;

	if(fs.alt_ctrl_speed_set != 0)
	{
		flag.ct_alt_hold = 0;
	}
	else
	{
		if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<20)
		{
			flag.ct_alt_hold = 1;
		}
	}

	if(flag.taking_off == 1)
	{
		if(flag.ct_alt_hold == 1)		//定高悬停		flag.ct_alt_hold标志位由程序控制
		{
			PID_calculate( dT_s,            //周期（单位：秒）
						0,				//前馈值
						loc_ctrl_2.exp[Z],				//期望值（设定值）
						loc_ctrl_2.fb[Z],			//反馈值（）
						&alt_arg_2, //PID参数结构体
						&alt_val_2,	//PID数据结构体
						100,//积分误差限幅
						0			//integration limit，积分限幅
						 );		//输出==>alt_val2.out
		}
		else
		{
			loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z] + alt_val_2.err;
		}
	}
	else
	{
		loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
		alt_val_2.out = 0;

	}

	alt_val_2.out  = LIMIT(alt_val_2.out,-150,150);
}

_PID_arg_st alt_arg_1;
_PID_val_st alt_val_1;

/*高度速度环PID参数初始化*/
void Alt_1level_PID_Init()
{
	alt_arg_1.kp = DY_Parame.set.pid_alt_1level[KP];
	alt_arg_1.ki = DY_Parame.set.pid_alt_1level[KI];
	alt_arg_1.kd_ex = 0.00f;
	alt_arg_1.kd_fb = DY_Parame.set.pid_alt_1level[KD];
	alt_arg_1.k_ff = 0.0f;

}

//static u8 thr_start_ok;
static float err_i_comp;
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	out_en = (flag.taking_off != 0) ? 1 : 0;

	loc_ctrl_1.exp[Z] = fs.alt_ctrl_speed_set + alt_val_2.out;

	loc_ctrl_1.fb[Z] = wcz_spe_fus.out;

	PID_calculate( dT_s,            //周期（单位：秒）
					0,				//前馈值
					loc_ctrl_1.exp[Z],				//期望值（设定值）
					loc_ctrl_1.fb[Z] ,			//反馈值（）
					&alt_arg_1, //PID参数结构体
					&alt_val_1,	//PID数据结构体
					100,//积分误差限幅
					(THR_INTE_LIM *10 - err_i_comp)*out_en			//integration limit，积分限幅
					 );

	if(flag.taking_off == 1)
	{
		LPF_1_(1.0f,dT_s,THR_START *10,err_i_comp);//err_i_comp = THR_START *10;
	}
	else
	{
		err_i_comp = 0;
	}

	alt_val_1.out = LIMIT(alt_val_1.out,-err_i_comp,MAX_THR *10);

	loc_ctrl_1.out[Z] = out_en *FINAL_P *(alt_val_1.out + err_i_comp - 0.2f *imu_data.w_acc[Z]);

	mc.ct_val_thr = loc_ctrl_1.out[Z];
}

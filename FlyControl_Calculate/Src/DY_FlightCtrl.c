/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞行控制
**********************************************************************************/
#include "DY_FlightCtrl.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "DY_MotionCal.h"
#include "DY_AttCtrl.h"
#include "DY_LocCtrl.h"
#include "DY_AltCtrl.h"
#include "DY_MotorCtrl.h"
#include "Drv_led.h"
#include "DY_RC.h"
#include "Drv_vl53l0x.h"
#include "DY_Flight_Log.h"

/*PID参数初始化*/
void All_PID_Init(void)
{
	/*姿态控制，角速度PID初始化*/
	Att_1level_PID_Init();

	/*姿态控制，角度PID初始化*/
	Att_2level_PID_Init();

	/*高度控制，高度速度PID初始化*/
	Alt_1level_PID_Init();

	/*高度控制，高度PID初始化*/
	Alt_2level_PID_Init();

	/*位置速度控制PID初始化*/
	Loc_1level_PID_Init();

}

/*一键起飞*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{
		flag.fly_ready = 1;
		if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF;
			//解锁、起飞
			flag.taking_off = 1;
		}
	}
}

/*一键降落*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

_flight_state_st fs;

s16 flying_cnt, ld_delay_cnt, landing_cnt, autolanding_cnt;

/*降落检测*/
extern u16 ref_tof_height;
void land_discriminat(s16 dT_ms)
{
	if (flag.auto_take_off_land == AUTO_LAND)
	{
		/*自动模式，激光高度低于低于140mm阈值即可锁定*/
		if (tof_height_mm > 0 && tof_height_mm < 140)
		{
			if (autolanding_cnt < 200)
			{
				autolanding_cnt += dT_ms;
			}
			else
			{
				flag.fly_ready = 0;
				/*手动模式交换*/
				DY_Debug_Mode = 0;
				DY_Debug_Height_Mode = 0;
				DY_Debug_Yaw_Mode = 0;
			}
		}
		else
		{
			autolanding_cnt = 0;
		}
	}
	else if (DY_Debug_Height_Mode == 0)
	{
		/*手动模式，摇杆油门小于阈值，垂直方向加速度小于阈值，保持200ms，开始下一次判断*/
		if (fs.speed_set_h_norm[Z] < 0.12f && imu_data.w_acc[Z] < 200)
		{
			if (ld_delay_cnt > 0) //200ms
			{
				ld_delay_cnt -= dT_ms;
			}
		}
		else
		{
			ld_delay_cnt = 200;
		}

		/*手动模式，摇杆油门始终小于阈值，电机油门最终输出量小于250，在解锁状态中，没有在手动解锁上锁过程中，持续200ms，认为着陆，然后上锁*/
		if(ld_delay_cnt <= 0 && flag.thr_low)
		{
			/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续200ms，认为着陆，然后上锁*/
			if (mc.ct_val_thr < 250 && flag.fly_ready == 1 && flag.locking != 2) //ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
			{
				if (landing_cnt < 200)
				{
					landing_cnt += dT_ms;
				}
				else
				{
					flag.fly_ready = 0;
				}
			}
			else
			{
				landing_cnt = 0;
			}
		}
		else
		{
			landing_cnt = 0;
		}
	}
}

/*飞行状态任务*/
/***************PIT、ROL控制变量初始化+OpenMv***************/
s16 dy_pit = 0,dy_rol = 0;
/***********************************************************/
void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim;
	/*设置油门摇杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;     //-1.035~1.035	油门归一值
	fs.speed_set_h_norm_lpf[Z] += 0.2f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);

	/*推油门起飞*/
	if(flag.fly_ready)
	{
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1)
		{
			flag.taking_off = 1;
		}
	}

	if(flag.taking_off)
	{
		if(flying_cnt<1000)     //1s
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;
		}

		if(fs.speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP;
		}
		else
		{
			/*设置下降速度*/
			fs.speed_set_h[Z]  = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW;
		}
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}

	float speed_set_tmp[2];

	/*速度设定量，正负参考DY坐标参考方向*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);

	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);

	max_speed_lim = MAX_SPEED;

	if(switchs.dy_pmw3901_on)		//使用光流模块
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}

	speed_set_tmp[X] = max_speed_lim *fs.speed_set_h_norm_lpf[X];
	speed_set_tmp[Y] = max_speed_lim *fs.speed_set_h_norm_lpf[Y];

	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],max_speed_lim,fs.speed_set_h_cms);

/***************OpenMv控制模式***************/
	if(DY_Debug_Mode == 1)
	{
		fs.speed_set_h[X] = dy_pit;
		fs.speed_set_h[Y] = dy_rol;
	}
	else
	{
		fs.speed_set_h[X] = fs.speed_set_h_cms[X];
		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];
	}
/******************************************************/

	/*调用检测着陆的函数*/
	land_discriminat(dT_ms);

	/*激光锁定*/
	if (switchs.tof_on == 0)
	{
		if (flag.flying)
		{
			flag.auto_take_off_land = AUTO_LAND;
		}
		else
		{
			flag.fly_ready = 0;
		}
	}

	/*倾斜过大上锁*/
	if (imu_data.z_vec[Z] < 0.70f) //40度
	{
		flag.fly_ready = 0;
	}

	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}

	/*复位重力方向时，认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_ok = 0;
		WCZ_Data_Reset();
	}
	else if(imu_state.G_reset == 0)
	{
		if(flag.sensor_ok == 0)
		{
			flag.sensor_ok = 1;
			DY_DT_SendString("IMU OK!",sizeof("IMU OK!"));
		}
	}

	/*飞行状态复位*/
	if(flag.fly_ready == 0)
	{
		autolanding_cnt = 0;
		flag.flying = 0;
		ld_delay_cnt = 200;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
	}
}

static void LED_Switch()
{
	if( (LED_state != 0 && LED_state <= 115) )
	{
		return;
	}

	if(flag.flight_mode == ATT_STAB)
	{
		if(flag.fly_ready)
		{
			LED_state = 131;
		}
		else
		{
			LED_state = 121;
		}
	}
	else if(flag.flight_mode == LOC_HOLD)
	{
		if(flag.fly_ready)
		{
			LED_state = 132;
		}
		else
		{
			LED_state = 122;
		}
	}
	else if(flag.flight_mode == RETURN_HOME)
	{
		if(flag.fly_ready)
		{
			LED_state = 133;
		}
		else
		{
			LED_state = 123;
		}
	}

}

void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;

    if(sens_hd_check.dy_pmw3901_ok)     //ATK-PMW3901光流模块
    {
      if(flag.flight_mode == LOC_HOLD)
      {
        if(switchs.dy_pmw3901_on == 0)
        {
          LED_state = 14 ;      //切换指示触发2下（2闪蓝）
        }
        switchs.dy_pmw3901_on = 1;
      }
      else
      {
        if(switchs.dy_pmw3901_on)
        {
          LED_state =  24 ;     //切换指示触发1下（2闪红）
        }
        switchs.dy_pmw3901_on = 0;
      }
    }
    else
    {
      switchs.dy_pmw3901_on = 0;
    }

	if(sens_hd_check.tof_ok)        //TOF模块
	{
		if(tof_height_mm<1900)
		{
			if(switchs.tof_on == 0)
			{
				LED_state = 14 ;        //切换指示触发1下（2闪蓝）
			}
			switchs.tof_on = 1;
		}
		else
		{
			if(switchs.tof_on )
			{
				LED_state = 24 ;        //切换指示触发1下（2闪红）
			}
			switchs.tof_on = 0;
		}
	}
	else
	{
		switchs.tof_on = 0;
	}
}

u8 DY_Debug_Mode = 0;   //启用OpenMv控制
u8 DY_Debug_Height_Mode = 0;
u8 DY_Debug_Yaw_Mode = 0;
u32 DY_Task_ExeTime = 0;
u8 DY_OpenMV_Flag = 0;
void Flight_Mode_Set(u8 dT_ms)
{
	LED_Switch();
	flag.flight_mode = LOC_HOLD;					//定高悬停

	if(CH_N[AUX1]<-200)
	{
		MAP_UARTCharPut(UART4_BASE, 'H');			//OpenMv开始工作
	}
	else if(CH_N[AUX1]<200)
	{

	}
	else
	{
		one_key_land();		                		//一键降落
	}

	if(CH_N[AUX2] > 200)							//一键起飞
	{
		if (flag.auto_take_off_land != AUTO_LAND)	//降落时屏蔽
		{
			if (flag.taking_off == 0)				//非起飞状态
			{
				if (DY_Debug_Height_Mode == 0)		//仅调用一次
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;
					DY_Debug_Height_Mode = 1;
					DY_Debug_Mode = 1;
					DY_Debug_Yaw_Mode = 1;
					one_key_take_off();
				}
			}
			else									//起飞状态
			{
				if (DY_OpenMV_Flag == 0)			//OpenMv初始化前执行定高
				{
					if (ref_tof_height >= 70 || wcz_hei_fus.out >= 70)
					{
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						DY_Task_ExeTime += dT_ms;
					}
					else
					{
						if (switchs.tof_on)			//防止高度不符,激光工作时继续起飞
						{
							flag.auto_take_off_land = AUTO_TAKE_OFF;
						}
						else						//激光出错,立即降落!
						{
							flag.auto_take_off_land = AUTO_LAND;
						}
						DY_Task_ExeTime = 0;
					}
					if(DY_Task_ExeTime >= 1000)		//满足定高一秒
					{
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						DY_OpenMV_Flag = 1;
					}
				}
				else
				{
					MAP_UARTCharPut(UART4_BASE, 'H');	//OpenMv开始工作
				}
			}
		}
	}
	else if (CH_N[AUX2] > -200) 						//初始化一键起飞
	{
		if (flag.taking_off == 0)
			{
				//初始化
				dy_flag.stop = 1;
				DY_Debug_Mode = 0;
				DY_Debug_Height_Mode = 0;
				DY_Debug_Yaw_Mode = 0;
				DY_Task_ExeTime = 0;
				DY_OpenMV_Flag = 0;
			}
	}
	else												//强制锁定
	{
		if (flag.flying)
		{
			flag.auto_take_off_land = AUTO_LAND;
		}
		else
		{
			flag.fly_ready = 0;
		}
	}


}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/

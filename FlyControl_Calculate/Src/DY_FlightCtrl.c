/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * ����    �����е��
 * ����    ��www.gototi.com
 * ����    �����п���
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

/*PID������ʼ��*/
void All_PID_Init(void)
{
	/*��̬���ƣ����ٶ�PID��ʼ��*/
	Att_1level_PID_Init();

	/*��̬���ƣ��Ƕ�PID��ʼ��*/
	Att_2level_PID_Init();

	/*�߶ȿ��ƣ��߶��ٶ�PID��ʼ��*/
	Alt_1level_PID_Init();

	/*�߶ȿ��ƣ��߶�PID��ʼ��*/
	Alt_2level_PID_Init();

	/*λ���ٶȿ���PID��ʼ��*/
	Loc_1level_PID_Init();

}

/*һ�����*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{
		flag.fly_ready = 1;
		if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF;
			//���������
			flag.taking_off = 1;
		}
	}
}

/*һ������*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

_flight_state_st fs;

s16 flying_cnt, ld_delay_cnt, landing_cnt, autolanding_cnt;

/*������*/
extern u16 ref_tof_height;
void land_discriminat(s16 dT_ms)
{
	if (flag.auto_take_off_land == AUTO_LAND)
	{
		/*�Զ�ģʽ������߶ȵ��ڵ���140mm��ֵ��������*/
		if (tof_height_mm > 0 && tof_height_mm < 140)
		{
			if (autolanding_cnt < 200)
			{
				autolanding_cnt += dT_ms;
			}
			else
			{
				flag.fly_ready = 0;
				/*�ֶ�ģʽ����*/
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
		/*�ֶ�ģʽ��ҡ������С����ֵ����ֱ������ٶ�С����ֵ������200ms����ʼ��һ���ж�*/
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

		/*�ֶ�ģʽ��ҡ������ʼ��С����ֵ������������������С��250���ڽ���״̬�У�û�����ֶ��������������У�����200ms����Ϊ��½��Ȼ������*/
		if(ld_delay_cnt <= 0 && flag.thr_low)
		{
			/*�������������С��250����û�����ֶ��������������У�����200ms����Ϊ��½��Ȼ������*/
			if (mc.ct_val_thr < 250 && flag.fly_ready == 1 && flag.locking != 2) //ABS(wz_spe_f1.out <20 ) //��Ӧ�� �����ٶ��������ٶ�С����20����ÿ�롣
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

/*����״̬����*/
/***************PIT��ROL���Ʊ�����ʼ��+OpenMv***************/
s16 dy_pit = 0,dy_rol = 0;
/***********************************************************/
void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim;
	/*��������ҡ����*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;     //-1.035~1.035	���Ź�һֵ
	fs.speed_set_h_norm_lpf[Z] += 0.2f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);

	/*���������*/
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
			/*��ɺ�1�룬��Ϊ�Ѿ��ڷ���*/
			flag.flying = 1;
		}

		if(fs.speed_set_h_norm[Z]>0)
		{
			/*���������ٶ�*/
			fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP;
		}
		else
		{
			/*�����½��ٶ�*/
			fs.speed_set_h[Z]  = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW;
		}
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}

	float speed_set_tmp[2];

	/*�ٶ��趨���������ο�DY����ο�����*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);

	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);

	max_speed_lim = MAX_SPEED;

	if(switchs.dy_pmw3901_on)		//ʹ�ù���ģ��
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);
	}

	speed_set_tmp[X] = max_speed_lim *fs.speed_set_h_norm_lpf[X];
	speed_set_tmp[Y] = max_speed_lim *fs.speed_set_h_norm_lpf[Y];

	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],max_speed_lim,fs.speed_set_h_cms);

/***************OpenMv����ģʽ***************/
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

	/*���ü����½�ĺ���*/
	land_discriminat(dT_ms);

	/*��������*/
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

	/*��б��������*/
	if (imu_data.z_vec[Z] < 0.70f) //40��
	{
		flag.fly_ready = 0;
	}

	/*У׼�У���λ��������*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}

	/*��λ��������ʱ����Ϊ������ʧЧ*/
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

	/*����״̬��λ*/
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

    if(sens_hd_check.dy_pmw3901_ok)     //ATK-PMW3901����ģ��
    {
      if(flag.flight_mode == LOC_HOLD)
      {
        if(switchs.dy_pmw3901_on == 0)
        {
          LED_state = 14 ;      //�л�ָʾ����2�£�2������
        }
        switchs.dy_pmw3901_on = 1;
      }
      else
      {
        if(switchs.dy_pmw3901_on)
        {
          LED_state =  24 ;     //�л�ָʾ����1�£�2���죩
        }
        switchs.dy_pmw3901_on = 0;
      }
    }
    else
    {
      switchs.dy_pmw3901_on = 0;
    }

	if(sens_hd_check.tof_ok)        //TOFģ��
	{
		if(tof_height_mm<1900)
		{
			if(switchs.tof_on == 0)
			{
				LED_state = 14 ;        //�л�ָʾ����1�£�2������
			}
			switchs.tof_on = 1;
		}
		else
		{
			if(switchs.tof_on )
			{
				LED_state = 24 ;        //�л�ָʾ����1�£�2���죩
			}
			switchs.tof_on = 0;
		}
	}
	else
	{
		switchs.tof_on = 0;
	}
}

u8 DY_Debug_Mode = 0;   //����OpenMv����
u8 DY_Debug_Height_Mode = 0;
u8 DY_Debug_Yaw_Mode = 0;
u32 DY_Task_ExeTime = 0;
u8 DY_OpenMV_Flag = 0;
void Flight_Mode_Set(u8 dT_ms)
{
	LED_Switch();
	flag.flight_mode = LOC_HOLD;					//������ͣ

	if(CH_N[AUX1]<-200)
	{
		MAP_UARTCharPut(UART4_BASE, 'H');			//OpenMv��ʼ����
	}
	else if(CH_N[AUX1]<200)
	{

	}
	else
	{
		one_key_land();		                		//һ������
	}

	if(CH_N[AUX2] > 200)							//һ�����
	{
		if (flag.auto_take_off_land != AUTO_LAND)	//����ʱ����
		{
			if (flag.taking_off == 0)				//�����״̬
			{
				if (DY_Debug_Height_Mode == 0)		//������һ��
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;
					DY_Debug_Height_Mode = 1;
					DY_Debug_Mode = 1;
					DY_Debug_Yaw_Mode = 1;
					one_key_take_off();
				}
			}
			else									//���״̬
			{
				if (DY_OpenMV_Flag == 0)			//OpenMv��ʼ��ǰִ�ж���
				{
					if (ref_tof_height >= 70 || wcz_hei_fus.out >= 70)
					{
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						DY_Task_ExeTime += dT_ms;
					}
					else
					{
						if (switchs.tof_on)			//��ֹ�߶Ȳ���,���⹤��ʱ�������
						{
							flag.auto_take_off_land = AUTO_TAKE_OFF;
						}
						else						//�������,��������!
						{
							flag.auto_take_off_land = AUTO_LAND;
						}
						DY_Task_ExeTime = 0;
					}
					if(DY_Task_ExeTime >= 1000)		//���㶨��һ��
					{
						flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
						DY_OpenMV_Flag = 1;
					}
				}
				else
				{
					MAP_UARTCharPut(UART4_BASE, 'H');	//OpenMv��ʼ����
				}
			}
		}
	}
	else if (CH_N[AUX2] > -200) 						//��ʼ��һ�����
	{
		if (flag.taking_off == 0)
			{
				//��ʼ��
				dy_flag.stop = 1;
				DY_Debug_Mode = 0;
				DY_Debug_Height_Mode = 0;
				DY_Debug_Yaw_Mode = 0;
				DY_Task_ExeTime = 0;
				DY_OpenMV_Flag = 0;
			}
	}
	else												//ǿ������
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

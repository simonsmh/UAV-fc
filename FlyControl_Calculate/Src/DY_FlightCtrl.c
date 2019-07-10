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
#include "DY_OF.h"

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

/*���Ʋ����ı�����*/
void ctrl_parameter_change_task()
{
	if(0)
	{
		Set_Att_2level_Ki(0);

	}
	else
	{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{

			Set_Att_1level_Ki(2);
		}
		else
		{

			Set_Att_1level_Ki(1);
		}

		Set_Att_2level_Ki(1);
	}
}

/*һ�����������ޣ�*/
void one_key_roll()
{
			if(flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
			{
				if(rolling_flag.roll_mode==0)
				{
					rolling_flag.roll_mode = 1;
				}
			}
}

// static u16 one_key_taof_start;
// /*һ�����������Ҫ����Ϊ�ӳ٣�*/
// void one_key_take_off_task(u16 dt_ms)
// {
// 	if(one_key_taof_start != 0)
// 	{
// 		one_key_taof_start += dt_ms;
// 		if(one_key_taof_start > 1400 && flag.motor_preparation == 1)		//1400*10=14000ms=14s
// 		{
// 			one_key_taof_start = 0;
// 				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
// 				{
// 					flag.auto_take_off_land = AUTO_TAKE_OFF;
// 					//���������
// 					flag.taking_off = 1;
// 				}
// 		}
// 	}
// }

/*һ�����*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{
		// one_key_taof_start = 1;
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

s16 flying_cnt,landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*������*/

static s16 ld_delay_cnt ;
void land_discriminat(s16 dT_ms)
{
	/*���Ź�һֵС��0.1���Ҵ�ֱ������ٶ�С����ֵ  ���������Զ�����*/
	if((fs.speed_set_h_norm[Z] < 0.1f && imu_data.w_acc[Z]<200) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(ld_delay_cnt>0)		//200ms
		{
			ld_delay_cnt -= dT_ms;
		}
	}
	else
	{
		ld_delay_cnt = 200;
	}

	/*�����ǣ���������������ţ�����Ҫ�ȴ�ֱ������ٶ�С��200cm/s2 ����200ms�ſ�ʼ���*/
	if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*�������������С��250����û�����ֶ��������������У�����1.5�룬��Ϊ��½��Ȼ������*/
		if(mc.ct_val_thr<250 && flag.fly_ready == 1 && flag.locking != 2)//ABS(wz_spe_f1.out <20 ) //��Ӧ�� �����ٶ��������ٶ�С����20����ÿ�롣
		{
			if(landing_cnt<1500)
			{
				landing_cnt += dT_ms;
			}
			else
			{

				flying_cnt = 0;
				flag.taking_off = 0;

				landing_cnt =0;
				flag.fly_ready =0;

				flag.flying = 0;
			}
		}
		else
		{
			landing_cnt = 0;
		}
	}
	else
	{
		landing_cnt  = 0;
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

	if(switchs.of_flow_on || switchs.dy_pmw3901_on)		//ʹ�ù���ģ��
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

	/*��б��������*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z]<0.25f)//75��  ////////////////////////////////////////*************************** ��б�������������á�
		{

			flag.fly_ready = 0;
		}
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
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;

		//��λ�ں�
		if(flag.taking_off == 0)
		{

		}
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

static u8 of_light_ok;
static u16 of_light_delay;
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

	if(sens_hd_check.of_ok)     //��������ģ��
	{
		if(OF_LIGHT>20 || flag.flying == 0)     //�������ȴ���20�����ڷ���֮ǰ����Ϊ�������ã��ж������ӳ�ʱ��Ϊ1��
		{
			if(of_light_delay<1000)
			{
				of_light_delay += dT_ms;
			}
			else
			{
				of_light_ok = 1;
			}
		}
		else
		{
			of_light_delay =0;
			of_light_ok = 0;
		}

		if(OF_ALT<1900 && flag.flight_mode == LOC_HOLD)
		{
			if(of_light_ok)
			{
				if(switchs.of_flow_on == 0)
				{
					LED_state = 13 ;        //�л�ָʾ����1�£�1������
				}
				switchs.of_flow_on = 1;
			}
			else
			{
				if(switchs.of_flow_on )
				{
					LED_state = 23 ;        //�л�ָʾ����1�£�1���죩
				}
				switchs.of_flow_on = 0;
			}

			if(switchs.of_tof_on == 0)
			{
				LED_state = 14 ;        //�л�ָʾ����1�£�2������
			}
			switchs.of_tof_on = 1;
		}
		else
		{
			if(switchs.of_tof_on )
			{
				LED_state =  24 ;       //�л�ָʾ����1�£�2���죩
			}
			switchs.of_tof_on = 0;


			switchs.of_flow_on = 0;
		}
	}
	else
	{
		switchs.of_flow_on = switchs.of_tof_on = 0;
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

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;

u8 DY_Debug_Mode = 0;   //����OpenMv����
u8 DY_Debug_Height_Mode = 0;
u8 DY_Debug_Yaw_Mode = 0;
u32 DY_Task_ExeTime = 0;
u8 DY_CountTime_Flag = 0;
u8 DY_Land_Flag = 0;

void Flight_Mode_Set(u8 dT_ms)
{
	LED_Switch();

	if(speed_mode_old != flag.speed_mode)
	{
		speed_mode_old = flag.speed_mode;

		if(flag.speed_mode == 1)
		{
			LED_state = 13;
		}
		else if(flag.speed_mode == 2)
		{
			LED_state = 14;
		}
		else
		{
			LED_state = 15;
		}
	}

	if(CH_N[AUX1]<-200)		        //-500`-200
	{
		flag.flight_mode = ATT_STAB;		//��̬ģʽ
	}
	else if(CH_N[AUX1]<200)		//-200~200
	{
		flag.flight_mode = LOC_HOLD;		//������ͣ
	}
	else		                    //200~500
	{
		flag.flight_mode = LOC_HOLD;		//������ͣ
        one_key_land();		                //һ������
	}


	if(flight_mode_old != flag.flight_mode) //״̬�ı�
	{
		flight_mode_old = flag.flight_mode;

	}

// 	if(CH_N[AUX2]<-200)
// 	{
//       if(DY_Debug_Height_Mode==0)
//       {
//         DY_Debug_Height_Mode = 1;
//         one_key_take_off();
//         dy_height = 30;
//       }
//       else
//       {
//         if(tof_height_mm>=1200 && DY_CountTime_Flag==0)
//         {
//           dy_height = 0;
//           DY_CountTime_Flag = 1;
//         }
//         if(DY_CountTime_Flag)
//         {
//           DY_Task_ExeTime++;
//           if(DY_Task_ExeTime>=1500 && DY_Land_Flag==0)
//           {
//             DY_Land_Flag = 1;
//             one_key_land();     //һ������
//           }
// //          if(DY_Task_ExeTime>=1500 && DY_Debug_Mode==0)
// //          {
// //            DY_Debug_Mode = 1;
// //            MAP_UARTCharPut(UART5_BASE, 'H');     //OpenMv��ʼ����
// //          }
//         }
//       }
// 	}
// 	else if(CH_N[AUX2]<200)
// 	{

// 	}
// 	else
// 	{
// 		if(tof_height_mm>=1200 && DY_CountTime_Flag==0)
//         {
//           dy_height = 0;
//           DY_CountTime_Flag = 1;
//         }
//         if(DY_CountTime_Flag)
//         {
//           DY_Task_ExeTime++;
// //          if(DY_Task_ExeTime>=1500 && DY_Land_Flag==0)
// //          {
// //            DY_Land_Flag = 1;
// //            one_key_land();     //һ������
// //          }
//           if(DY_Task_ExeTime>=1500 && DY_Debug_Mode==0)
//           {
//             DY_Debug_Mode = 1;
//             MAP_UARTCharPut(UART5_BASE, 'H');     //OpenMv��ʼ����
//           }
//         }
// 	}

	if(CH_N[AUX2] > 200)
	{
		if ((DY_Debug_Height_Mode == 0) && (DY_Debug_Mode == 0))
		{
			DY_Debug_Height_Mode = 1;
			DY_Debug_Mode = 1;
			one_key_take_off();
		}
		if ((tof_height_mm >= 1000) && (DY_CountTime_Flag == 0))
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			DY_CountTime_Flag = 1;
		}
		if (DY_CountTime_Flag)
		{
			DY_Task_ExeTime += dT_ms;
			if ((DY_Task_ExeTime >= 3000) && (DY_Land_Flag == 0))
			{
				dy_pit = 20;
				dy_rol = 0;
				DY_Land_Flag = 1;
			}
			if ((DY_Task_ExeTime >= 4500) && (DY_Land_Flag == 1))
			{
				dy_pit = 0;
				dy_rol = 20;
				DY_Land_Flag = 2;
			}
			if ((DY_Task_ExeTime >= 6000) && (DY_Land_Flag == 2))
			{
				dy_pit = -20;
				dy_rol = 0;
				DY_Land_Flag = 3;
			}
			if ((DY_Task_ExeTime >= 7500) && (DY_Land_Flag == 3))
			{
				dy_pit = 0;
				dy_rol = -20;
				DY_Land_Flag = 4;
			}
			if ((DY_Task_ExeTime >= 9000) && (DY_Land_Flag == 4))
			{
				dy_pit = 0;
				dy_rol = 0;
				DY_Land_Flag = 5;
			}
			if ((DY_Task_ExeTime >= 10500) && (DY_Land_Flag == 5))
			{
				DY_Land_Flag = 6;
				one_key_land(); //һ������
			}
		}
	}
	else
	{
		flag.flight_mode = LOC_HOLD;
		if ((DY_Debug_Mode == 1) || (DY_Debug_Height_Mode == 1))
		{
			//��ʼ��
			DY_Debug_Mode = 0;
			DY_Debug_Height_Mode = 0;
			DY_Debug_Yaw_Mode = 0;
			dy_pit = 0;
			dy_rol = 0;
			dy_height = 0;
			dy_yaw = 0;
			DY_CountTime_Flag = 0;
			DY_Land_Flag = 0;
			DY_Task_ExeTime = 0;
		}
	}

}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/

#include "DY_FlightDataCal.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "Drv_ak8975.h"
#include "DY_MotionCal.h"
#include "Drv_vl53l0x.h"
#include "Drv_led.h"

void Fc_Sensor_Get()
{
	static u8 cnt;
	if(flag.start_ok)
	{
		/*��ȡ�����Ǻͼ��ٶȼ�����*/
		Drv_Icm20602_Read();

		cnt ++;
		cnt %= 20;
		if(cnt==0)
		{
			/*��ȡ�������̴���������*/
			Drv_AK8975_Read();
			/*��ȡ��ѹ������*/
			baro_height = (s32)Drv_Spl0601_Read();
		}
	}
}

extern s32 sensor_val_ref[];

static u8 reset_imu_f;
void IMU_Update_Task(u8 dT_ms)
{
        ////////////////////////////////////////////////////////////////////////
        /*���׼�����У���λ������λ��Ǻʹ����Ƹ�λ���*/
        if(flag.fly_ready)
        {
            imu_state.G_reset = imu_state.M_reset = 0;
            reset_imu_f = 0;
        }
        else
        {
            if(flag.motionless == 0)
            {

            }

            if(reset_imu_f==0 )//&& flag.motionless == 1)
            {
                imu_state.G_reset = 1;      //������λ
                sensor.gyr_CALIBRATE = 2;   //У׼�����ǣ�������
                reset_imu_f = 1;            //�Ѿ���λ��λ���
            }
        }

		/*�������������ں�����kpϵ��*/
		imu_state.gkp = 0.3f;//0.4f;

		/*�������������ں�����kiϵ��*/
		imu_state.gki = 0.002f;

		/*�������̻����ں�����kiϵ��*/
		imu_state.mkp = 0.2f;

        imu_state.M_fix_en = sens_hd_check.mag_ok;		//����������ʹ��

        /*��̬���㣬���£��ں�*/
        IMU_update(dT_ms *1e-3f, &imu_state,sensor.Gyro_rad, sensor.Acc_cmss, mag.val,&imu_data);//x3_dT_1[2] * 0.000001f
}

static s16 mag_val[3];
void Mag_Update_Task(u8 dT_ms)
{
	Mag_Get(mag_val);

	Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);
}


s32 baro_height,baro_h_offset,ref_height_get_1,ref_height_get_2,ref_height_used;
s32 baro2tof_offset,tof2baro_offset;

float baro_fix1,baro_fix2,baro_fix;

static u8 wcz_f_pause;
float wcz_acc_use;

void WCZ_Acc_Get_Task()//��С����
{
	wcz_acc_use += 0.2f *(imu_data.w_acc[Z] - wcz_acc_use);

}

u16 ref_tof_height;
static u8 baro_offset_ok,tof_offset_ok;
void WCZ_Fus_Task(u8 dT_ms)
{
	if(flag.taking_off)
	{
		baro_offset_ok = 2;
	}
	else
	{
		if(baro_offset_ok == 2)
		{
			baro_offset_ok = 0;
		}
	}

	if(baro_offset_ok >= 1)//(flag.taking_off)
	{
		ref_height_get_1 = baro_height - baro_h_offset + baro_fix  + tof2baro_offset;//��ѹ����Ը߶ȣ��л������TOF
	}
	else
	{
		if(baro_offset_ok == 0 )
		{
			baro_h_offset = baro_height;
			if(flag.sensor_ok)
			{
				baro_offset_ok = 1;
			}
		}
	}

	if((flag.flying == 0) && flag.auto_take_off_land == AUTO_TAKE_OFF	)
	{
		wcz_f_pause = 1;

		baro_fix = 0;
	}
	else
	{
		wcz_f_pause = 0;

		if(flag.taking_off == 0)
		{
			baro_fix1 = 0;
			baro_fix2 = 0;

		}
		baro_fix2 = -BARO_FIX;


		baro_fix = baro_fix1 + baro_fix2 - BARO_FIX;//+ baro_fix3;
	}

	if(sens_hd_check.tof_ok && baro_offset_ok) //TOFӲ������������ѹ�Ƽ�¼���ֵ�Ժ�
	{
		if(switchs.tof_on) //TOF������Ч
		{
			ref_tof_height = tof_height_mm/10;
			if(tof_offset_ok == 1)
			{
				ref_height_get_2 = ref_tof_height + baro2tof_offset;//TOF�ο��߶ȣ��л��������ѹ��

				ref_height_used = ref_height_get_2;

				tof2baro_offset += 0.5f *((ref_height_get_2 - ref_height_get_1) - tof2baro_offset);//��¼��ѹ���л��㣬��ѹ�Ʋ�������΢�˲�һ��
				//tof2baro_offset = ref_height_get_2 - ref_height_get_1;
			}
			else
			{
				baro2tof_offset = ref_height_get_1 - ref_tof_height ; //��¼TOF�л���

				tof_offset_ok = 1;
			}
		}
		else
		{
			tof_offset_ok = 0;

			ref_height_used = ref_height_get_1 ;
		}
	}
	else
	{
		ref_height_used = ref_height_get_1;
	}

	WCZ_Data_Calc(dT_ms,wcz_f_pause,(s32)wcz_acc_use,(s32)(ref_height_used));
}

#include "DY_AttCtrl.h"
#include "DY_Imu.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "Drv_spl06.h"
#include "DY_MotionCal.h"
#include "DY_FlightCtrl.h"
#include "DY_LocCtrl.h"
#include "DY_MotorCtrl.h"

//�ǶȻ����Ʋ���
_PID_arg_st arg_2[VEC_RPY] ;

//���ٶȻ����Ʋ���
_PID_arg_st arg_1[VEC_RPY] ;


//�ǶȻ���������
_PID_val_st val_2[VEC_RPY];

//���ٶȻ���������
_PID_val_st val_1[VEC_RPY];

/*�ǶȻ�PID������ʼ��*/
void Att_2level_PID_Init()
{
	arg_2[ROL].kp = DY_Parame.set.pid_att_2level[ROL][KP];
	arg_2[ROL].ki = DY_Parame.set.pid_att_2level[ROL][KI];
	arg_2[ROL].kd_ex = DY_Parame.set.pid_att_2level[ROL][KD];
	arg_2[ROL].kd_fb = DY_Parame.set.pid_att_2level[ROL][KD];
	arg_2[ROL].k_ff = 0.0f;

	arg_2[PIT].kp = DY_Parame.set.pid_att_2level[PIT][KP];
	arg_2[PIT].ki = DY_Parame.set.pid_att_2level[PIT][KI];
	arg_2[PIT].kd_ex = DY_Parame.set.pid_att_2level[PIT][KD];
	arg_2[PIT].kd_fb = DY_Parame.set.pid_att_2level[PIT][KD];
	arg_2[PIT].k_ff = 0.0f;

	arg_2[YAW].kp = DY_Parame.set.pid_att_2level[YAW][KP];
	arg_2[YAW].ki = DY_Parame.set.pid_att_2level[YAW][KI];
	arg_2[YAW].kd_ex = DY_Parame.set.pid_att_2level[YAW][KD];
	arg_2[YAW].kd_fb = DY_Parame.set.pid_att_2level[YAW][KD];
	arg_2[YAW].k_ff = 0.0f;
}

/*
��̬�����ʲ��ֿ��Ʋ���

arg_1_kp���������ٶ���Ӧ�ٶȣ����𵴵�ǰ���£�����Խ��Խ�á�

���ԣ����Խ���arg_1_kp������arg_1_kd��

������arg_1_kd�Ѿ����������𵴣���Ҫ��kp��kdͬʱ��С��
*/
// #define CTRL_1_KI_START 0.f

/*���ٶȻ�PID������ʼ��*/
void Att_1level_PID_Init()
{
	arg_1[ROL].kp = DY_Parame.set.pid_att_1level[ROL][KP];
	arg_1[ROL].ki = DY_Parame.set.pid_att_1level[ROL][KI];
	arg_1[ROL].kd_ex = 0.005f   ;
	arg_1[ROL].kd_fb = DY_Parame.set.pid_att_1level[ROL][KD];
	arg_1[ROL].k_ff = 0.0f;

	arg_1[PIT].kp = DY_Parame.set.pid_att_1level[PIT][KP];
	arg_1[PIT].ki = DY_Parame.set.pid_att_1level[PIT][KI];
	arg_1[PIT].kd_ex = 0.005f   ;
	arg_1[PIT].kd_fb = DY_Parame.set.pid_att_1level[PIT][KD];
	arg_1[PIT].k_ff = 0.0f;

	arg_1[YAW].kp = DY_Parame.set.pid_att_1level[YAW][KP];
	arg_1[YAW].ki = DY_Parame.set.pid_att_1level[YAW][KI];
	arg_1[YAW].kd_ex = 0.00f   ;
	arg_1[YAW].kd_fb = DY_Parame.set.pid_att_1level[YAW][KD];
	arg_1[YAW].k_ff = 0.00f;

#if (MOTOR_ESC_TYPE == 2)		//��ˢ�����ɲ��
	#define DIFF_GAIN 0.3f
//	arg_1[ROL].kd_ex = arg_1[ROL].kd_ex *DIFF_GAIN;
//	arg_1[PIT].kd_ex = arg_1[PIT].kd_ex *DIFF_GAIN;
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#elif (MOTOR_ESC_TYPE == 1)		//��ˢ�������ɲ��
	#define DIFF_GAIN 1.0f
//	arg_1[ROL].kd_ex = arg_1[ROL].kd_ex *DIFF_GAIN;
//	arg_1[PIT].kd_ex = arg_1[PIT].kd_ex *DIFF_GAIN;
	arg_1[ROL].kd_fb = arg_1[ROL].kd_fb *DIFF_GAIN;
	arg_1[PIT].kd_fb = arg_1[PIT].kd_fb *DIFF_GAIN;
#endif
}

void Set_Att_1level_Ki(u8 mode)
{
	if(mode == 0)
	{
		arg_1[ROL].ki = arg_1[PIT].ki = 0;
	}
	else
	{
		arg_1[ROL].ki = DY_Parame.set.pid_att_1level[ROL][KI];
		arg_1[PIT].ki = DY_Parame.set.pid_att_1level[PIT][KI];
	}
}

_att_2l_ct_st att_2l_ct;

static s32 max_yaw_speed,set_yaw_av_tmp;

#define POS_V_DAMPING 0.02f
static float exp_rol_tmp,exp_pit_tmp;

/***************YAW���Ʊ�����ʼ��***************/
float dy_yaw = 0.0f;

/*�ǶȻ�����*/
void Att_2level_Ctrl(float dT_s,s16 *CH_N)
{
	/*����΢��*/
	exp_rol_tmp = - loc_ctrl_1.out[Y];
	exp_pit_tmp = - loc_ctrl_1.out[X];

	if(ABS(exp_rol_tmp + att_2l_ct.exp_rol_adj) < 5)
	{
		att_2l_ct.exp_rol_adj += 0.1f *exp_rol_tmp *dT_s;
		att_2l_ct.exp_rol_adj = LIMIT(att_2l_ct.exp_rol_adj,-1,1);
	}

	if(ABS(exp_pit_tmp + att_2l_ct.exp_pit_adj) < 5)
	{
		att_2l_ct.exp_pit_adj += 0.1f *exp_pit_tmp *dT_s;
		att_2l_ct.exp_pit_adj = LIMIT(att_2l_ct.exp_pit_adj,-1,1);
	}

	/*�����ο�DY����ο�����*/
	att_2l_ct.exp_rol = exp_rol_tmp + att_2l_ct.exp_rol_adj + POS_V_DAMPING *imu_data.h_acc[Y];		//exp_rol_tmp����Ҫ����
	att_2l_ct.exp_pit = exp_pit_tmp + att_2l_ct.exp_pit_adj + POS_V_DAMPING *imu_data.h_acc[X];

	/*�����Ƕ��޷�*/
	att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol,-MAX_ANGLE,MAX_ANGLE);
	att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit,-MAX_ANGLE,MAX_ANGLE);

	max_yaw_speed = MAX_SPEED_YAW;

	/*ҡ����ת��ΪYAW�������ٶ�*/
	att_1l_ct.set_yaw_speed = (s32)(0.0023f *my_deadzone(CH_N[CH_YAW],0,65) *max_yaw_speed);
	/*���YAW���ٶ��޷�*/
	set_yaw_av_tmp = LIMIT(att_1l_ct.set_yaw_speed ,-max_yaw_speed,max_yaw_speed);

	/*û����ɣ���λ*/
	if(flag.taking_off==0)//if(flag.locking)
	{
		att_2l_ct.exp_rol = att_2l_ct.exp_pit = set_yaw_av_tmp = 0;
		att_2l_ct.exp_yaw = att_2l_ct.fb_yaw;
	}

    /***************OpenMv����***************/
	if(DY_Debug_Yaw_Mode == 1)
	{
		att_2l_ct.exp_yaw += dy_yaw*dT_s;
	}
	else
	{
		att_2l_ct.exp_yaw += set_yaw_av_tmp *dT_s;
	}

	/*����Ϊ+-180��*/
	if(att_2l_ct.exp_yaw<-180) att_2l_ct.exp_yaw += 360;
	else if(att_2l_ct.exp_yaw>180) att_2l_ct.exp_yaw -= 360;

	/*����YAW�Ƕ����*/
	att_2l_ct.yaw_err = (att_2l_ct.exp_yaw - att_2l_ct.fb_yaw);
	/*����Ϊ+-180��*/
	if(att_2l_ct.yaw_err<-180) att_2l_ct.yaw_err += 360;
	else if(att_2l_ct.yaw_err>180) att_2l_ct.yaw_err -= 360;

	/*�����������*/
	if(att_2l_ct.yaw_err>90)
	{
		if(set_yaw_av_tmp>0)
		{
			set_yaw_av_tmp = 0;
		}
	}
	else if(att_2l_ct.yaw_err<-90)
	{
		if(set_yaw_av_tmp<0)
		{
			set_yaw_av_tmp = 0;
		}
	}

	/*��ֵ�����Ƕ�ֵ*/
	att_2l_ct.fb_yaw = imu_data.yaw;
	att_2l_ct.fb_rol = imu_data.rol;
	att_2l_ct.fb_pit = imu_data.pit;


	PID_calculate( dT_s,            //���ڣ���λ���룩
										0 ,				//ǰ��ֵ
										att_2l_ct.exp_rol ,				//����ֵ���趨ֵ��
										att_2l_ct.fb_rol ,			//����ֵ����
										&arg_2[ROL], //PID�����ṹ��
										&val_2[ROL],	//PID���ݽṹ��
	                  5,//��������޷�
										5 *flag.taking_off			//integration limit�������޷�
										 )	;

	PID_calculate( dT_s,            //���ڣ���λ���룩
										0 ,				//ǰ��ֵ
										att_2l_ct.exp_pit ,				//����ֵ���趨ֵ��
										att_2l_ct.fb_pit ,			//����ֵ����
										&arg_2[PIT], //PID�����ṹ��
										&val_2[PIT],	//PID���ݽṹ��
	                  5,//��������޷�
										5 *flag.taking_off		//integration limit�������޷�
										 )	;

	PID_calculate( dT_s,            //���ڣ���λ���룩
										0 ,				//ǰ��ֵ
										att_2l_ct.yaw_err ,				//����ֵ���趨ֵ��
										0 ,			//����ֵ����
										&arg_2[YAW], //PID�����ṹ��
										&val_2[YAW],	//PID���ݽṹ��
	                  5,//��������޷�
										5 *flag.taking_off			//integration limit�������޷�
										 )	;

}

_att_1l_ct_st att_1l_ct;
static float ct_val[4];
/*���ٶȻ�����*/
void Att_1level_Ctrl(float dT_s)
{
	////////////////�ı���Ʋ���������С���������ڣ�////////////////////////
	if (flag.auto_take_off_land == AUTO_TAKE_OFF)
	{
		Set_Att_1level_Ki(0);
	}
	else
	{
		Set_Att_1level_Ki(1);
	}

	/*Ŀ����ٶȸ�ֵ*/
     for(u8 i = 0;i<3;i++)
    {
        att_1l_ct.exp_angular_velocity[i] = val_2[i].out;
    }

    /*Ŀ����ٶ��޷�*/
    att_1l_ct.exp_angular_velocity[ROL] = LIMIT(att_1l_ct.exp_angular_velocity[ROL],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
    att_1l_ct.exp_angular_velocity[PIT] = LIMIT(att_1l_ct.exp_angular_velocity[PIT],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);


	/*�������ٶȸ�ֵ*/
	att_1l_ct.fb_angular_velocity[ROL] =  sensor.Gyro_deg[X];
	att_1l_ct.fb_angular_velocity[PIT] = -sensor.Gyro_deg[Y];
	att_1l_ct.fb_angular_velocity[YAW] = -sensor.Gyro_deg[Z];


	/*PID����*/
     for(u8 i = 0;i<3;i++)
     {
            PID_calculate( dT_s,            //���ڣ���λ���룩
                                            0,				//ǰ��ֵ
                                            att_1l_ct.exp_angular_velocity[i],				//����ֵ���趨ֵ��
                                            att_1l_ct.fb_angular_velocity[i],			//����ֵ����
                                            &arg_1[i], //PID�����ṹ��
                                            &val_1[i],	//PID���ݽṹ��
                        200,//��������޷�
                                            CTRL_1_INTE_LIM *flag.taking_off			//integration limit�����ַ����޷�
                                             )	; 		//���==>val_1[i].out


         ct_val[i] = (val_1[i].out);
     }

	/*��ֵ�����ձ�������*/
	mc.ct_val_rol = FINAL_P * ct_val[ROL];
	mc.ct_val_pit = FINAL_P * ct_val[PIT];
	mc.ct_val_yaw = FINAL_P * ct_val[YAW];
	/*������޷�*/
	mc.ct_val_rol = LIMIT(mc.ct_val_rol,-1000,1000);
	mc.ct_val_pit = LIMIT(mc.ct_val_pit,-1000,1000);
	mc.ct_val_yaw = LIMIT(mc.ct_val_yaw,-400,400);

}

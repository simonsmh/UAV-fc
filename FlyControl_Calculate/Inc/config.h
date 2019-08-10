/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CONFIG_H_
#define _CONFIG_H_

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/***************����******************/
#define ANGLE_TO_RADIAN 0.01745329f             //*0.01745 = /57.3	�Ƕ�ת����

#define DY_DT_USE_NRF24l01


#define OFFSET_AV_NUM 50

#define G_1G 4096
#define SP_EST_DRAG 1.0f
#define BARO_WIND_COMP 0.10f


#define MOTOR_ESC_TYPE 2                //2����ˢ�����ɲ���ĵ����1����ˢ�������ɲ���ĵ����0����ˢ���
#define MOTORSNUM 4

#define MAX_ANGLE     25.0f

#define MAX_SPEED_ROL 200               //�Ƕ�ÿ��
#define MAX_SPEED_PIT 200               //�Ƕ�ÿ��
#define MAX_SPEED_YAW 360               //�Ƕ�ÿ��

#define MAX_ROLLING_SPEED 1600          //�Ƕ�ÿ��

#define MAX_SPEED 500           //���ˮƽ�ٶȣ�����ÿ�� cm/s

#define MAX_Z_SPEED_UP 350              //����ÿ�� cm/s
#define MAX_Z_SPEED_DW 300              //����ÿ�� cm/s

#define MAX_EXP_XY_ACC   500            //����ÿƽ���� cm/ss

#define CTRL_1_INTE_LIM 250             //���ٶȻ������޷� �����

#define FINAL_P 			0.3f            //������������
#define ANGULAR_VELOCITY_PID_INTE_D_LIM 300/FINAL_P

#define MAX_THR_SET    85               //������Űٷֱ� %
#define THR_INTE_LIM_SET   70           //���Ż��ְٷֱ� %

#define MAX_THR       MAX_THR_SET/FINAL_P               //283.33
#define THR_INTE_LIM   THR_INTE_LIM_SET/FINAL_P

#define THR_START      40               //����������ٷֱ� %

#define BARO_FIX -0             //��ѹ�ٶȻ����������ֵ/CM����

#endif

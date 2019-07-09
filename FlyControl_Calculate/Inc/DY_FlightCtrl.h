/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * ����    �����е��
 * ����    ��www.gototi.com
 * ����    �����п���
**********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FLIGHT_CTRL_H_
#define _FLIGHT_CTRL_H_
/* Includes ------------------------------------------------------------------*/
#include "DY_FcData.h"
#include "DY_Filter.h"
#include "DY_Math.h"

/* Exported types ------------------------------------------------------------*/

enum
{
   null=0,
   takeoff,
   landing,
	
    s_up_down_2, //С���������½�2��
	
    s_yaw_pn_2,  //С��������ת2��
    b_yaw_pn_1,  //���������ת1��

    s_rol_pn_2,  //С��������2��
    b_rol_pn_1,  //���������1��
    
    s_pit_pn_2,  //С����ǰ��2��
    b_pit_pn_1,  //�����ǰ��1��

    yaw_n360,   //��ת1Ȧ
    yaw_p360,   //��ת1Ȧ

    roll_1,    //����

    pit_jump_pn_2,  //ǰ����2��
    rol_jump_pn_2,  //������2��
    
    rol_up_down_2,  //���������½�	  
    yaw_up_dowm_1,  //��ת�����½�		2
    pit_rol_pn_2,   //����ǰ������		3
		
};

typedef struct
{
	s16 alt_ctrl_speed_set;
	s16 speed_set_h[VEC_XYZ];	
	float speed_set_h_cms[VEC_XYZ];
	
	float speed_set_h_norm[VEC_XYZ];
	float speed_set_h_norm_lpf[VEC_XYZ];
	
}_flight_state_st;
extern _flight_state_st fs;
/* Exported constants --------------------------------------------------------*/
extern float wifi_selfie_mode_yaw_vlue;

extern s16 dy_pit,dy_rol;
extern u8 DY_Debug_Mode;
extern u8 DY_Debug_Height_Mode;
extern u8 DY_Debug_Yaw_Mode;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void user_fun(float dT,u8 action_num);

void All_PID_Init(void);

void one_key_take_off(void);
void one_key_land(void);

void one_key_roll(void);
void app_one_key_roll(void);
void app_one_key_roll_reset(void);
void one_key_take_off_task(u16 dt_ms);

void ctrl_parameter_change_task(void);
	
void Flight_State_Task(u8,s16 *CH_N);

void Flight_Mode_Set(u8 dT_ms);

void Swtich_State_Task(u8 dT_ms);

#endif
/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/

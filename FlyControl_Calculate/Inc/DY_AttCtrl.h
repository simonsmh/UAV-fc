/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ATT_CTRL_H
#define __ATT_CTRL_H
/* Includes ------------------------------------------------------------------*/
#include "DY_FcData.h"
#include "DY_Filter.h"
#include "DY_Math.h"
#include "DY_Pid.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	s32 set_yaw_speed;

	float exp_angular_velocity[VEC_RPY];

	float fb_angular_velocity[VEC_RPY];
}_att_1l_ct_st;
extern _att_1l_ct_st att_1l_ct;

typedef struct
{
	float yaw_err;
    float exp_rol_adj;
	float exp_pit_adj;

	float exp_rol,exp_pit,exp_yaw;
	float fb_rol,fb_pit,fb_yaw;

}_att_2l_ct_st;
extern _att_2l_ct_st att_2l_ct;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Att_1level_PID_Init(void);
void Att_2level_PID_Init(void);

void Set_Att_1level_Ki(u8 mode);

void Att_1level_Ctrl(float dT);
void Att_2level_Ctrl(float dT, s16 *CH_N);

extern _PID_arg_st arg_1[VEC_RPY] ;
extern _PID_arg_st arg_2[VEC_RPY];

extern _PID_val_st val_1[VEC_RPY];
extern _PID_val_st val_2[VEC_RPY];

#endif

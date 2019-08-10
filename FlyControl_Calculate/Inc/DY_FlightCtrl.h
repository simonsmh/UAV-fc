/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _FLIGHT_CTRL_H_
#define _FLIGHT_CTRL_H_
/* Includes ------------------------------------------------------------------*/
#include "DY_FcData.h"
#include "DY_Filter.h"
#include "DY_Math.h"

/* Exported types ------------------------------------------------------------*/
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
extern s16 dy_pit, dy_rol, dy_height;
extern float dy_yaw;
extern u8 DY_Debug_Mode;
extern u8 DY_Debug_Height_Mode;
extern u8 DY_Debug_Yaw_Mode;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void All_PID_Init(void);

void one_key_take_off(void);
void one_key_land(void);

void Flight_State_Task(u8,s16 *CH_N);

void Flight_Mode_Set(u8 dT_ms);

void Swtich_State_Task(u8 dT_ms);

#endif

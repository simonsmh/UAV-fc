#include "DY_power.h"
#include "DY_Parameter.h"
#include "DY_Filter.h"
#include "Drv_led.h"
#include "DY_Math.h"
#include "DY_DT.h"

float Plane_Votage = 0;
static float voltage_f = 30000;
static u8 voltage_init_ok;

void Power_UpdateTask(u8 dT_ms)
{
	static s16 voltage_s16;
	float cut_off_freq;

	voltage_s16 = (s16)(adc0_value[0] *8.8653f);//1.128f

	/*****voltage_init_ok=1前后，cut_off_freq不同*****/
	if(voltage_init_ok == 0)
	{
		cut_off_freq = 2.0f;

		if(voltage_f >2000 && ABS(voltage_s16 - voltage_f) <200)
		{
			voltage_init_ok = 1;
		}
	}
	else
	{
		cut_off_freq = 0.05f;
	}
	/***********************************************/

	LPF_1_(cut_off_freq,dT_ms*1e-3f,voltage_s16,voltage_f);

	Plane_Votage = voltage_f *0.001f;

	if(Plane_Votage < DY_Parame.set.lowest_power_voltage)
	{
		flag.power_state = LOWEST_POWER;
		if (LED_state > 115)
		{
			LED_state = 1;
			DY_DT_SendString("Battery Failure!", sizeof("Battery Failure!"));
		}
	}
	else if(Plane_Votage < DY_Parame.set.return_home_power_voltage)
	{
		flag.power_state = RETURN_HOME_POWER;
		if (LED_state > 115)
		{
			LED_state = 1;
			DY_DT_SendString("Out of Battery!", sizeof("Out of Battery!"));
		}
	}
	else if (Plane_Votage < DY_Parame.set.warn_power_voltage)
	{
		flag.power_state = WARN_POWER;
		if (LED_state > 115)
		{
			LED_state = 1;
			DY_DT_SendString("Low Power!", sizeof("Low Power!"));
		}
	}
	else
	{
		flag.power_state = HIGH_POWER;
	}
}

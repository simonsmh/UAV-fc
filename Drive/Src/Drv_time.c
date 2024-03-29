
#include "Drv_time.h"
#include "include.h"
#include "Drv_led.h"

volatile uint32_t sysTickUptime = 0;

void  SysTick_Configuration ( void )
{
  MAP_SysTickPeriodSet(g_ui32SysClock/1000);    //时间基准：1ms
  MAP_SysTickIntEnable();
  MAP_SysTickEnable();
}

uint32_t GetSysTime_us ( void )
{
    register uint32_t ms;
    u32 value;
    ms = sysTickUptime;
    value = ms * TICK_US + ( (g_ui32SysClock/1000) - SysTickValueGet() ) * TICK_US / (g_ui32SysClock/1000);		//ms*1000+(重装载值-当前倒计数值)/重装载值[对应1ms]*1000=SysTime_us
    return value;
}

void Delay_us ( uint32_t us )
{
    uint32_t now = GetSysTime_us();
    while ( GetSysTime_us() - now < us );
}

void Delay_ms ( uint32_t ms )
{
    while ( ms-- )
        Delay_us ( 1000 );
}

u32 systime_ms;

void sys_time()
{
	systime_ms++;
}

u32 SysTick_GetTick(void)
{
	return systime_ms;
}

void SysTick_Handler(void)
{
  sysTickUptime++;
  sys_time();
  LED_1ms_DRV();
}

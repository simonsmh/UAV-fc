#include "common.h"
#include "include.h"

int main(void)
{
  BootLoader_Setup();
  flag.start_ok = All_Init();		//进行所有设备的初始化，并将初始化结果保存
  Scheduler_Setup();			    //裸机系统，人工做了一个任务调度器
  while(1)
  {
    Scheduler_Run();			    //运行任务调度器，所有系统功能，除了中断服务函数外，都在任务调度器内完成
  }
}

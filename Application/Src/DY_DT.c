/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：数据传输
**********************************************************************************/
#include "DY_DT.h"
#include "DY_RC.h"
#include "Drv_usart.h"
#include "Drv_time.h"
#include "DY_IMU.h"
#include "Drv_icm20602.h"
#include "DY_MagProcess.h"
#include "DY_MotorCtrl.h"
#include "DY_Power.h"
#include "DY_FlightCtrl.h"
#include "include.h"
#include "DY_MotionCal.h"
#include "DY_FlightDataCal.h"
#include "Drv_vl53l0x.h"
#include "DY_LocCtrl.h"
#include "DY_AttCtrl.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR	0x05
#define SWJADDR		0xAF

s32 ParValList[PARNUM+1]; //参数列表

dt_flag_t f;				//需要发送数据的标志
u8 data_to_send[50];	    //发送数据缓存
u8 checkdata_to_send,checksum_to_send;

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void DY_DT_Send_Data(u8 *dataToSend , u8 length)
{
	Uart3_Send(data_to_send, length);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
extern float ultra_dis_lpf;
void DY_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 senser2_cnt = 50;
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 speed_cnt   = 50;
	static u8 location_cnt   = 200;

	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;

	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;

	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;

	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;

	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;

	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;

	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location += 1;
	}

	if(++cnt>200) cnt = 0;
	if(f.send_version)
	{
		f.send_version = 0;
		DY_DT_Send_Version(4,300,100,400,0);
	}
	else if(f.paraToSend < 0xffff)
	{
		DY_DT_SendParame(f.paraToSend);
		f.paraToSend = 0xffff;
	}
	else if(f.send_status)
	{
		f.send_status = 0;
		DY_DT_Send_Status(imu_data.rol,imu_data.pit,imu_data.yaw,wcz_hei_fus.out,0,flag.fly_ready);
	}
	else if(f.send_speed)
	{
		f.send_speed = 0;
		DY_DT_Send_Speed(loc_ctrl_1.fb[Y],loc_ctrl_1.fb[X],loc_ctrl_1.fb[Z]);
	}
	else if(f.send_user)
	{
		f.send_user = 0;
		DY_DT_Send_User(1, tof_height_mm, 0);
		DY_DT_Send_User(2, att_1l_ct.exp_angular_velocity[X] * 100, att_1l_ct.fb_angular_velocity[X] * 100);
		DY_DT_Send_User(3, att_1l_ct.exp_angular_velocity[Y] * 100, att_1l_ct.fb_angular_velocity[Y] * 100);
		DY_DT_Send_User(4, att_1l_ct.exp_angular_velocity[Z] * 100, att_1l_ct.fb_angular_velocity[Z] * 100);
		DY_DT_Send_User(5, att_2l_ct.exp_rol * 100, att_2l_ct.fb_rol * 100);
		DY_DT_Send_User(6, att_2l_ct.exp_pit * 100, att_2l_ct.fb_pit * 100);
		DY_DT_Send_User(7, att_2l_ct.exp_yaw * 100, att_2l_ct.fb_yaw * 100);
		DY_DT_Send_User(8, loc_ctrl_1.exp[X] * 10, loc_ctrl_1.fb[X] * 10);
		DY_DT_Send_User(9, loc_ctrl_1.exp[Y] * 10, loc_ctrl_1.fb[Y] * 10);
		DY_DT_Send_User(10, loc_ctrl_1.exp[Z] * 10, loc_ctrl_1.fb[Z] * 10);
	}
	else if(f.send_senser)
	{
		f.send_senser = 0;
		DY_DT_Send_Senser(sensor.Acc[X],sensor.Acc[Y],sensor.Acc[Z],sensor.Gyro[X],sensor.Gyro[Y],sensor.Gyro[Z],mag.val[X],mag.val[Y],mag.val[Z]);
	}
	else if(f.send_senser2)
	{
		f.send_senser2 = 0;
		DY_DT_Send_Senser2(baro_height,ref_tof_height);//原始数据
	}
	else if(f.send_rcdata)		//发送遥控器数据
	{
		f.send_rcdata = 0;
		s16 CH_GCS[CH_NUM];

		for(u8 i=0;i<CH_NUM;i++)
		{
//			if(Rc_Pwm_In[i]!=0  || Rc_Ppm_In[i] !=0)//该通道有值
            if(Rc_Pwm_In[i]!=0)//该通道有值
			{
				CH_GCS[i] = CH_N[i] + 1500;		//1000~2000
			}
			else
			{
				CH_GCS[i] = 0;
			}
		}
		DY_DT_Send_RCData(CH_GCS[2],CH_GCS[3],CH_GCS[0],CH_GCS[1],CH_GCS[4],CH_GCS[5],CH_GCS[6],CH_GCS[7],0,0);
	}
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		DY_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],0,0,0,0);
	}
	else if(f.send_power)
	{
		f.send_power = 0;
		DY_DT_Send_Power(Plane_Votage*100,0);
	}
	else if(f.send_location == 2)
	{

		f.send_location = 0;
		DY_DT_Send_Location(0,0,0 *10000000,0  *10000000,0);

	}
	else if(f.send_vef)
	{
		DY_DT_Send_VER();
		f.send_vef = 0;
	}
	DY_DT_Data_Receive_Anl_Task();		//数传数据解析
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
static u8 DT_RxBuffer[256],DT_data_cnt = 0,ano_dt_data_ok;

void DY_DT_Data_Receive_Anl_Task()
{
	if(ano_dt_data_ok)
	{
		DY_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+6);
		ano_dt_data_ok = 0;
	}
}

void DY_DT_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 _state = 0;

	if (_state == 0 && data == 0xAA) //帧头0xAA
	{
		_state = 1;
		DT_RxBuffer[0]=data;
	}
	else if (_state == 1 && data == 0xAF) //数据源，0xAF表示数据来自上位机
	{
		_state = 2;
		DT_RxBuffer[1]=data;
	}
	else if (_state == 2) //数据目的地
	{
		_state = 3;
		DT_RxBuffer[2]=data;
	}
	else if (_state == 3) //功能字
	{
		_state = 4;
		DT_RxBuffer[3]=data;
	}
	else if (_state == 4) //数据长度
	{
		_state = 5;
		DT_RxBuffer[4]=data;
		_data_len = data;
		DT_data_cnt = 0;
	}
	else if (_state == 5 && _data_len > 0)
	{
		_data_len--;
		DT_RxBuffer[5+DT_data_cnt++]=data;
		if(_data_len==0)
			_state = 6;
	}
	else if (_state == 6)
	{
		_state = 0;
		DT_RxBuffer[5+DT_data_cnt]=data;
		ano_dt_data_ok = 1;//DY_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+5);
	}
	else
		_state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 flash_save_en_cnt = 0;
void DY_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头

	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==0XE0)			//命令E0
		{
			switch(*(data_buf+5))		//CMD1
			{
				case 0x01:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x01)
						sensor.acc_CALIBRATE = 1;		//加速度计校准
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x02)
						sensor.gyr_CALIBRATE = 1;		//陀螺仪校准
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x04)
						mag.mag_CALIBRATE = 1;		    //磁力计校准
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xB0)
						f.send_version = 1;		        //读取版本信息
					break;
				case 0x02:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAA)
					{
						PID_Rest();		    //恢复默认PID参数
						All_PID_Init();		//PID初始化
						data_save();		//Flash存储
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAB)
					{
						PID_Rest();		    //恢复默认PID参数
						All_PID_Init();		//PID初始化
						Parame_Reset();		//参数初始化
						data_save();		//Flash存储
					}
					break;
				case 0xE1:
					f.paraToSend = (u16)(*(data_buf+6)<<8)|*(data_buf+7);	//读取参数
					break;
				default:
					break;
			}
			DY_DT_SendCmd(SWJADDR,*(data_buf+5),(u16)(*(data_buf+6)<<8)|*(data_buf+7));
		}
		else if(*(data_buf+3)==0XE1)	//设置参数
		{
			u16 _paraNum = (u16)(*(data_buf+5)<<8)|*(data_buf+6);
			s32 _paraVal = (s32)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
			DY_DT_GetParame(_paraNum,_paraVal);
		}
	}
}

void DY_DT_SendCmd(u8 dest, u8 cmd1, u16 cmd2)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=dest;
	data_to_send[_cnt++]=0xE0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=cmd1;
	data_to_send[_cnt++]=BYTE1(cmd2);
	data_to_send[_cnt++]=BYTE0(cmd2);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_SendParame(u16 num)
{
	u8 _cnt=0;
	int32_t data;
	if(num > PARNUM)
		return;
	DY_DT_ParUsedToParList();
	data = ParValList[num];
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x09;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(num);
	data_to_send[_cnt++]=BYTE0(num);
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_GetParame(u16 num,s32 data)
{
	if(num > PARNUM)		//num<=100
		return;
	ParValList[num] = data;		//参数列表ParValList[100]
	DY_DT_ParListToParUsed();
	f.paraToSend = num;	//将接收到的参数发回上位机进行双向验证
	data_save();		//Flash存储
}

void DY_DT_ParListToParUsed(void)
{
	DY_Parame.set.pid_att_1level[ROL][KP] = (float) ParValList[PAR_PID_1_P] / 1000;
	DY_Parame.set.pid_att_1level[ROL][KI] = (float) ParValList[PAR_PID_1_I] / 1000;
	DY_Parame.set.pid_att_1level[ROL][KD] = (float) ParValList[PAR_PID_1_D] / 1000;
	DY_Parame.set.pid_att_1level[PIT][KP] = (float) ParValList[PAR_PID_2_P] / 1000;
	DY_Parame.set.pid_att_1level[PIT][KI] = (float) ParValList[PAR_PID_2_I] / 1000;
	DY_Parame.set.pid_att_1level[PIT][KD] = (float) ParValList[PAR_PID_2_D] / 1000;
	DY_Parame.set.pid_att_1level[YAW][KP] = (float) ParValList[PAR_PID_3_P] / 1000;
	DY_Parame.set.pid_att_1level[YAW][KI] = (float) ParValList[PAR_PID_3_I] / 1000;
	DY_Parame.set.pid_att_1level[YAW][KD] = (float) ParValList[PAR_PID_3_D] / 1000;

	DY_Parame.set.pid_att_2level[ROL][KP] = (float) ParValList[PAR_PID_4_P] / 1000;
	DY_Parame.set.pid_att_2level[ROL][KI] = (float) ParValList[PAR_PID_4_I] / 1000;
	DY_Parame.set.pid_att_2level[ROL][KD] = (float) ParValList[PAR_PID_4_D] / 1000;
	DY_Parame.set.pid_att_2level[PIT][KP] = (float) ParValList[PAR_PID_5_P] / 1000;
	DY_Parame.set.pid_att_2level[PIT][KI] = (float) ParValList[PAR_PID_5_I] / 1000;
	DY_Parame.set.pid_att_2level[PIT][KD] = (float) ParValList[PAR_PID_5_D] / 1000;
	DY_Parame.set.pid_att_2level[YAW][KP] = (float) ParValList[PAR_PID_6_P] / 1000;
	DY_Parame.set.pid_att_2level[YAW][KI] = (float) ParValList[PAR_PID_6_I] / 1000;
	DY_Parame.set.pid_att_2level[YAW][KD] = (float) ParValList[PAR_PID_6_D] / 1000;

	DY_Parame.set.pid_alt_1level[KP] = (float) ParValList[PAR_PID_7_P] / 1000;
	DY_Parame.set.pid_alt_1level[KI] = (float) ParValList[PAR_PID_7_I] / 1000;
	DY_Parame.set.pid_alt_1level[KD] = (float) ParValList[PAR_PID_7_D] / 1000;
	DY_Parame.set.pid_alt_2level[KP] = (float) ParValList[PAR_PID_8_P] / 1000;
	DY_Parame.set.pid_alt_2level[KI] = (float) ParValList[PAR_PID_8_I] / 1000;
	DY_Parame.set.pid_alt_2level[KD] = (float) ParValList[PAR_PID_8_D] / 1000;

	DY_Parame.set.pid_loc_1level[KP] = (float) ParValList[PAR_PID_9_P] / 1000;
	DY_Parame.set.pid_loc_1level[KI] = (float) ParValList[PAR_PID_9_I] / 1000;
	DY_Parame.set.pid_loc_1level[KD] = (float) ParValList[PAR_PID_9_D] / 1000;
	DY_Parame.set.pid_loc_2level[KP] = (float) ParValList[PAR_PID_10_P] / 1000;
	DY_Parame.set.pid_loc_2level[KI] = (float) ParValList[PAR_PID_10_I] / 1000;
	DY_Parame.set.pid_loc_2level[KD] = (float) ParValList[PAR_PID_10_D] / 1000;

	if(ParValList[PAR_RCINMODE] == 0)
		DY_Parame.set.pwmInMode = PWM;
	else
		DY_Parame.set.pwmInMode = PPM;

	DY_Parame.set.warn_power_voltage = (float) ParValList[PAR_LVWARN] / 10;
	DY_Parame.set.return_home_power_voltage = (float) ParValList[PAR_LVRETN] / 10;
	DY_Parame.set.lowest_power_voltage = (float) ParValList[PAR_LVDOWN] / 10;

	DY_Parame.set.auto_take_off_height = ParValList[PAR_TAKEOFFHIGH];//cm
}

void DY_DT_ParUsedToParList(void)
{
	ParValList[PAR_PID_1_P] = DY_Parame.set.pid_att_1level[ROL][KP] * 1000;
	ParValList[PAR_PID_1_I] = DY_Parame.set.pid_att_1level[ROL][KI] * 1000;
	ParValList[PAR_PID_1_D] = DY_Parame.set.pid_att_1level[ROL][KD] * 1000;
	ParValList[PAR_PID_2_P] = DY_Parame.set.pid_att_1level[PIT][KP] * 1000;
	ParValList[PAR_PID_2_I] = DY_Parame.set.pid_att_1level[PIT][KI] * 1000;
	ParValList[PAR_PID_2_D] = DY_Parame.set.pid_att_1level[PIT][KD] * 1000;
	ParValList[PAR_PID_3_P] = DY_Parame.set.pid_att_1level[YAW][KP] * 1000;
	ParValList[PAR_PID_3_I] = DY_Parame.set.pid_att_1level[YAW][KI] * 1000;
	ParValList[PAR_PID_3_D] = DY_Parame.set.pid_att_1level[YAW][KD] * 1000;

	ParValList[PAR_PID_4_P] = DY_Parame.set.pid_att_2level[ROL][KP] * 1000;
	ParValList[PAR_PID_4_I] = DY_Parame.set.pid_att_2level[ROL][KI] * 1000;
	ParValList[PAR_PID_4_D] = DY_Parame.set.pid_att_2level[ROL][KD] * 1000;
	ParValList[PAR_PID_5_P] = DY_Parame.set.pid_att_2level[PIT][KP] * 1000;
	ParValList[PAR_PID_5_I] = DY_Parame.set.pid_att_2level[PIT][KI] * 1000;
	ParValList[PAR_PID_5_D] = DY_Parame.set.pid_att_2level[PIT][KD] * 1000;
	ParValList[PAR_PID_6_P] = DY_Parame.set.pid_att_2level[YAW][KP] * 1000;
	ParValList[PAR_PID_6_I] = DY_Parame.set.pid_att_2level[YAW][KI] * 1000;
	ParValList[PAR_PID_6_D] = DY_Parame.set.pid_att_2level[YAW][KD] * 1000;

	ParValList[PAR_PID_7_P] = DY_Parame.set.pid_alt_1level[KP] * 1000;
	ParValList[PAR_PID_7_I] = DY_Parame.set.pid_alt_1level[KI] * 1000;
	ParValList[PAR_PID_7_D] = DY_Parame.set.pid_alt_1level[KD] * 1000;
	ParValList[PAR_PID_8_P] = DY_Parame.set.pid_alt_2level[KP] * 1000;
	ParValList[PAR_PID_8_I] = DY_Parame.set.pid_alt_2level[KI] * 1000;
	ParValList[PAR_PID_8_D] = DY_Parame.set.pid_alt_2level[KD] * 1000;

	ParValList[PAR_PID_9_P] = DY_Parame.set.pid_loc_1level[KP] * 1000;
	ParValList[PAR_PID_9_I] = DY_Parame.set.pid_loc_1level[KI] * 1000;
	ParValList[PAR_PID_9_D] = DY_Parame.set.pid_loc_1level[KD] * 1000;
	ParValList[PAR_PID_10_P] = DY_Parame.set.pid_loc_2level[KP] * 1000;
	ParValList[PAR_PID_10_I] = DY_Parame.set.pid_loc_2level[KI] * 1000;
	ParValList[PAR_PID_10_D] = DY_Parame.set.pid_loc_2level[KD] * 1000;

	if(DY_Parame.set.pwmInMode == PWM)
		ParValList[PAR_RCINMODE] = 0;
	else
		ParValList[PAR_RCINMODE] = 1;

	ParValList[PAR_LVWARN] = DY_Parame.set.warn_power_voltage * 10;
	ParValList[PAR_LVRETN] = DY_Parame.set.return_home_power_voltage * 10;
	ParValList[PAR_LVDOWN] = DY_Parame.set.lowest_power_voltage * 10;

	ParValList[PAR_TAKEOFFHIGH] = DY_Parame.set.auto_take_off_height;
}

void DY_DT_Send_VER(void)
{
	u8 temp[14];
	temp[0] = 0xAA;
	temp[1] = 0xAA;
	temp[2] = 0x00;
	temp[3] = 9;
	temp[4] = HW_TYPE;
	temp[5] = HW_VER/256;
	temp[6] = HW_VER%256;
	temp[7] = SOFT_VER/256;
	temp[8] = SOFT_VER%256;
	temp[9] = PT_VER/256;
	temp[10] = PT_VER%256;
	temp[11] = BL_VER/256;
	temp[12] = BL_VER%256;
	u8 check_sum = 0;
	for(u8 i=0;i<13;i++)
		check_sum += temp[i];
	temp[13] = check_sum;

	DY_DT_Send_Data(temp,14);
}

void DY_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;

	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);

}

void DY_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;

	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);


	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);

}

void DY_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	data_to_send[_cnt++] = fly_model;

	data_to_send[_cnt++] = armed;

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;

	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_Senser2(s32 bar_alt,s32 csb_alt)
{
	u8 _cnt=0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE3(csb_alt);
	data_to_send[_cnt++]=BYTE2(csb_alt);
	data_to_send[_cnt++]=BYTE1(csb_alt);
	data_to_send[_cnt++]=BYTE0(csb_alt);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;

	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_SendString(char *str, u8 len)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xA0;
	data_to_send[_cnt++]=0;

	for(u8 i=0; i<len; i++)
		data_to_send[_cnt++] = *(str+i);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

void DY_DT_Send_User(u8 num, s16 data_1, s16 data_2)
{
	u8 _cnt=0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xF0+num; //用户数据，1开始
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=BYTE1(data_1);
	data_to_send[_cnt++]=BYTE0(data_1);
	data_to_send[_cnt++]=BYTE1(data_2);
	data_to_send[_cnt++]=BYTE0(data_2);

	data_to_send[4] = _cnt-5;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	DY_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/

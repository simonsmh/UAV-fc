#include "DY_OF.h"
#include "DY_FcData.h"

uint8_t		OF_QUA,OF_LIGHT;
int8_t		OF_DX,OF_DY;
int16_t		OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
uint16_t	OF_ALT,OF_ALT2;
int16_t		OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
int16_t		OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
int16_t		OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
int16_t		OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
float		OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
float		OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;

void DY_OF_DataAnl(uint8_t *data_buf,uint8_t num);

static uint8_t _datatemp[50];
static u8 _data_cnt = 0;

void DY_OF_DataAnl_Task(u8 dT_ms)
{
  DY_OF_Check(dT_ms);
}


//DY_OF_GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽڹ������ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������DY_OF_DataAnl
void DY_OF_GetOneByte(uint8_t data)
{
  	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//Դ��ַ
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//Ŀ�ĵ�ַ
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//������
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//����
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		DY_OF_DataAnl(_datatemp,_data_cnt+6);//anoof_data_ok = 1 ;//
	}
	else
		state = 0;
    
//	static u8 _data_len = 0;
//	static u8 state = 0;
//	
//	if(state==0&&data==0xAA)		//���ֽ�0xAA
//	{
//		state=1;
//		_datatemp[0]=data;
//	}
//	else if(state==1&&data==0xAA)		//�ڶ��ֽ�0xAA
//	{
//		state=2;
//		_datatemp[1]=data;
//	}
//	else if(state==2&&data<0XF1)		//�����ֽ�:<0xF1
//	{
//		state=3;
//		_datatemp[2]=data;
//	}
//	else if(state==3&&data<45)		//�����ֽ�:0<���ݳ���<45
//	{
//		state = 4;
//		_datatemp[3]=data;
//		_data_len = data;
//		_data_cnt = 0;
//	}
//	else if(state==4&&_data_len>0)
//	{
//		_data_len--;
//		_datatemp[4+_data_cnt++]=data;
//		if(_data_len==0)
//			state = 5;
//	}
//	else if(state==5)		//���һ�ֽ�
//	{
//		state = 0;
//		_datatemp[4+_data_cnt]=data;
//		DY_OF_DataAnl(_datatemp,_data_cnt+5);//anoof_data_ok = 1 ;//_datatemp����֡��_data_cnt+5֡����
//	}
//	else
//		state = 0;
}
//DY_OF_DataAnlΪ�������ݽ�������������ͨ���������õ�����ģ������ĸ�������
//�������ݵ����壬�������������ģ��ʹ���ֲᣬ����ϸ�Ľ���
static u8 of_check_f[2];
static u16 of_check_cnt[2] = { 10000,10000 };
void DY_OF_Check(u8 dT_ms)
{
	for(u8 i=0;i<2;i++)
	{
		if(of_check_f[i] == 0 )		//of_check_f[0]������Ϣ��of_check_f[1]�߶���Ϣ�����ι�����Ϣ���͸߶���Ϣ��������<1s������Ϊ����ģ��OK��
		{
			if(of_check_cnt[i]<10000)
			{
				of_check_cnt[i] += dT_ms;	
			}
		}
		else
		{
			of_check_cnt[i] = 0;
		}
		

		of_check_f[i] = 0;
	}
	
	
	if(of_check_cnt[0] > 1000 || of_check_cnt[1] > 1000)
	{
		sens_hd_check.of_ok = 0;		//
	}
	else
	{
		sens_hd_check.of_ok = 1;		//
	}
	
}

void DY_OF_DataAnl(uint8_t *data_buf,uint8_t num)		//�������ݽ�������
{
    u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+3)==0X51)//������Ϣ
	{
		if(*(data_buf+5)==0)//ԭʼ������Ϣ
		{
			OF_QUA 		= *(data_buf+6);
			OF_DX  		= *(data_buf+7);
			OF_DY  		= *(data_buf+8);
			OF_LIGHT  	= *(data_buf+9);
		}
		else if(*(data_buf+5)==1)//�ںϺ������Ϣ
		{
			OF_QUA 		= *(data_buf+6);
			OF_DX2		= (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_DY2		= (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_DX2FIX	= (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_DY2FIX	= (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_LIGHT  	= *(data_buf+19);
			
			of_check_f[0] = 1;
		}
	}
	if(*(data_buf+3)==0X52)//�߶���Ϣ
	{
		if(*(data_buf+5)==0)//ԭʼ�߶���Ϣ
		{
			OF_ALT = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			of_check_f[1] = 1;
		}
		else if(*(data_buf+5)==1)//�ںϺ�߶���Ϣ
		{
			OF_ALT2 = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
		}
	}
//	if(*(data_buf+2)==0X53)//��������
//	{
//		if(*(data_buf+4)==0)//ԭʼ����
//		{
//			OF_GYR_X = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			OF_GYR_Y = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			OF_GYR_Z = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			OF_ACC_X = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			OF_ACC_Y = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			OF_ACC_Z = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//		else if(*(data_buf+4)==1)//�˲�������
//		{
//			OF_GYR_X2 = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			OF_GYR_Y2 = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			OF_GYR_Z2 = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			OF_ACC_X2 = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			OF_ACC_Y2 = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			OF_ACC_Z2 = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//	}
//	if(*(data_buf+2)==0X54)//��̬��Ϣ
//	{
//		if(*(data_buf+4)==0)//ŷ���Ǹ�ʽ
//		{
//			OF_ATT_ROL = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.01 ;
//			OF_ATT_PIT = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.01 ;
//			OF_ATT_YAW = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.01 ;
//		}
//		else if(*(data_buf+4)==1)//��Ԫ����ʽ
//		{
//			OF_ATT_S1 = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.0001 ;
//			OF_ATT_S2 = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.0001 ;
//			OF_ATT_S3 = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.0001 ;
//			OF_ATT_S4 = ((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)) * 0.0001 ;
//		}
//	}
    
//	u8 sum = 0;
//	for(u8 i=0;i<(num-1);i++)
//		sum += *(data_buf+i);
//	if(!(sum==*(data_buf+num-1)))		return;		//У��
//	
//	if(*(data_buf+2)==0X51)//������Ϣ
//	{
//		if(*(data_buf+4)==0)//ԭʼ������Ϣ
//		{
//			OF_QUA 		= *(data_buf+5);
//			OF_DX  		= *(data_buf+6);
//			OF_DY  		= *(data_buf+7);
//			OF_LIGHT  	= *(data_buf+8);
//		}
//		else if(*(data_buf+4)==1)//�ںϺ������Ϣ	;���ⲿʹ��
//		{
//			OF_QUA 		= *(data_buf+5);
//			OF_DX2		= (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
//			OF_DY2		= (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) ;
//			OF_DX2FIX	= (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) ;
//			OF_DY2FIX	= (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ;
//			OF_LIGHT  	= *(data_buf+14);
//			
//			of_check_f[0] = 1;
//		}
//	}
//	if(*(data_buf+2)==0X52)//�߶���Ϣ
//	{
//		if(*(data_buf+4)==0)//ԭʼ�߶���Ϣ	;���ⲿʹ��
//		{
//			OF_ALT = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//            
//			of_check_f[1] = 1;
//		}
//		else if(*(data_buf+4)==1)//�ںϺ�߶���Ϣ
//		{
//			OF_ALT2 = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//		}
//	}
//	if(*(data_buf+2)==0X53)//��������	;���ⲿʹ��
//	{
//		if(*(data_buf+4)==0)//ԭʼ����
//		{
//			OF_GYR_X = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			OF_GYR_Y = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			OF_GYR_Z = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			OF_ACC_X = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			OF_ACC_Y = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			OF_ACC_Z = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//		else if(*(data_buf+4)==1)//�˲�������
//		{
//			OF_GYR_X2 = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
//			OF_GYR_Y2 = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
//			OF_GYR_Z2 = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
//			OF_ACC_X2 = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
//			OF_ACC_Y2 = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
//			OF_ACC_Z2 = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
//		}
//	}
//	if(*(data_buf+2)==0X54)//��̬��Ϣ	;���ⲿʹ��
//	{
//		if(*(data_buf+4)==0)//ŷ���Ǹ�ʽ
//		{
//			OF_ATT_ROL = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.01 ;
//			OF_ATT_PIT = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.01 ;
//			OF_ATT_YAW = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.01 ;
//		}
//		else if(*(data_buf+4)==1)//��Ԫ����ʽ
//		{
//			OF_ATT_S1 = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.0001 ;
//			OF_ATT_S2 = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.0001 ;
//			OF_ATT_S3 = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.0001 ;
//			OF_ATT_S4 = ((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)) * 0.0001 ;
//		}
//	}
}

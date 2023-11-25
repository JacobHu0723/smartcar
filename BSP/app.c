/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "app.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "oled.h"

uint8_t Buffer1;            //����4���ʹ������
uint8_t buf1[200];
char  sendbuf[200];
volatile uint8_t buf1_cnt=0;
char oledbuf[200];

volatile uint8_t flag_u1dmas=0;   // u1 dma send flag  1:  sending  0  send over
volatile uint8_t flag_move=0;   //  0, stop    1,move
volatile int8_t  flag_yaw=0;   //ƫ����־λ ���ο�ͼƬ
volatile int8_t  flag_yaw_last=0;  //  ƫ����־λ����һ��ֵ
volatile uint8_t flag_IRED=0;     //�ĸ�ǰ�ú��⴫����
volatile uint8_t  pre=10;
uint16_t pwm1=0,pwm2=0,pwm3=0,pwm4=0;  // pwm1>0,pwm2=0 ����ǰ����pwm3=0,pwm4>0 ����ǰ��
uint16_t pm1=0,pm2=0,pm3=0,pm4=0;   // ��¼�趨ֵ�������ϴ�ʵʱֵ
int16_t EnML_cnt,EnMR_cnt;     // ��  �����ٶȷ���
uint8_t L_Mot_speed=0,R_Mot_speed=0,set_speed=0;   // ����  ����   �� ����  �ĸ���ֵ  ������ٶ�ֵ�� ������ֵ ��Աȣ��޵�λ��û�л����׼�ٶ�

// �����������趨�Ե������Ӱ��ǳ���
/*************************************/
float   Proportion=3.5;       //�������� Proportional Const
float   Integral=0.5;       //���ֳ��� Integral Const
float   Derivative=0;       //΢�ֳ��� Derivative Const
/********************����ʽPID�������************************************/
//NextPoint��ǰ���ֵ
//SetPoint�趨ֵ

//����PID
int PID_Calc_Left(int NextPoint,int SetPoint) 
{
                             
	static int      LastError;                                //Error[-1]
	static int      PrevError;                                //Error[-2]
  int iError,Outpid;                                        //��ǰ���
	
  iError=SetPoint-NextPoint;                                //��������
  Outpid=(Proportion * iError)                              //E[k]��
              -(Integral * LastError)                       //E[k-1]��
              +(Derivative * PrevError);                    //E[k-2]��
              
  PrevError=LastError;                                      //�洢�������´μ���
  LastError=iError;
  return(Outpid);                                           //��������ֵ
}


//����PID
int PID_Calc_Right(int NextPoint,int SetPoint) 
{                      
	static int      LastError;                                //Error[-1]
	static int      PrevError;                                //Error[-2]
  int iError,Outpid;                                        //��ǰ���
	
  iError=SetPoint-NextPoint;                                //��������
  Outpid=(Proportion * iError)                              //E[k]��
              -(Integral * LastError)                       //E[k-1]��
              +(Derivative * PrevError);                    //E[k-2]��
              
  PrevError=LastError;                                      //�洢�������´μ���
  LastError=iError;
  return(Outpid);                                           //��������ֵ
}
//==========================================================
//	�������ƣ�	move    
//	�������ܣ�	С����ǰ���˶�
//	��ڲ����� speed �����ٶ�ֵ
//	���ز�����	��    
//========================================================== 
void move( uint8_t speed)
{ 
	flag_move=1;    // С�����˶�״̬
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	set_speed=speed;
	if(speed>100)  speed=100;
	L_Mot_speed=speed;
	R_Mot_speed=speed;
	 pwm2=0;
	 pwm3=0;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm2); 
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm3);
	pwm1=2000+speed*16;                              // pwm<2000,�������ת
	pwm4=2000+speed*16; 
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm1); 
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm4);
	pm1=pwm1,pm2=pwm2,pm3=pwm3,pm4=pwm4;
}
//==========================================================
//	�������ƣ�	brake   
//	�������ܣ�	С��ɲ��
//	��ڲ�����  ��
//	���ز�����	��    
//========================================================== 
void brakes(void)
{
	flag_move=0;
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	pwm1=3600;
	pwm2=3600;
	pwm3=3600;
	pwm4=3600;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm2); 
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm3);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm1); 
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm4);	
	pm1=pwm1,pm2=pwm2,pm3=pwm3,pm4=pwm4;
} 
//==========================================================
//	�������ƣ�	turn_left   
//	�������ܣ�	�������٣�С����ת
//	��ڲ�����  ����ƫ����
//	���ز�����	��    
//========================================================== 
void turn_left(uint8_t yaw )
{  float sx;
	if(flag_move)
	{
		switch(yaw)
		{
			case 1:
				sx=0.88;
			break;
			case 2:
				sx=0.78;
			break;
			case 3:
				sx=0.68;
			break;
			
		}	
	L_Mot_speed=set_speed/sx;   
	R_Mot_speed=set_speed*sx;
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET);
	}
}
//==========================================================
//	�������ƣ�	turn_right   
//	�������ܣ�	�ҵ�����٣�С����ת
//	��ڲ�����  ����ƫ����
//	���ز�����	��    
//========================================================== 
void turn_right(uint8_t yaw )
{  float sx;
	if(flag_move)
	{
		switch(yaw)
		{
			case 1:
				sx=0.88;
			break;
			case 2:
				sx=0.78;
			break;
			case 3:
				sx=0.68;
			break;
			
		}	
	R_Mot_speed=set_speed/sx;   
	L_Mot_speed=set_speed*sx;
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
	}
}
//==========================================================
//	�������ƣ�	ctrdeal     ���������� tick �ж��д���
//	�������ܣ�	10msִ��һ�Σ���Ϊ��������
//	��ڲ�����  ��
//	���ز�����	��    
//========================================================== 
void	ctrdeal(void)   // 10ms����һ��
{  
	uint8_t spd;
		EnML_cnt=-__HAL_TIM_GetCounter(&htim2);
		EnMR_cnt=__HAL_TIM_GetCounter(&htim3);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
		
	spd=(int)(set_speed/10);
	pre=10-spd;
	if(spd<=2) pre=9;
	if(spd>=9) pre=2;
	
	EnML_cnt=EnML_cnt*10/pre;
	EnMR_cnt=EnMR_cnt*10/pre;
	 flag_IRED=(flag_IRED&0xf7)|(HAL_GPIO_ReadPin(IRED0_GPIO_Port,IRED0_Pin)<<3);  //  5  0101    1101  0110
	 flag_IRED=(flag_IRED&0xfb)|(HAL_GPIO_ReadPin(IRED1_GPIO_Port,IRED1_Pin)<<2);
	 flag_IRED=(flag_IRED&0xfd)|(HAL_GPIO_ReadPin(IRED2_GPIO_Port,IRED2_Pin)<<1);
	 flag_IRED=(flag_IRED&0xfe)|(HAL_GPIO_ReadPin(IRED3_GPIO_Port,IRED3_Pin));
   if(flag_move==0)
	   return;
	 
	 
	 if((flag_IRED&0x0f)==0)   // �ĸ���������δ����
		 flag_yaw=0;
	
	 if((flag_IRED&0x0f)==0x06)
		flag_yaw=0;
		else
	 if(flag_yaw==0)   //  ����ܹؼ�������ĸ�������δ������������һ��״̬�жϣ���ֹ����ѭ�������һ�������������֮��
	 {
		 if(flag_yaw_last==-3||flag_yaw_last==-2)
			 flag_yaw=-2;
		 if(flag_yaw_last==3||flag_yaw_last==2)
			 flag_yaw=2;		 
	 
		 switch(flag_yaw)
		 {
			 case 0:
				L_Mot_speed=set_speed;
				R_Mot_speed=set_speed;
			 HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
			 break;
			case -2:   // 
				turn_right(2);
			 break;
			 case -3:   // ��2������
				turn_right(3);
			 break;
			 
			 case 2:
				turn_left(2);
			 break;
			 case 3:  // ��2������
				turn_left(3);
			 break;
			 
		 }
	 }
	 flag_yaw_last=flag_yaw;
	if(flag_move)
	{ int16_t para_L,para_R;		
		
		para_L=PID_Calc_Left(EnML_cnt,L_Mot_speed);	       //�����������õ�����ʽPID��������ֵ 
		para_R=PID_Calc_Right(EnMR_cnt,R_Mot_speed);	       //�ҵ���������õ�����ʽPID��������ֵ 
		if((para_L<-5)||(para_L>5))                        //���� PID ��������������СʱƵ�����������𵴡�
			{
				pwm1 +=para_L;  
			}   
		if((para_R<-5)||(para_R>5))                     //���� PID ��������������СʱƵ�����������𵴡�
			{
				pwm4 +=para_R;  
			}			
  		if(pwm1>3600)pwm1=3600;    //�޷�
			if(pwm4>3600)pwm4=3600;    //�޷�
			if(pwm1<1000) pwm1=1000;
			if(pwm4<1000) pwm4=1000;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm1); 
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm4);
			pm1=pwm1,pm2=pwm2,pm3=pwm3,pm4=pwm4;
	}
	
	
	
	
}
//==========================================================
//	�������ƣ�  sec_dealtask     ���������� ��ѭ���д���
//	�������ܣ�	1���Ӵ���һ������
//	��ڲ�����  ��
//	���ز�����	��    
//========================================================== 
void  sec_dealtask(void)
{ 
	sprintf(sendbuf,"P1:%d,P2:%d,M_L:%d;SM_L:%d;P3:%d,P4:%d,M_R:%d;SM_R:%d;IR:%x\n",pwm1,pwm2,EnML_cnt,L_Mot_speed,pwm3,pwm4,EnMR_cnt,R_Mot_speed,flag_IRED);	
	u1dma_sendstr(sendbuf);	
	OLED_ShowString(0,0,(u8 *)"PID Para",12);
	sprintf(oledbuf,"P=%.1f,I=%.1f",Proportion,Integral);
	OLED_ShowString(0,2,(u8 *)oledbuf,16);
}
//==========================================================
//	�������ƣ�	deal_u1dat     ���������� ��ѭ���д���
//	�������ܣ�	����������λ�����������
//	��ڲ�����  cmd   len
//	���ز�����	��    
//==========================================================  
void deal_u1dat(uint8_t *buf,uint8_t len)   // 
{  
	
	switch(buf[0])
	{
		case 'p':
		
			switch(buf[1])
			{
				case '1':    // ����pwm1��ֵ����  p1:3000  ������pwm1 ֵΪ3000  ��ͬ
				 sscanf((char *)buf+3,"%d",(int *)&pwm1);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm1);
				 sprintf(sendbuf,"set pwm1=%d ok!\n",pwm1);
				 u1dma_sendstr(sendbuf);
				break;
				case '2':
				 sscanf((char *)buf+3,"%d",(int *)&pwm2);
				  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm2);
				  sprintf(sendbuf,"set pwm2=%d ok!\n",pwm2);
				  u1dma_sendstr(sendbuf);
				break;
				
				case '3':
					sscanf((char *)buf+3,"%d",(int *)&pwm3);
				  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm3);
				 sprintf(sendbuf,"set pwm3=%d ok!\n",pwm3);
				 u1dma_sendstr(sendbuf);
				break;
				
				case '4':
				 sscanf((char *)buf+3,"%d",(int *)&pwm4);
				 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm4);
				 sprintf(sendbuf,"set pwm4=%d ok!\n",pwm4);
				 u1dma_sendstr(sendbuf);
				break;
				case '5':				 // ��ʾPID�Ĳ���ֵ
				 sprintf(sendbuf,"PID para P=%f,I=%f,D=%f\n",Proportion,Integral,Derivative);
				 u1dma_sendstr(sendbuf);
				break;
				
			}		
			pm1=pwm1,pm2=pwm2,pm3=pwm3,pm4=pwm4;
		break;
				case 'd':     
						  __HAL_TIM_SET_COUNTER(&htim2, 0);
              __HAL_TIM_SET_COUNTER(&htim3, 0);
				break;
				
			case 'e':    // ��ʾpwm��ֵ
				sprintf(sendbuf,"PWM1:%d,PWM2:%d,PWM3:%d,PWM4:%d\n",pm1,pwm2,pwm3,pwm4);
				u1dma_sendstr(sendbuf);
			break;
			
			case 'm':   // С������   ��  m30 ���������ٶ�30��ʼ�˶�
			{  uint8_t spd;				
				 sscanf((char *)buf+1,"%d",(int *)&spd);
				 set_speed=spd;
				 move(set_speed);
				 sprintf(sendbuf,"zhixing ,set speed=%d ok!\n",set_speed);
				 u1dma_sendstr(sendbuf);
			}
			break;
			
			case 's':   //  ɲ��   s
				brakes();
			break;
			
			case 'P':  // ����Pֵ  ��P2.1������Pֵ=2.1   
			{  			
			 sscanf((char *)buf+1,"%f",&Proportion);				
			 sprintf(sendbuf,"set Proportion=%f ok!\n",Proportion);
			 u1dma_sendstr(sendbuf);
			}
			break;
			
			case 'I':  // ����Iֵ  
			{  			
			sscanf((char *)buf+1,"%f",&Integral);		
		  sprintf(sendbuf,"set Integral=%f ok!\n",Integral);
		  u1dma_sendstr(sendbuf);
			}
			break;
			
			case 'D': // ����Dֵ  
			{  	
			 sscanf((char *)buf+1,"%f",&Derivative);			
			 sprintf(sendbuf,"set Derivative=%f ok!\n",Derivative);
			 u1dma_sendstr(sendbuf);
			}
			break;
				
	}

  memset(buf,0,len);

}
//==========================================================
//	�������ƣ�	HAL_GPIO_EXTI_Callback
//	�������ܣ�	�ⲿ�жϻص���������Ҫ����������
//	��ڲ�����  
//	���ز�����	  
//==========================================================  
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t beep=10;
		
	switch(GPIO_Pin)
	{	
		case KEY1_Pin:
			HAL_Delay(5);
		  if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin))
			{
				LED1_Toggle();
				if(HAL_GPIO_ReadPin(LED1_GPIO_Port,LED1_Pin))
				{HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  				beep+=10;
				 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, beep);
				}
				else
					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			}
		break;
		
		case KEY2_Pin:  //����С���˶���ֹͣ
				HAL_Delay(5);
		  if(!HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin))
			{
				LED2_Toggle();
				if(HAL_GPIO_ReadPin(LED2_GPIO_Port,LED2_Pin))
				{
					move(25);   // �˶�
				}
				else
				{
					brakes();   // ͣ��
				}
			}
		break;
		case KEY3_Pin:
				HAL_Delay(5);
		  if(!HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin))
			{
				LED3_Toggle();
				flag_yaw_last=0;
				flag_yaw=0;
				if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin))
				{
					
				}
				else
				{
					
				}
			}
		break;
		
		case IRED0_Pin:   //��2
			turn_right(3);
		  flag_yaw=-3;
		break;
		case IRED1_Pin:   //��1
			turn_right(1);
		   flag_yaw=-1;
		break;
		case IRED2_Pin:   //��1
			turn_left(1);
		  flag_yaw=1;
		break;
		case IRED3_Pin:   //��2
			turn_left(3);
		  flag_yaw=3;
		break;
		
		
	}
}

// ���������ʹ������
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance==USART1)
    {
	  flag_u1dmas=0;
		}		
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
 if(huart->Instance==USART1)
    {
        buf1[buf1_cnt++]=Buffer1; 
        HAL_UART_Receive_IT(&huart1,&Buffer1,1);
    }	
   
}

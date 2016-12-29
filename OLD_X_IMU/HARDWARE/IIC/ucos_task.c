#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "ms5611_2.h"
#include "hml5833l.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "eso.h"
#include "neuron_pid.h"
#include "sonar_avoid.h"
#include "error.h"
#include "circle.h"
#include "bmp_adc.h"
#include "filter.h"
#include "gps.h"
//==============================传感器 任务函数==========================
float inner_loop_time_time;
u8 GOL_LINK_BUSY[2]={0,0};
void TIM3_IRQHandler(void)
{
	OSIntEnter();static u16 cnt,cnt1,cnt_init,init;
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
	inner_loop_time_time = Get_Cycle_T(GET_T_MEMS); 	
	if(!init){
	if(cnt_init++>1000)
	init=1;
	inner_loop_time_time=0.0025;
	}
	else{
							//获取内环准确的执行周期
	if(inner_loop_time_time<0.002)inner_loop_time_time=0.0025;
	MPU6050_Read(); 															//读取mpu6轴传感器
	MPU6050_Data_Prepare( inner_loop_time_time );			//mpu6轴传感器数据处理
	if(cnt++>=5){cnt=0;ANO_AK8975_Read();	}			//获取电子罗盘数据	
	if(cnt1++>=2){cnt1=0;
	MS5611_ThreadNew();}
	static u8 cnt_uart;
	if(!GOL_LINK_BUSY[1]&&cnt_uart++>10){cnt_uart=0;
	GOL_LINK_BUSY[0]=1;
	GOL_LINK_TASK();
	GOL_LINK_BUSY[0]=0;	
	}		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
	}
	OSIntExit(); 
}


//========================外环  任务函数============================
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
float YawR,PitchR,RollR;
float outer_loop_time;
void outer_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
 	while(1)
	{	
	outer_loop_time = Get_Cycle_T(GET_T_OUTTER);								//获取外环准确的执行周期	
	if(!init){if(cnt_init++>200)
		init=1;
	outer_loop_time=0.005;
	}
	else{

	IMUupdate(0.5f *outer_loop_time,my_deathzoom_2(mpu6050.Gyro_deg.x,0.0), my_deathzoom_2(mpu6050.Gyro_deg.y,0.0), my_deathzoom_2(mpu6050.Gyro_deg.z,0.0), mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&RollR,&PitchR,&YawR);		
	if(mode.en_imu_ekf==0){
	Yaw=YawR;
	Pitch=PitchR;
	Roll=RollR;}
	#define SEL_1 0
	#if SEL_1//1 梯度
	MadgwickAHRSupdate(outer_loop_time,my_deathzoom_2(mpu6050.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(mpu6050.Gyro_deg.y,0.5)/57.3, 
	my_deathzoom_2(mpu6050.Gyro_deg.z,0.5)/57.3,(float)mpu6050.Acc.x/4096., (float)mpu6050.Acc.y/4096., (float)mpu6050.Acc.z/4096.,
	0,0,0,
	&Roll_mid_down,&Pitch_mid_down,&Yaw_mid_down);
	#else //0 互补
	MahonyAHRSupdate(outer_loop_time,my_deathzoom_2(mpu6050.Gyro_deg.x,0.5)/57.3, my_deathzoom_2(mpu6050.Gyro_deg.y,0.5)/57.3, 
	my_deathzoom_2(mpu6050.Gyro_deg.z,0.5)/57.3,(float)mpu6050.Acc.x/4096., (float)mpu6050.Acc.y/4096., (float)mpu6050.Acc.z/4096.,
	0,0,0,
	&Roll_mid_down,&Pitch_mid_down,&Yaw_mid_down);
	#endif
	
  }
	
	delay_ms(5);
	}
}		


//========================EKF  任务函数============================
OS_STK EKF_TASK_STK[EKF_STK_SIZE];
float ekf_loop_time;

#include "FastMath.h"
#include "Quaternion.h"
#include "ekf_ins.h"
//#define USE_EKF
//#define USE_UKF
//#define USE_CKF
#define USE_SRCKF
//#define USE_9AXIS_EKF
//#define USE_EKF_INS

#ifdef USE_EKF
	#include "EKF.h"
#elif defined USE_UKF
  #include "UKF.h"
#elif defined USE_CKF
	#include "CKF.h"
#elif defined USE_SRCKF
	#include "SRCKF.h"
#elif defined USE_9AXIS_EKF
  #include "miniARSH.h"
#endif


#ifdef USE_EKF
	EKF_Filter ekf;
#elif defined USE_UKF
	UKF_Filter ukf;
#elif defined USE_CKF
	CKF_Filter ckf;
#elif defined USE_SRCKF
	SRCKF_Filter srckf;
#endif
	float fRealGyro[3] = {0}, fRealAccel[3] = {0,0,0};
	float fRealMag[3] = {0}, fRealQ[4] = {0,0,0,0};
	long lQuat[4] = {0,0,0,0};
	float fRPY[3] = {0};
	float fQ[4] = {0};
	int flag_mag[3]={1,1,-1};
	float Kp_ekf=0.3f,Kp_ekf_pr=6; 
	float reference_v_ekf[3];
void ekf_task(void *pdata)
{	static u8 cnt,cnt1,cnt2;			
  static u8 init,cnt_init;	
	float fRPYr[3] = {0};
 	while(1)
	{	
		
		ekf_loop_time = Get_Cycle_T(GET_T_EKF);			
if(cnt_init++>100&&!init){cnt_init=101;
		init=1;
#ifdef USE_EKF
	//Create a new EKF object;
	EKF_New(&ekf);
#elif defined USE_UKF
	//Create a new UKF object;
	UKF_New(&ukf);
#elif defined USE_CKF
	//Create a new CKF object;
	CKF_New(&ckf);
#elif defined USE_SRCKF
	SRCKF_New(&srckf);
#endif
#ifdef USE_EKF
				EKF_Init(&ekf, fRealQ, fRealGyro);
#elif defined USE_UKF
				UKF_Init(&ukf, fRealQ, fRealGyro);
#elif defined USE_CKF
				CKF_Init(&ckf, fRealQ, fRealGyro);
#elif defined USE_SRCKF
				SRCKF_Init(&srckf, fRealAccel, fRealMag);
#elif defined USE_6AXIS_EKF
				EKF_IMUInit(fRealAccel, fRealGyro);
#elif defined USE_6AXIS_FP_EKF
				FP_EKF_IMUInit(fRealAccel, fRealGyro);
#elif defined USE_9AXIS_EKF
				EKF_AHRSInit(fRealAccel, fRealMag);
#endif		

	}
	else{
	
	u16 i,rxlen;
	u16 lenx;
	if(USART3_RX_STA&0X8000)		//接收到一次数据了
		{
			rxlen=USART3_RX_STA&0X7FFF;	//得到数据长度
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART3_RX_BUF[i];	   
 			USART3_RX_STA=0;		   	//启动下一次接收
			USART1_TX_BUF[i]=0;			//自动添加结束符
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
 		}
	
		
	static u8 ekf_gps_cnt;	
	//if(ekf_gps_cnt++>1){ekf_gps_cnt=0;	
	//EKF_INS_GPS_Run(0.015);		
		GpsUkfProcess(0.02);
	//}
}
	/*
		    fRealGyro[0] = my_deathzoom_2(mpu6050.Gyro_deg.x,0.0) * DEG_RAD*1;
				fRealGyro[1] = my_deathzoom_2(mpu6050.Gyro_deg.y,0.0) * DEG_RAD*1;
				fRealGyro[2] = my_deathzoom_2(mpu6050.Gyro_deg.z,0.0) * DEG_RAD*1;
			
				fRealAccel[0] = mpu6050.Acc.x/ 4096;
				fRealAccel[1] = mpu6050.Acc.y/ 4096;
				fRealAccel[2] = mpu6050.Acc.z/ 4096;

	 fRealMag[0]=flag_mag[0]*ak8975.Mag_Val.x;
	 fRealMag[1]=flag_mag[1]*ak8975.Mag_Val.y;
	 fRealMag[2]=flag_mag[2]*ak8975.Mag_Val.z;
		
		Quaternion_From6AxisData(fRealQ, fRealAccel, fRealMag);
		float fDeltaTime;		
				if(ekf_loop_time!=0)fDeltaTime=ekf_loop_time;
				else
				fDeltaTime=0.01;	
#ifdef USE_EKF
				EFK_Update(&ekf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_UKF
				UKF_Update(&ukf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_CKF
				CKF_Update(&ckf, fRealQ, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_SRCKF
				SRCKF_Update(&srckf, fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#elif defined USE_6AXIS_EKF
				EKF_IMUUpdate(fRealGyro, fRealAccel, fDeltaTime);
#elif defined USE_6AXIS_FP_EKF
				FP_EKF_IMUUpdate(fRealGyro, fRealAccel, fDeltaTime);
#elif defined USE_9AXIS_EKF
				EKF_AHRSUpdate(fRealGyro, fRealAccel, fRealMag, fDeltaTime);
#endif	
#ifdef USE_EKF
			EKF_GetAngle(&ekf, fRPYr);
			EKF_GetQ(&ekf, fQ);
#elif defined USE_UKF
			UKF_GetAngle(&ukf, fRPYr);
			UKF_GetQ(&ukf, fQ);
#elif defined USE_CKF
			CKF_GetAngle(&ckf, fRPYr);
			CKF_GetQ(&ckf, fQ);
#elif defined USE_SRCKF
			SRCKF_GetAngle(&srckf, fRPYr);
			SRCKF_GetQ(&srckf, fQ);
#elif defined USE_6AXIS_EKF
			EKF_IMUGetAngle(fRPYr);
			EKF_IMUGetQ(fQ);
#elif defined USE_6AXIS_FP_EKF
			FP_EKF_IMUGetAngle(fRPYr);
			FP_EKF_IMUGetQ(fQ);
#elif defined USE_9AXIS_EKF
			EKF_AHRSGetAngle(fRPYr);
			EKF_AHRSGetQ(fQ);
#endif
			lQuat[0] = (long)(fQ[0] * 2147483648.0f);
			lQuat[1] = (long)(fQ[1] * 2147483648.0f);
			lQuat[2] = (long)(fQ[2] * 2147483648.0f);
			lQuat[3] = (long)(fQ[3] * 2147483648.0f);
	 fRPY[0] += Kp_ekf_pr *0.1f *(-fRPY[0]+(fRPYr[0]));
	 fRPY[1] += Kp_ekf_pr *0.1f *(-fRPY[1]+(-fRPYr[1]));
	 fRPY[2]  =-fRPYr[2];
	reference_v_ekf[0]= 2*(fQ[1]*fQ[3] - fQ[0]*fQ[2]);
	reference_v_ekf[1]= 2*(fQ[0]*fQ[1] + fQ[2]*fQ[3]);
	reference_v_ekf[2] = 1 - 2*(fQ[1]*fQ[1] + fQ[2]*fQ[2]);
#if NOT_DEBUG_EKF
if(mode.en_imu_ekf) {
	q_nav[0]=fQ[0];
	q_nav[1]=fQ[1];
	q_nav[2]=fQ[2];
	q_nav[3]=fQ[3];
  reference_vr[0]=reference_v.x =reference_v_ekf[0];// 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y =reference_v_ekf[1];// 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z =reference_v_ekf[2];// 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + r
	Pitch= fRPY[1] ;
	Roll=  fRPY[0] ;
	Yaw=   fRPY[2] ;
}
#endif
*/
	delay_ms(15);
	}
}		


//气压计 任务函数
OS_STK BARO_TASK_STK[BARO_STK_SIZE];
void baro_task(void *pdata)
{							  
 	while(1)
	{ static u8 cnt;
		altUkfProcess(0);
		delay_ms(20);  
	}
}	

//=======================超声波 任务函数==================
OS_STK SONAR_TASK_STK[SONAR_STK_SIZE];
void sonar_task(void *pdata)
{							  
 	while(1)
	{

//		#if defined(SONAR_USE_SCL) 
//			if(!Thr_Low)Ultra_Duty_SCL();delay_ms(100);
//		#else
//			//if(fly_ready||en_ble_debug)
//				Ultra_Duty(); 
//		#if defined(USE_KS103)
//			 #if defined(SONAR_SAMPLE1)
//				delay_ms(40);
//			 #elif defined(SONAR_SAMPLE2)
//				delay_ms(100);
//		   #elif defined(SONAR_SAMPLE3)
//				delay_ms(70);
//			
//			 #endif
//		#elif defined(USE_US100)
//				delay_ms(100);
//		#elif defined(USE_LASER|| USE_LASER_BEIX)	
			 Laser_Duty(); 
			 delay_ms(1000);
		//#endif
	//	#endif

	}
}	


//=======================FLOW 任务函数==================
OS_STK FLOW_TASK_STK[FLOW_STK_SIZE];
float acc_neo[3],flow_ground_temp[4];
float flow_filter[4],flow_rad_fix[4];
float flow_rad_fix_k[2]={0.8,1},flow_rad_fix_k2=0.0;
float dead_rad_fix[2]={40,50};
float flow_height_fliter;
float wz_speed_flow[2];
float w_acc_spd=0.815;
float w_acc_fix=0.8;
float scale_pix=0.0055;//0.002;//.003;//0.005;
float k_acc_forward=0;
u8 MID_CNT_SPD=10;
float k_flp=1-0.1; 
//px4
double rate_threshold = 0.15f; 
float flow_k=0.15f;float k_gro_off=1;
float flow_m[3];  
float flow_module_offset_y=0,flow_module_offset_x=0; 	
float flow_gyrospeed[3];
u8 flow_px_sel=1;
float flow_use;
float scale_px4_flow=1300;	float x_flow_orign_temp,y_flow_orign_temp;		
void flow_task1(void *pdata)
{		float temp_sonar;			  
 	while(1)
	{
		
	 if(ultra_distance==0)
	 temp_sonar=650;
	 else if(ultra_distance>4000)
	 temp_sonar=4000;
	 else
	 temp_sonar=ultra_distance;
	 float spd_temp[2];
	 flow_height_fliter=ALT_POS_SONAR2;//sonar_filter_bmp((float)ultra_distance/1000,0.01);
	 
	 //px4
		flow_gyrospeed[0] = flow_rad.integrated_xgyro / (float)flow_rad.integration_time_us * 1000000.0f;  
		flow_gyrospeed[1] = flow_rad.integrated_ygyro / (float)flow_rad.integration_time_us * 1000000.0f;  
		flow_gyrospeed[2] = flow_rad.integrated_zgyro / (float)flow_rad.integration_time_us * 1000000.0f;  
	  static u8 n_flow;
	  static float gyro_offset_filtered[3],att_gyrospeed_filtered[3],flow_gyrospeed_filtered[3];
		float flow_ang[2];
		if(flow_rad.integration_time_us){
		//if (fabs(mpu6050.Gyro_deg.y/57.3) < rate_threshold) {  
		if (fabs(flow_gyrospeed[0]) < rate_threshold) {  
		flow_ang[0] = (flow_rad.integrated_x / (float)flow_rad.integration_time_us * 1000000.0f) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		else {  
		//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)  
		flow_ang[0] = ((flow_rad.integrated_x - LIMIT(flow_rad.integrated_xgyro*k_gro_off,-fabs(flow_rad.integrated_x),fabs(flow_rad.integrated_x))) / (float)flow_rad.integration_time_us * 1000000.0f  
		+ gyro_offset_filtered[0]*0) * flow_k;//for now the flow has to be scaled (to small)  
		}  
//横向移动--1
		//if (fabs(mpu6050.Gyro_deg.x/57.3) < rate_threshold) {  
		if (fabs(flow_gyrospeed[1]) < rate_threshold) {  
		flow_ang[1] = (flow_rad.integrated_y/ (float)flow_rad.integration_time_us  * 1000000.0f) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		else {  
		flow_ang[1] = ((flow_rad.integrated_y- LIMIT(flow_rad.integrated_ygyro*k_gro_off,-fabs(flow_rad.integrated_y),fabs(flow_rad.integrated_y))) / (float)flow_rad.integration_time_us  * 1000000.0f  
		+ gyro_offset_filtered[1]*0) * flow_k;//for now the flow has to be scaled (to small)  
		}  
		float yaw_comp[2];

		yaw_comp[0] = - flow_module_offset_y * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  
		yaw_comp[1] = flow_module_offset_x * (flow_gyrospeed[2] - gyro_offset_filtered[2]);  

		/* flow measurements vector */  

		flow_m[0] = flow_ang[0];  
		flow_m[1] = -flow_ang[1] ;  
		
	
		if(flow_px_sel){
		x_flow_orign_temp=flow_m[1]*scale_px4_flow;
		y_flow_orign_temp=flow_m[0]*scale_px4_flow;		
		}else{
		x_flow_orign_temp=flow.flow_x.origin;
		y_flow_orign_temp=flow.flow_y.origin;		
		}
	}
		//x
		flow_rad_fix[0]=x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.x,dead_rad_fix[0])*flow_rad_fix_k[0];
		if(fabs(mpu6050.Gyro_deg.x)<dead_rad_fix[0]*0.7)
		flow_rad_fix[0]=x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.x,0)*flow_rad_fix_k2; 

		//y
		flow_rad_fix[1]=y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.y,dead_rad_fix[0])*flow_rad_fix_k[0];
		if(fabs(mpu6050.Gyro_deg.y)<dead_rad_fix[0]*0.7)
		flow_rad_fix[1]=y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.y,0)*flow_rad_fix_k2; 

		//z
		flow_rad_fix[2]=0;//x_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.z,dead_rad_fix[1])*flow_rad_fix_k[1];
		flow_rad_fix[3]=0;//y_flow_orign_temp*sign_flow(mpu6050.Gyro_deg.z,dead_rad_fix[1])*flow_rad_fix_k[1];
		flow_filter[0]=Moving_Median(18,10,my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0));
		flow_filter[1]=Moving_Median(19,10,my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0));

		flow_ground_temp[0]=flow_filter[0]*flow_height_fliter*scale_pix;
		flow_ground_temp[1]=flow_filter[1]*flow_height_fliter*scale_pix;
		//flow->3
		flow_ground_temp[2]=my_deathzoom_2(x_flow_orign_temp-flow_rad_fix[0]-flow_rad_fix[2],0)*flow_height_fliter*scale_pix;
		flow_ground_temp[3]=my_deathzoom_2(y_flow_orign_temp-flow_rad_fix[1]-flow_rad_fix[3],0)*flow_height_fliter*scale_pix;
		flow_use=flow_ground_temp[3]*cos(Roll*DEG_RAD)+flow_ground_temp[2]*sin(Roll*DEG_RAD);
		float a_br[3],tmp[3];	
		float acc_temp[3];
		a_br[0] =(float) mpu6050.Acc.x/4096.;//16438.;
		a_br[1] =(float) mpu6050.Acc.y/4096.;//16438.;
		a_br[2] =(float) mpu6050.Acc.z/4096.;//16438.;
		// acc
		tmp[0] = a_br[0];
		tmp[1] = a_br[1];
		tmp[2] = a_br[2];

		acc_temp[0] = tmp[1]*reference_vr_imd_down[2]  - tmp[2]*reference_vr_imd_down[1] ;
		acc_temp[1] = tmp[2]*reference_vr_imd_down[0]  - tmp[0]*reference_vr_imd_down[2] ;
//横向移动  -》0
		acc_neo[0]=((float)((int)((-acc_temp[0])*200)))/200*9.87;
		acc_neo[1]=((float)((int)((-acc_temp[1])*200)))/200*9.87;
		acc_neo[2]=((float)((int)((acc_temp[2]-1.0f)*200)))/200*9.87;	
		
		FlowUkfProcess(0);
		float temp_spd[2];
		temp_spd[0]=Moving_Median(16,MID_CNT_SPD,FLOW_VEL_X);//ALT_VEL_BMP+acc_neo[0]*k_acc_forward;
		temp_spd[1]=Moving_Median(17,MID_CNT_SPD,FLOW_VEL_Y);//ALT_VEL_SONAR+acc_neo[1]*k_acc_forward;
		imu_nav.flow.speed.x=imu_nav.flow.speed.x*(1-k_flp)+k_flp*temp_spd[0];
		imu_nav.flow.speed.y=imu_nav.flow.speed.y*(1-k_flp)+k_flp*temp_spd[1];
		delay_ms(10); 
	}
}	


//=======================DEBUG串口 任务函数===========================
OS_STK  UART_TASK_STK[UART_STK_SIZE];
u8 UART_UP_LOAD_SEL=4;//<------------------------------UART UPLOAD DATA SEL
u8 DEBUG_SEL=0;
void uart_task(void *pdata)
{	static u8 cnt[4];					 		
 	while(1)
	{
	if(cnt[1]++>1){cnt[1]=0;			
								if(!GOL_LINK_BUSY[0]){
								GOL_LINK_BUSY[1]=1;
								switch(UART_UP_LOAD_SEL)
								{
								case 0://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow_rad.integrated_y*100, flow_rad.integrated_ygyro*100,
								0,flow.flow_x.origin,flow.flow_y.origin,
								0,flow_m[0]*1000,flow_m[1]*1000);break;
								case 1://海拔速度
								Send_BLE_DEBUG((int16_t)(0),flow.flow_x.origin,flow.flow_y.origin,
								0,ALT_VEL_SONAR*1000,flow_ground_temp[3]*1000,
								0,ALT_VEL_BMP*1000,flow_ground_temp[2]*1000);break;
								case 2://海拔速度
								Send_BLE_DEBUG((int16_t)(0),x_flow_orign_temp,0,
								0,flow.flow_x.origin,0,
								0,0,flow.flow_y.origin);break;
								case 3://海拔速度
								Send_BLE_DEBUG((int16_t)(baroAlt),ALT_POS_BMP*1000,ALT_POS_BMP_EKF*1000,
								0,-ALT_VEL_BMP*100,ALT_VEL_BMP_EKF*100,
								0,acc_z_view[1],acc_z_view[0]);break;		
								case 4://海拔速度
								Send_BLE_DEBUG((int16_t)(0),NAV_VEL_E*1000,0,
								0,0*100,Global_GPS_Sensor.NED_Vel[0]*1000,
								0,0,v_test[1]*1000);break;		
								}				
								GOL_LINK_BUSY[1]=0;		
								}   
								//RC_OUT[0],RC_OUT[1],RC_OUT[2]
								if(state_use_m100)
									Yaw=m100_yaw;
					switch(DEBUG_SEL){
						case 0:                                                                  									 //AVOID 
								 UART2_ReportMotion(ERR_GPS[0],ERR_GPS[1],0  ,HEIGHT,EX_HEIGHT,RC_OUT[2]  ,ALT_POS_SONAR2*1000,ALT_POS_SONAR_HEAD_LASER_SCANER,EX_AVOID);
								 UART2_ReportIMU( Yaw*10,Pitch*10,Roll*10,0,state_fc*10,state_m100*10,m100_refresh*10000);
						break;
						case 1:
								 UART2_ReportMotion(0,flow_ground_temp[2]*100,flow_ground_temp[3]*100,0,imu_nav.flow.speed.x*100,0,0,imu_nav.flow.speed.y*100,flow_use*100);
								 UART2_ReportIMU( Yaw*10,Pitch*10,Roll*10,0,state_fc*10,0,0);
						break;
						case 2:
								 UART2_ReportMotion(0,flow_ground_temp[2]*100,flow_ground_temp[3]*100,0,imu_nav.flow.speed.x*100,0,0,imu_nav.flow.speed.y*100,ALT_VEL_BMP_EKF*1000);
								 UART2_ReportIMU( Yaw*10,Pitch*10,Roll*10,0,state_fc*10,0,0);
						break;
					}
				}					
		delay_ms(5);  
	}
}	

//=======================故障保护 任务函数==================
OS_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *pdata)
{							  
 	while(1)
	{ 
		flow.rate=flow.flow_cnt/0.2;
	  flow.flow_cnt=0;
		laser_cnt[0]++; 
		if(laser_cnt[0]>5)
			laser_check[0]=0;
		if(laser_cnt[1]>5)
			laser_check[1]=0;
		delay_ms(200); 
	}
}	

//------------------------------软件定时器--------------------------------//
OS_TMR   * tmr1;			//软件定时器1
OS_TMR   * tmr2;			//软件定时器2
OS_TMR   * tmr3;			//软件定时器3

//软件定时器1的回调函数	  OSCPUUsage
//每100ms执行一次,用于显示CPU使用率和内存使用率		
 u16 cpuusage=0;
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	static u8 tcnt=0;	    

	if(tcnt==5)
	{
 		
		cpuusage=0;
		tcnt=0; 
	}
	cpuusage+=OSCPUUsage;
	tcnt++;				    
}

//软件定时器2的回调函数				  	   
void tmr2_callback(OS_TMR *ptmr,void *p_arg) 
{	
static u8 cnt;
	Mode_FC();
	LEDRGB_STATE();
}
//软件定时器3的回调函数				  	   
void tmr3_callback(OS_TMR *ptmr,void *p_arg) 
{	
 
} 


//
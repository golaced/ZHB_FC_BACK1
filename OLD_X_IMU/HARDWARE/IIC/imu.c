
#include "imu.h"
#include "include.h"
#include "hml5833l.h"
#include "my_math.h"
#include "filter.h"

float Kp =0.6f;//2.25f;//0.6f   ;             	// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki =0.1f    ;            	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float q_nav[4];
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
//xyz_f_t Gravity_Vec;  				//�������������
	
float Roll,Pitch,Yaw;    				//��̬��
float Roll_mid_down,Pitch_mid_down,Yaw_mid_down;    				//��̬��
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

int test_flag[3]={1,-1,1};

//use
float mag_norm ,mag_norm_xyz, yaw_mag_view[4];
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	
	mag_norm_tmp = 20 *(6.28f *half_T);	
	
	mag_norm_xyz = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z);
	
		if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
//		mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
//		mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
//		mag_tmp.z += mag_norm_tmp *( (float)-ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}

	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);
	
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag_view[1] = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;
		
	}
	  float calMagY,calMagX,magTmp2[3],euler[3];
	magTmp2[0]=test_flag[0]*ak8975.Mag_Val.x;
	magTmp2[1]=test_flag[1]*ak8975.Mag_Val.y;
	magTmp2[2]=test_flag[2]*ak8975.Mag_Val.z;
	euler[1]=Pitch/RAD_DEG  ;
	euler[0]=Roll/RAD_DEG   ;
		calMagY = magTmp2[2] * sin(euler[0]) - magTmp2[1] * cos(euler[0]); //��б���������Ƶ�Y�����
		calMagX = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1]) * sin(euler[0]) + magTmp2[2] * sin(euler[1]) * cos(euler[0]); //��б���������Ƶ�X�����
	//calMagY = mz * sin(sita) + my * cos(sita); //��б���������Ƶ�Y�����
 // calMagX = mx * cos(fi) + my * sin(fi) * sin(sita) - mz * sin(fi) * cos(sita); //��б���������Ƶ�X�����
	
		yaw_mag_view[0] = fast_atan2(calMagY, calMagX) * RAD_DEG; //����Yaw (-PI < Roll < PI) ��������ת���ɽǶ�
	yaw_mag_view[3]=yaw_mag_view[0]/2+yaw_mag_view[1]/2;
	yaw_mag=yaw_mag_view[1] ;
//		euler_view[1]=euler[1]=PitchR;//*= RAD_DEG; //������ת��Ϊ�Ƕ�
//		euler_view[0]=euler[0]=RollR;// *= RAD_DEG; //������ת��Ϊ�Ƕ�
	
	//=============================================================================
	// �����Ч��������//ʮ����Ҫ
	if(mode.en_imu_ekf==0){
	reference_vr[0]=reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1]=reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2]=reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);}//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]
	//reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	//reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	//reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������       
	//=============================================================================


	// ������ٶ�������ģ
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   
	norm_acc_lpf +=  NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//�ѼӼƵ���ά����ת�ɵ�λ������
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* ��˵õ���� */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	    //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* ����ͨ */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
	//			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//				ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	/* ������ */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* �����޷� */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
	if( reference_v.z > 0.0f )
	{
		if( fly_ready||(fabs(Pitch)>10)||(fabs(Roll)>10)  )
		{
	//	yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - YAW_R);
			yaw_correct = Kp *0.1f *LIMIT( my_deathzoom( To_180_degrees(yaw_mag - Yaw), 10),-20,20 );
			//�Ѿ�������ֻ��Ҫ���پ�����
		}
		else
		{
			yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw);
			//û�н�������������ʱ�̣����پ���
		}
// 		if( yaw_correct>360 || yaw_correct < -360  )
// 		{
// 			yaw_correct = 0;
// 			//���ƾ�����Χ+-360�����+-180��ȡֵ����
// 		}
	}

	
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* �ò���������PI����������ƫ */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* ��Ԫ����һ�� normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	q_nav[0]=ref_q[0] = ref_q[0] / norm_q;
	q_nav[1]=ref_q[1] = ref_q[1] / norm_q;
	q_nav[2]=ref_q[2] = ref_q[2] / norm_q;
	q_nav[3]=ref_q[3] = ref_q[3] / norm_q;
	

	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
// 				//Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] + ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw = yaw_mag;
}


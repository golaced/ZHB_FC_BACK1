#ifndef CommonConversions_H_
#define CommonConversions_H_

#include "include.h"

typedef float real_T;
typedef double real64_T;
typedef int8_t int8_T;
typedef int32_t int32_T;
// ��Ԫ��ת����һ����������NED����ϵ��ת����������ϵ�ľ���
void Quaternion2Tnb(const float q[4], float Tnb[3][3]);

// ��Ԫ��ת����һ���������ɻ�������ϵ��ת��NED����ϵ�ľ���
void Quaternion2Tbn(const float q[4], float Tbn[3][3]);

// ����Ԫ��ת��ŷ����
void Quaternion2Euler(const float q[4], float euler[3]);

// ��ŷ����ת����Ԫ��
void Euler2Quaternion(const float euler[3], float q[4]);

// ��һ��������A����ϵ��ת��B����ϵ
void A_Fixed2B_Fixed(float TAB[3][3], float V[3]);

// ����Ԫ��������ο�ϵ��������ת��NED����ϵ��
void Quaternion2NED(const float q[4], float V_NED[3]);

void Cal_Vel_NED(double velNED[3], double gpsCourse, double gpsGndSpd, double gpsVelD);

//����γ��ת����NED
void Cal_Pos_NED(double posNEDr[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference);

// ��3*3����������
void Matrix_3X3_Inv(const real_T a[9], real_T c[9]);

// ��4*4����������
void Matrix_4X4_Inv(const real_T x[16], real_T y[16]);

// ��6*6����������
void Matrix_6X6_Inv(const real_T x[36], real_T y[36]);

// ��7*7����������
void Matrix_7X7_Inv(const real_T x[49], real_T y[49]);

// ��10*10����������
void Matrix_10X10_Inv(const real_T x[100], real_T y[100]);

// ��ת�þ���a  c=a' line Ϊ a������ rowΪa������
void Matrix_Tran(const real_T *a, real_T *c, int line, int row);

// ������� ��Ҫ���� a���� �� b���� a������ al ���� ar b����� ���� bl ����br (a b ������һά����)
void ML_R_X_ML_R(const real_T *a, const real_T *b, real_T *c, int al, int ar, int bl, int br);

int inv(float *p,int n);

#endif

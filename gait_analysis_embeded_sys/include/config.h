#ifndef __CONFIG_H__
#define __CONFIG_H__

#define PI 3.1415926

// //对于无用的数据不予处理
// #define SHRINK
#define MAX_ARRAY_SIZE 600

#define SPORT_FACTOR_WALK 0.8214
#define SPORT_FACTOR_RUN 1.036

//取样频率及周期
#define FRQ 100
#define T (1.0f/FRQ)

//输入文件地址
#define FILE_PATH "../data/040.txt"
//左右脚定义
#define LEFT_FOOT
// #define RIGHT_FOOT

//滤波器信息
#define G 9.81
#define MEAN_WINDOWSIZE 4
//还原真实值
#define ACC_RATE (8192 / 9.81)
#define GYRO_RATE (57.3 * 16.4)
//卡尔曼滤波器输入值
#define Kalman_Q 1e-6
#define Kalman_R 1e-5
#define Kalman_X0 -1
#define Kalman_P0 1
// 这里的KpKi是用于调整加速度计修正陀螺仪的速度
#define Kp 100.0f 
#define Ki 0.002f

//取点阈值
//研究pitch图像时,高峰值相关阈值
#define PITCH_PEAKS_DIS 30 //相邻两点之间最短距离
#define PITCH_PEAKS_LOWBOUND 0.44 //最小峰值
#define PITCH_PEAKS_UPBOUND 2 //最大峰值
//研究pitch图像时,矮峰值相关阈值
#define PITCH_TINYPEAKS_DIS 30 //相邻两点之间最短距离
#define PITCH_TINYPEAKS_LOWBOUND 0.03 //最小峰值
#define PITCH_TINYPEAKS_UPBOUND 0.44 //最大峰值
//研究pitch图像时,谷值相关阈值
#define PITCH_VALLEIES_DIS 30 //相邻两点之间最短距离
#define PITCH_VALLEIES_LOWBOUND -10 //最小谷值
#define PITCH_VALLEIES_UPBOUND -0.3 //最大谷值
//研究pitch图像时,近零点相关阈值
#define NEARZERO_CONTIUNES_MINNUM 10 //连续性最小值
#define NEARZERO_LOWBOUND -0.1  //下限
#define NEARZERO_UPBOUND 0.2    //上限

//由于数据存在部分损坏，所以将利用局部数据对全部数据进行估计，
//但是仍要设定一个真实比例(0~1)
#define CONFIDENCE_RATE 0.985

/*******************************/
typedef char                CHAR;

typedef unsigned char       INT8U;
typedef signed   char       INT8S;

typedef unsigned short      INT16U;
typedef signed   short      INT16S;

typedef unsigned long       INT32U;
typedef signed   long       INT32S;

typedef unsigned long long  INT64U;
typedef signed   long long  INT64S;

typedef float               F32;
typedef double              F64;

typedef unsigned char       U8;
typedef signed   char       S8;

typedef unsigned short      U16;
typedef signed   short      S16;

typedef unsigned long       U32;
typedef signed   long       S32;
/*******************************/
#endif
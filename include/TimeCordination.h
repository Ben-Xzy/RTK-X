#pragma once
#include"math.h"
#include"string.h"
#include<iostream>
#include"Matrix.h"

#ifndef TIMECORDINATION_H
#define TIMECORDINATION_H
#define PI 3.1415926535897932 
#define eSquared  0.00669437999013 //第一偏心率平方 
#define A1 6378137.0//半长轴
#define rad PI/180.0
#define deg 180.0/PI
struct JDTIME ;
struct COMMONTIME ;
struct GPSTIME ;
struct XYZ ;
struct BLH ;
struct NEU ;//先将类空定义一次防止后面因类名未出现导致报错

struct JDTIME
	/*简化儒略日*/
{
	int Days;
	double FracDay;
	int MJDDays;
	double MJDFracDay;
	JDTIME()
	{
		Days = 0;
		FracDay = 0.0;
		MJDDays = 0;
		MJDFracDay = 0.0;
	}
};
struct COMMONTIME
	/*通用时*/
{
	short Year;
	int Month;
	int Day;
	int Hour;
	int Minute;
	double Second;
	
};
struct GPSTIME
	/*GPS时*/
{
	int Week;
	double SecOfWeek;
	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	}
};
struct ENU 
/*测站地平坐标系*/
{
	XMatrix PosMat;
	XMatrix PosStation;//基站坐标矩阵
	double dEnu[3];
	double Elev;
	double Azim;
	bool Valid;
	ENU(double xyz[], double station[])
	{
		PosMat.MatrixResize(3, 1);
		PosStation.MatrixResize(3, 1);//定义为列向量
		for (int i = 0; i < 3; i++)
		{
			PosMat.matrix[i] = xyz[i];
			PosStation.matrix[i] = station[i];
		}
		Valid = true;
	}
	ENU() { Valid=false; }

};
void BDST2GPST(GPSTIME* GPS, GPSTIME* BDS);
void GPST2BDST(GPSTIME* BDS, GPSTIME* GPS);
void CompSatElAz(ENU* enu);
void CompEnudPos(ENU* enu);
void Blh2Xyz(double* blh, double* xyz);
void Xyz2Blh(double* xyz, double* blh);
void GPST2JD(GPSTIME* GPST, JDTIME* JD);
void Com2JD(COMMONTIME* COM, JDTIME* JD);
void JD2Com(JDTIME* JD, COMMONTIME* COM);
void JD2GPST(JDTIME* JD, GPSTIME* GPS);
void FromJDGetMJD(JDTIME* JD);
void FromMJDGetJD(JDTIME* JD);
double GPSTMius(GPSTIME* T1, GPSTIME* T2);
#endif // !TIMECORDINATION_H
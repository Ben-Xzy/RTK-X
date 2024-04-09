#ifndef TIMECORDINATION_H
#define TIMECORDINATION_H
#include"math.h"
#include"string.h"
#include<iostream>
#include"Matrix.h"


#define PI 3.1415926535897932 
#define eSquared  0.00669437999013 //The first eccentricity is squared 
#define A1 6378137.0//Semi-major axis
#define rad PI/180.0
#define deg 180.0/PI
struct JDTIME ;
struct COMMONTIME ;
struct GPSTIME ;
struct XYZ ;
struct BLH ;
struct NEU ;//First, define the class as empty once to prevent errors caused by the class name not appearing

struct JDTIME
	/*Simplification of Julian Day*/
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
	/*General Purpose*/
{
	short Year;
	int Month;
	int Day;
	int Hour;
	int Minute;
	double Second;
	
};
struct GPSTIME
	/*GPS time*/
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

{
	XMatrix PosMat;
	XMatrix PosStation;//Base station coordinate matrix
	double dEnu[3];
	double Elev;
	double Azim;
	bool Valid;
	ENU(double xyz[], double station[])
	{
		PosMat.MatrixResize(3, 1);
		PosStation.MatrixResize(3, 1);//Defined as a column vector
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

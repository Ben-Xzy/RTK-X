#ifndef MATHF_H
#define MATHF_H
#include<iostream>
#include"DataClassSet.h"
#include"Matrix.h"
#include"param.h"

/*********************
The header file of general mathematical operations, including lambda methods and rotation matrices, etc., is mainly the basic mathematical functions involved in SPP and RTK
*********************/
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D,
    const double* zs, double* zn, double* s);
static int LD(int n, const double* Q, double* L, double* D);
int lambda(int n, int m, double* a,const double* Q, double* F,
    double* s);
void inputRDot(XMatrix& RDot, double OMEGAk, double ik, double orbit_xk, double orbit_yk);
void inputRx(XMatrix& Rx, double phiX);
void inputRz(XMatrix& Rz, double phiZ);
void inputRzDot(XMatrix& RzDot, double Coe, double tk);
void CalRho(double pos_r[], double pos_s[], double& rho);
void LSQCalx(XMatrix B, XMatrix P, XMatrix W, XMatrix& x, XMatrix& Q);
void LSQCalPrCis(LSQ& ls, double& theta, double& PDOP, int model);
template <typename T, typename T2>/*The template function header file cannot be separated from the source file, so write it together*/
void CalMWGFPIF(T& res, T2& value)
{
	switch (value.System)
	{
	case GPS:
	{
		res.MW = (GPS_L1 * value.L[0] - GPS_L2 * value.L[1]) / (GPS_L1 - GPS_L2) -
			(GPS_L1 * value.P[0] + GPS_L2 * value.P[1]) / (GPS_L1 + GPS_L2);
		res.PIF = (GPS_L1 * GPS_L1 * value.P[0] - GPS_L2 * GPS_L2 * value.P[1]) / (GPS_L1 * GPS_L1 - GPS_L2 * GPS_L2);
		break;
	}
	case BDS:
	{
		res.MW = (BDS_B1I * value.L[0] - BDS_B3I * value.L[1]) / (BDS_B1I - BDS_B3I) -
			(BDS_B1I * value.P[0] + BDS_B3I * value.P[1]) / (BDS_B1I + BDS_B3I);
		res.PIF = (BDS_B1I * BDS_B1I * value.P[0] - BDS_B3I * BDS_B3I * value.P[1]) / (BDS_B1I * BDS_B1I - BDS_B3I * BDS_B3I);
		break;
	}
	default:break;
	}
	res.GF = value.L[0] - value.L[1];
}
#endif
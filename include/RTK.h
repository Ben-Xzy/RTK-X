#ifndef RTK_H
#define RTK_H
#include"DataClassSet.h"
#include<iostream>
#include<fstream>
#include<ostream>
#include"param.h"
#include"output.h"
#include"sockets.h"

using namespace std;
/*****************    RTK main Func  *********************/
int RtkObsSyn(FILE* fb, FILE* fr, RTKDATA* rawdata, POSRES* BasPos, POSRES* RovPos);
int RtkObsSyn(SOCKET& bIp, SOCKET& rIp, RTKDATA* rawdata, POSRES* BasPos, POSRES* RovPos);
void CalStaSinDif(EPOCHOBSDATA* BasEpk, EPOCHOBSDATA* RovEpk, POSRES* BasPos, POSRES* RovPos, SDEPOCHOBS* SdObs);
void DTSinDifCySlip(EPOCHOBSDATA* BasEpk, EPOCHOBSDATA* RovEpk, SDEPOCHOBS* SdObs);
void CalStaDouDif(EPOCHOBSDATA* RovEpk, SDEPOCHOBS* SdObs, DDCEPOCHOBS* DDObs);
bool RTK(EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs, POSRES *RovPos, POSRES* BasPos, DDCEPOCHOBS* DDObs);
bool RTKFixed(EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs, POSRES* RovPos, DDCEPOCHOBS* DDObs, RtkAlignData& RAlign, RtkAlignData BAlign);
void EKFinitial(RTKEKF* e, DDCEPOCHOBS* DDObs, POSRES* Sppr, XMatrix& ekfP);
void EKF(RTKEKF* e, DDCEPOCHOBS* d, POSRES* r, POSRES* b, XMatrix& P, EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs);
void TwiceUpdate(XMatrix& x_1, XMatrix& P, double fixedN[]);

/*****************   RTK script Func(tool)    *********************/
void RtkAlignDataIni(RtkAlignData* a);
void CalStaSatDis(double Pos_r[], EPOCHOBSDATA* Obs, DDCOBS* DDObs, RtkAlignData* DisData);
int CalRefDis(double Pos_r[], EPOCHOBSDATA* Obs, int RefPrn[], RtkAlignData* Data);
void AmbigInitial(EPOCHOBSDATA* Obs, POSRES* Res);
void RtkBElement(double Pos_r[], double TarSatPos[], double RefSatPos[], double TarDis, double RefDis, double vec[]);
void RtkInputB(RtkAlignData BasSatData, RtkAlignData RovSatData, double RovPos[], SATMIDRES BasSatPos[], XMatrix& B_r, int Model);
void RtkInputP(int GPSDDNum, int BDSDDNum, XMatrix& P_r, int Model);
void RtkWElement(double Rirho, double Rjrho, double Birho, double Bjrho, double& DDrho);
void RtkInputW(RtkAlignData BasSatData, RtkAlignData RovSatData, DDCEPOCHOBS* DDObs, XMatrix& W_r, int model);
void GetDDNSet(double DDNSet[], DDCEPOCHOBS* DDObs);
void CalZeroLine(double B[], double R[], double l[], double enu[]);
void CalRatio(double Fixedrms[], double& ratio);
void CalSatScore(SATMIDRES p, SATOBSDATA d, double& s, int* f,DDCEPOCHOBS o);
int searchSamPrn(int d, int s[], int b,int n);
void inputPhi(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& Phi, GNSSSys s);
void consEKFPhi(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& Phi);
void updateE(RTKEKF* e, DDCEPOCHOBS* d);
void consEKFQ(DDCEPOCHOBS* d, XMatrix& Q);
void chkDDSlip(SDSATOBS* rs, SDSATOBS* ts, DDCOBS* d);
void EKFParryReIni(int i, int c, XMatrix& P, GNSSSys sys);
void consEKFP(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& P, XMatrix& Q, XMatrix& Phi);
void RTKInputL(XMatrix B, XMatrix& L, POSRES* r, RtkAlignData BasSatData, RtkAlignData RovSatData, DDCEPOCHOBS* DDObs);
void consEKFR(XMatrix& R, DDCEPOCHOBS* d);
void calEKFK(XMatrix& K, XMatrix P, XMatrix H, XMatrix R);
void UpdateX(XMatrix x, XMatrix& x_1, XMatrix Phi, DDCEPOCHOBS* d);
void DDReini(DDCEPOCHOBS* d);
void EkfPGetQnn(XMatrix P, double Qnn[]);
void EkfXGetfN(XMatrix x, double fN[]);
void RecordPrn(DDCEPOCHOBS* o, int r[], GNSSSys s[]);
bool SearchPrn(DDCEPOCHOBS d, int Prn, GNSSSys Sys);
#endif
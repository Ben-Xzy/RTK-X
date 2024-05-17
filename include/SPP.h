#ifndef SPP_H
#define SPP_H
#include <iostream>
#include <cmath>
#include <string>
#include"DataClassSet.h"
#include"Matrix.h"
#include"ReadBinary.h"
#include"TimeCordination.h"
#include"param.h"
#include"output.h"

/*****************   SPP main Func   *********************/
bool CompSatClkOff(const GPSTIME* t, GPSEPHREC* RealEph, SATMIDRES* Mid, bool flag);
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC*Eph, SATMIDRES* Mid);
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC*Eph, SATMIDRES* Mid); 
double Hopfield(double H, double Elev);
void DetectOutlier(EPOCHOBSDATA* Obs);
void ComputeSatPVTAtSignalTrans(EPOCHOBSDATA* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[],bool flag);
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);
bool SPV(EPOCHOBSDATA* Epoch, POSRES* Res);

/*****************   SPP script Func(tool)   *********************/
bool AlignEphObs(SATOBSDATA* EachObs, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, GPSEPHREC* OutPutEph);
bool JudgeEphEffect(GNSSSys sys, GPSTIME obsTime, GPSEPHREC RealEph);
double Countvk(double e, double  Ek);
void SPPBelement(double pos_r[], double pos_s[], double vec[]);
void SPPintputB(XMatrix& B, EPOCHOBSDATA* Epoch, double pos_r[]);
void SPPinputW(XMatrix& W, EPOCHOBSDATA* Epoch, double pos_r[]);
void SPVInputW(XMatrix& W, EPOCHOBSDATA* Epoch, POSRES* Res);
void SPVInputB(XMatrix& B, EPOCHOBSDATA* Epoch, POSRES* Res);
double SPVWElement(SATOBSDATA Satobs, SATMIDRES SatPvT, POSRES* Res);
void InputP(int Prow, EPOCHOBSDATA* Epoch, bool flag, XMatrix& P);
void EarthRotateCorrect(GNSSSys sys, SATMIDRES* Satpos, double Pos[]);
void DTCycleSlipIni(SATOBSDATA* LatObs,double Formlocktime[],bool &errFlag);
#endif
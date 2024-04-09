#include"Matrix.h"
#include"ReadBinary.h"
#include"SPP.h"
#include<ostream>
#include<windows.h>
#include"mathf.h"
using namespace std;
bool AlignEphObs(SATOBSDATA* EachObs, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, GPSEPHREC* OutPutEph)
{
	bool flag = false;
	if (EachObs->System == GPS)
	{
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			if (EachObs->Prn == GPSEph[i].PRN)
			{
				if (GPSEph[i].SVHealth != 0)
				{
					flag = false;
					break;
				}
				memcpy(OutPutEph, GPSEph + i, sizeof(GPSEPHREC));
				flag = true;
				break;
			}
		}
	}
	else if (EachObs->System == BDS)
	{
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (EachObs->Prn == BDSEph[i].PRN)
			{
				if (BDSEph[i].SVHealth != 0)
				{
					flag = false;
					break;
				}
				memcpy(OutPutEph, BDSEph + i, sizeof(GPSEPHREC));
				flag = true;
				break;
			}
		}
	}
	return flag;
}
bool JudgeEphEffect(GNSSSys sys, GPSTIME obsTime, GPSEPHREC RealEph)
{
	GPSTIME UniT;
	if (sys == GPS)
	{
		memcpy(&UniT, &obsTime, sizeof(GPSTIME));
	}
	else if (sys == BDS)
	{
		BDST2GPST(&UniT, &obsTime);
	}
	double DT = GPSTMius(&UniT, &RealEph.TOE);
	if (fabs(DT) > 7500)//不能管时间先后，故取绝对值，其要求小于两小时加300s
	{
		//cout << "星历数据过期" << endl;
		return false;
	}
	else return true;
}
double Countvk(double e, double  Ek)
{
	double Modecule = sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek));
	double Denominator = (cos(Ek) - e) / (1 - e * cos(Ek));
	return atan2(Modecule, Denominator);
}
void SPPBelement(double pos_r[], double pos_s[], double vec[])
/*计算B矩阵里面特定元素*/
{
	double rho = 0;/*初始化rho*/
	CalRho(pos_r, pos_s, rho);
	for (int i = 0; i < 3; i++)
	{
		vec[i] = (pos_r[i] - pos_s[i]) / rho;
	}
}
void SPPintputB(XMatrix& B_re, EPOCHOBSDATA* Epoch, double pos_r[])
{
	XMatrix B;
	int countBDS = 0;
	int countGPS = 0;
	int Bindex = 0;
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (!Epoch->SatPvT[i].Valid)//无效卫星PVT直接跳过
		{
			continue;
		}
		double vec[3];
		GNSSSys Sys = Epoch->Satobs[i].System;
		SPPBelement(pos_r, Epoch->SatPvT[i].SatPos, vec);
		B(Bindex, 0) = vec[0]; B(Bindex, 1) = vec[1]; B(Bindex, 2) = vec[2];
		if (Sys == BDS) { B(Bindex, 3) = 0; B(Bindex, 4) = 1; countBDS++; }
		else if (Sys == GPS) { B(Bindex, 3) = 1; B(Bindex, 4) = 0; countGPS++; }
		Bindex++;
	}
	/*矩阵重构*/
	if (countBDS == 0)
	{
		B.MatrixResize(Bindex, 4);
		//B.MatrixDis();
	}
	else if (countGPS == 0)
	{
		for (int j = 0; j < Bindex; j++)
		{
			B(j, 3) = 1;
			B(j, 4) = 0;
		}
		B.MatrixResize(Bindex, 4);
	}
	B_re = B;
}
void SPPinputW(XMatrix& W_r, EPOCHOBSDATA* Epoch, double pos_r[])
{
	int WIndex = 0;
	XMatrix W;
	/*用的是消电离层组合，所以GPS无TGD，BDS用TGD1*/
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (!Epoch->SatPvT[i].Valid)//无效卫星PVT直接跳过
		{
			continue;
		}
		double pos_s[3],BlhPos[3];
		GNSSSys Sys = Epoch->Satobs[i].System;
		memcpy(pos_s, Epoch->SatPvT[i].SatPos, 3 * sizeof(double));
		double rho = sqrt(pow(pos_r[0] - pos_s[0], 2) + pow(pos_r[1] - pos_s[1], 2) + pow(pos_r[2] - pos_s[2], 2));
		if (pos_r[0] == InitialValue && pos_r[1] == InitialValue && pos_r[2] == InitialValue)
		{
			Epoch->SatPvT[i].TropCorr = 0;
		}
		else {
			Xyz2Blh(pos_r, BlhPos);
			ENU enuPos(pos_s, pos_r);
			CompSatElAz(&enuPos);
			Epoch->SatPvT[i].Elevation = enuPos.Elev/PI*180.0;
			Epoch->SatPvT[i].Azimuth = enuPos.Azim / PI * 180.0;
			Epoch->SatPvT[i].TropCorr = Hopfield(BlhPos[2],Epoch->SatPvT[i].Elevation);
		}
		if (Sys == BDS)
		{
			double Coe = BDS_B1I * BDS_B1I / (BDS_B1I * BDS_B1I - BDS_B3I * BDS_B3I);
			W(WIndex, 0) = Epoch->ComObs[i].PIF - (rho - CLight * Epoch->SatPvT[i].SatClkOft + Epoch->SatPvT[i].TropCorr + CLight * Coe * Epoch->SatPvT[i].Tgd1);/*北斗要加群延改正*/
		}
		else if (Sys == GPS)
		{
			W(WIndex, 0) = Epoch->ComObs[i].PIF - (rho - CLight * Epoch->SatPvT[i].SatClkOft + Epoch->SatPvT[i].TropCorr);/*GPS不加群延改正*/
		}
		WIndex++;
	}
	W_r = W;
}
double SPVWElement(SATOBSDATA Satobs, SATMIDRES SatPvT, POSRES* Res)
{
	double* Pos_r = Res->Pos;
	double* Pos_s = SatPvT.SatPos;
	double rho = 0;
	CalRho(Pos_r, Pos_s, rho);
	double rho_dot = 0;
	for (int i = 0; i < 3; i++)
	{
		rho_dot = rho_dot + (Pos_s[i] - Pos_r[i]) * SatPvT.SatVel[i] / rho;
	}
	double wElement = Satobs.D[0] - (rho_dot - CLight * SatPvT.SatClkSft);
	return wElement;
}
void SPVInputW(XMatrix& W_r, EPOCHOBSDATA* Epoch, POSRES* Res)
{
	XMatrix W;
	int WIndex = 0;
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (!Epoch->SatPvT[i].Valid)//无效卫星PVT直接跳过
		{
			continue;
		}

		W(WIndex, 0) =SPVWElement(Epoch->Satobs[i], Epoch->SatPvT[i], Res);
		WIndex++;
	}
	W_r = W;
}
void SPVInputB(XMatrix& B_r, EPOCHOBSDATA* Epoch, POSRES* Res)
{
	XMatrix B;
	double* pos_r = Res->Pos;
	int Bindex = 0;
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		if (!Epoch->SatPvT[i].Valid)//无效卫星PVT直接跳过
		{
			continue;
		}
		double vec[3];
		GNSSSys Sys = Epoch->Satobs[i].System;
		SPPBelement(pos_r, Epoch->SatPvT[i].SatPos, vec);
		B(Bindex, 0) = vec[0]; B(Bindex, 1) = vec[1]; B(Bindex, 2) = vec[2];
		B(Bindex, 3) = 1;
		Bindex++;
	}
	B_r = B;
}
void InputP(int Prow, EPOCHOBSDATA* Epoch, bool flag,XMatrix &P_r)
{
	XMatrix P;
	int PIndex = 0;
	int Cor_A = 0.5;
	int Cor_B = 4;
	int Sig0 = 1;
	if (!flag)
	{
		for ( ; PIndex < Prow; PIndex++)
		{
			P(PIndex, PIndex) = 1;
		}
	}
	else {
		for (int i = 0; i < Epoch->SatNum; i++)
		{
			if (Epoch->SatPvT[i].Valid)
			{
				P(PIndex, PIndex) = 1/(Cor_A + Cor_B *cos( Epoch->SatPvT[i].Elevation)*rad);
				PIndex++;
			}
		}
	}
	P_r = P;
}
void EarthRotateCorrect(GNSSSys sys, SATMIDRES* Satpos, double Pos[])
{
	XMatrix Rz(3, 3), VecPos(3, 1), VecVel(3, 1), ResP(3, 1), ResV(3, 1);
	double OMEGA = (sys == BDS) ? BDS_OMEGAEARTH : GPS_OMEGAEARTH;
	double rho = sqrt(pow(Satpos->SatPos[0] - Pos[0], 2) + pow(Satpos->SatPos[1] - Pos[1], 2) + pow(Satpos->SatPos[2] - Pos[2], 2));
	double deltaT = rho / CLight;
	inputRz(Rz, OMEGA * deltaT);
	VecPos(0, 0) = Satpos->SatPos[0]; VecVel(0, 0) = Satpos->SatVel[0];
	VecPos(1, 0) = Satpos->SatPos[1]; VecVel(1, 0) = Satpos->SatVel[1];
	VecPos(2, 0) = Satpos->SatPos[2]; VecVel(2, 0) = Satpos->SatVel[2];
	ResP = Rz * VecPos;
	ResV = Rz * VecVel;
	Satpos->SatPos[0] = ResP(0, 0); Satpos->SatVel[0] = ResV(0, 0);
	Satpos->SatPos[1] = ResP(1, 0); Satpos->SatVel[1] = ResV(1, 0);
	Satpos->SatPos[2] = ResP(2, 0); Satpos->SatVel[2] = ResV(2, 0);
}
/***************
利用观测数据里面给出的质量标准先初步判断周跳：
FormLocktime:前历元下单个卫星跟踪时长
LatObs:后历元下单个卫星观测数据
****************/
void DTCycleSlipIni(SATOBSDATA* LatObs)
{

	double FormLocktime[2];
	memcpy(FormLocktime, LatObs->FormLocktime, 2 * sizeof(double));
	for (int i = 0; i < 2; i++)
	{
		if (LatObs->fFlag[i] == 0) continue;/*只有fFlag为1时才能继续执行下去*/
		/*先针对locktime*/
		if (LatObs->locktime[i] < 6) {
			LatObs->fFlag[i] = -1;
			continue;
		};
		double dt = LatObs->locktime[i] - FormLocktime[i];
		if (dt < 0) LatObs->fFlag[i] = -1;
		/*再看Parity,判断半周情况*/
		if (LatObs->Parity[i] == 0)
		{
			LatObs->fFlag[i] = -1;
		}
		else { LatObs->fFlag[i] = 1; }
	}
}

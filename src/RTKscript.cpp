#include"mathf.h"
#include"RTK.h"
using namespace std;
/*********************
Calculate the zero baseline function:
Input: the real coordinates of the base station, the fixed solution coordinates of the rover
********************/
void CalZeroLine(double B[], double R[], double l[], double enu[])
{
	for (int i = 0; i < 3; i++)
	{
		l[i] = R[i] - B[i];
	}
	ENU a(R, B);
	CompEnudPos(&a);
	memcpy(enu, a.dEnu, 3 * sizeof(double));
}
void RtkAlignDataIni(RtkAlignData* a)
{
	a->Prn.clear();
	a->PvtInd.clear();
	a->Sys.clear();
	a->TarDis.clear();
	for (int i = 0; i < 2; i++)
	{
		a->RefDis[i] = 0.0;
		a->RefInd[i] = 0;
	}
}
/********************
Ambiguity initialization function:
Input parameters: Observation data, result data
********************/
void AmbigInitial(EPOCHOBSDATA* Obs, POSRES* Res)
{
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	int Prn = 0;
	int SysFlag = 0;
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Obs->Satobs[i].System == UNKS)/*Only BDS and GPS dual-band data are processed*/
		{
			continue;
		}
		if (!Obs->SatPvT[i].Valid)/*Consider PVT of sats*/
		{
			continue;
		}
		Prn = Obs->Satobs[i].Prn;
		SysFlag = (Obs->Satobs[i].System == GPS) ? 0 : 1;
		if (SysFlag == 1)
		{
			Prn = Prn + MAXGPSNUM;
		}
		Res->AmbiL1[Prn] = (Obs->Satobs[i].L[0] - Obs->Satobs[i].P[0]) / Lambda[2 * SysFlag + 0];
		Res->AmbiL2[Prn] = (Obs->Satobs[i].L[1] - Obs->Satobs[i].P[1]) / Lambda[2 * SysFlag + 1];
		Prn = 0;
	}
}
/*******************
Calculate the distance function (which does not include the distance of the reference satellite)
*****************/
void CalStaSatDis(double Pos_r[], EPOCHOBSDATA* Obs, DDCOBS* DDObs, RtkAlignData* DisData)
{
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Obs->Satobs[i].System != DDObs->Sys)
		{
			continue;
		}
		if (Obs->Satobs[i].Prn == DDObs->TarPrn)
		{
			double rho = 0;
			CalRho(Pos_r, Obs->SatPvT[i].SatPos, rho);
			DisData->TarDis.push_back(rho);
			DisData->PvtInd.push_back(i);
			DisData->Prn.push_back(Obs->Satobs[i].Prn);
			DisData->Sys.push_back(Obs->Satobs[i].System);
		}

	}
}
/*******************
Calculate the distance function from the reference satellite to the ground:
As long as the return is not zero, there is a reference satellite distance
*****************/
int CalRefDis(double Pos_r[], EPOCHOBSDATA* Obs, int RefPrn[], RtkAlignData* Data)
{
	int c = 0;
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Obs->Satobs[i].Prn == RefPrn[0] && Obs->Satobs[i].System == GPS)
		{
			double rho = 0;
			CalRho(Pos_r, Obs->SatPvT[i].SatPos, rho);
			Data->RefDis[0] = rho;
			Data->RefInd[0] = i;
			c++;
		}
		else if (Obs->Satobs[i].Prn == RefPrn[1] && Obs->Satobs[i].System == BDS)
		{
			double rho = 0;
			CalRho(Pos_r, Obs->SatPvT[i].SatPos, rho);
			Data->RefDis[1] = rho;
			Data->RefInd[1] = i;
			c++;
		}
	}
	return c;
}
/**************
Enter the specific elements of the B-matrix:
The output vec here is l, m, n
The input parameters are:
Rover coordinates, other satellite coordinates, reference star coordinates, rover station reference satellite ground distance, rover station other satellite satellite ground distance, output parameter :lmn
*************/
void RtkBElement(double Pos_r[], double TarSatPos[], double RefSatPos[], double TarDis, double RefDis, double vec[])
{
	for (int i = 0; i < 3; i++)
	{
		vec[i] = (Pos_r[i] - TarSatPos[i]) / TarDis - (Pos_r[i] - RefSatPos[i]) / RefDis;
	}
}
/*****************
Construct the B-matrix of Rtk least squares:
Satellite positions are based on the calculations of BasStation.
Parameters: Base Station Alignment Data, Rover Alignment Data, Rover Data, Base Station, Model: Floating-Point Solution Strategy(0), Fixed Solution Strategy(1)
****************/
void RtkInputB(RtkAlignData BasSatData, RtkAlignData RovSatData, double RovPos[], SATMIDRES BasSatPos[], XMatrix& B_r, int Model)
{
	XMatrix B;
	int PvtInd = 0;
	int RefInd = 0;
	int SysFlag = 0;
	for (int i = 0; i < BasSatData.TarDis.size(); i++)
	{
		double vec[3];
		PvtInd = BasSatData.PvtInd[i];
		SysFlag = (BasSatData.Sys[i] == GPS) ? 0 : 1;
		RefInd = BasSatData.RefInd[SysFlag];
		RtkBElement(RovPos, BasSatPos[PvtInd].SatPos, BasSatPos[RefInd].SatPos, RovSatData.TarDis[i], RovSatData.RefDis[SysFlag], vec);
		if (Model == 0)
		{
			for (int m = 0; m < 4; m++)
			{
				B(4 * i + m, 0) = vec[0];
				B(4 * i + m, 1) = vec[1];
				B(4 * i + m, 2) = vec[2];
			}
			int col = B.col - 1;/*-1 is required in the actual programming sequence*/
			if (BasSatData.Sys[i] == GPS)
			{
				B(4 * i + 2, col + 1) = CLight / GPS_L1;
				B(4 * i + 3, col + 2) = CLight / GPS_L2;
			}
			else if (BasSatData.Sys[i] == BDS)
			{
				B(4 * i + 2, col + 1) = CLight / BDS_B1I;
				B(4 * i + 3, col + 2) = CLight / BDS_B3I;
			}
		}
		else if (Model == 1)
		{
			for (int m = 0; m < 2; m++)
			{
				B(2 * i + m, 0) = vec[0];
				B(2 * i + m, 1) = vec[1];
				B(2 * i + m, 2) = vec[2];
			}
		}
	}
	B_r = B;
}
void RtkInputP(int GPSDDNum, int BDSDDNum, XMatrix& P_r, int Model)
{
	XMatrix P;
	/*Add the P array of GPS first*/
	/*Fill the diagonal array first*/
	bool flag = (Model == 0);
	int n = (flag) ? 4 : 2;
	for (int i = 0; i < n * GPSDDNum; i++)
	{
		if (i % 4 < 2 && flag)
		{
			P(i, i) = 1.0 * GPSDDNum / (GPSDDNum + 1);
		}
		else
		{
			P(i, i) = 1000.0 * GPSDDNum / (GPSDDNum + 1);
		}
	}
	/*Fill the triangle area*/
	for (int m = 1; m < GPSDDNum; m++)
	{
		for (int i = 0 + n * m; i < n * GPSDDNum; i++)
		{
			if (i % 4 < 2 && flag)
			{
				P(i, i - n * m) = -1.0 / (GPSDDNum + 1);
				P(i - n * m, i) = -1.0 / (GPSDDNum + 1);
			}
			else
			{
				P(i, i - n * m) = -1000.0 / (GPSDDNum + 1);
				P(i - n * m, i) = -1000.0 / (GPSDDNum + 1);
			}
		}
	}
	/*Plus the P array of BDS*/
	int Allnum = GPSDDNum + BDSDDNum;

	for (int i = n * GPSDDNum; i < n * Allnum; i++)
	{
		if (i % 4 < 2 && flag)
		{
			P(i, i) = 1.0 * BDSDDNum / (BDSDDNum + 1);
		}
		else
		{
			P(i, i) = 1000.0 * BDSDDNum / (BDSDDNum + 1);
		}
	}
	for (int m = 1; m < BDSDDNum; m++)
	{
		for (int i = n * GPSDDNum + n * m; i < n * Allnum; i++)
		{
			if (i % 4 < 2 && flag)
			{
				P(i, i - n * m) = -1.0 / (BDSDDNum + 1);
				P(i - n * m, i) = -1.0 / (BDSDDNum + 1);
			}
			else
			{
				P(i, i - n * m) = -1000.0 / (BDSDDNum + 1);
				P(i - n * m, i) = -1000.0 / (BDSDDNum + 1);
			}
		}
	}
	P_r = P;
}
/*************
Enter the W matrix element function:
Parameter:
Geometric distance from rover to reference star, geometric distance from rover to Tar star, 
geometric distance from base station to reference star, geometric distance from base station to
Tar star, and double difference geometric distance
*************/
void RtkWElement(double Rirho, double Rjrho, double Birho, double Bjrho, double& DDrho)
{
	DDrho = Rjrho - Rirho - Bjrho + Birho;
}
/*************
Enter the W matrix function:
Parameter:
Base Station Alignment Data, Rover Alignment Data, Double-Difference Data, W Matrix, 
Model: Floating-Point Solution Strategy(0), Fixed Solution Strategy(1)
*************/
void RtkInputW(RtkAlignData BasSatData, RtkAlignData RovSatData, DDCEPOCHOBS* DDObs, XMatrix& W_r, int model)
{
	XMatrix W;
	double DDrho = 0;
	int SysFlag = 0;
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	if (model == 0)
	{
		for (int i = 0; i < BasSatData.TarDis.size(); i++)
		{
			SysFlag = (BasSatData.Sys[i] == GPS) ? 0 : 1;
			RtkWElement(RovSatData.RefDis[SysFlag], RovSatData.TarDis[i], BasSatData.RefDis[SysFlag], BasSatData.TarDis[i], DDrho);

			
			W(4 * i + 0, 0) = DDObs->DDValue[i].ddP[0] - DDrho;
			W(4 * i + 1, 0) = DDObs->DDValue[i].ddP[1] - DDrho;
			W(4 * i + 2, 0) = DDObs->DDValue[i].ddL[0] - DDrho - DDObs->DDValue[i].ddN[0] * Lambda[2 * SysFlag + 0];
			W(4 * i + 3, 0) = DDObs->DDValue[i].ddL[1] - DDrho - DDObs->DDValue[i].ddN[1] * Lambda[2 * SysFlag + 1];
			//if (W(4 * i + 0, 0) > 1e3)
			//{
			//	int a = 0;
			//}
		}
	}
	else if (model == 1)
	{
		for (int i = 0; i < BasSatData.TarDis.size(); i++)
		{
			SysFlag = (BasSatData.Sys[i] == GPS) ? 0 : 1;
			RtkWElement(RovSatData.RefDis[SysFlag], RovSatData.TarDis[i], BasSatData.RefDis[SysFlag], BasSatData.TarDis[i], DDrho);
			W(2 * i + 0, 0) = DDObs->DDValue[i].ddL[0] - DDrho - DDObs->FixedAmb[2 * i + 0] * Lambda[2 * SysFlag + 0];
			W(2 * i + 1, 0) = DDObs->DDValue[i].ddL[1] - DDrho - DDObs->FixedAmb[2 * i + 1] * Lambda[2 * SysFlag + 1];
		}
	}

	W_r = W;
}
/***************
Organize all the double difference ambiguity into an array
***************/
void GetDDNSet(double DDNSet[], DDCEPOCHOBS* DDObs)
{
	for (int i = 0; i < DDObs->Sats; i++)
	{
		DDNSet[2 * i + 0] = DDObs->DDValue[i].ddN[0];
		DDNSet[2 * i + 1] = DDObs->DDValue[i].ddN[1];
	}
}

void CalRatio(double Fixedrms[],double &ratio)
{
	ratio = Fixedrms[1] / Fixedrms[0];
	if(ratio>1e4)
	{
		ratio=9999;
	}
}
/***************
Calculating Satellite Scores:
p: Satellite result data
d: Satellite observation data
s: Score
f: Weekly jump
****************/
void CalSatScore(SATMIDRES  p,SATOBSDATA d, double& s,int *f,DDCEPOCHOBS o)
{
	s = p.Elevation + (d.SNR[0] + d.SNR[1]) / 2.0;
	if (f[0] != 1 || f[1] != 1)
	{
		s = s - 60;
	}
	bool flag = SearchPrn(o, d.Prn, d.System);
	if (!flag)
	{
		s = s - 60;
	}
}
/*****************
Find an index number equal to d in s, otherwise return -1
n: quantity,
b: Start number
*******************/
int searchSamPrn(int d, int s[],int b, int n)
{
	for (int i = b; i < b+n; i++)
	{
		if (d == s[i])
		{
			return i;
		}
	}
	return -1;
}

/*****************
Build a state transition matrix
****************/
void consEKFPhi(RTKEKF* e, DDCEPOCHOBS* d,XMatrix &Phi)
{
	/*The XYZ partial state is shifted to 1 and remains unchanged*/
	for (int i = 0; i < 3; i++)
	{
		Phi(i, i) = 1;
	}
	/*GPS part*/
	inputPhi(e, d, Phi, GPS);
	/*BDS part*/
	inputPhi(e, d, Phi, BDS);
}

void inputPhi(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& Phi, GNSSSys s)
{
	int sysFlag = 0;
	int rb = 0, cb = 0;/*begining of row/col*/
	if (s == BDS)
	{
		sysFlag = 1;
		rb = d->DDSatNum[0];
		cb = e->GBNum[0];
	}

	bool sign = (e->refPrn[sysFlag] == d->RefPrn[sysFlag]);
	int v = 0;
	/*The ambiguity part first depends on whether the reference star changes*/
	if (sign)/*When the reference star does not change*/
	{
		for (int i = rb; i < rb + d->DDSatNum[sysFlag]; i++)
		{
			v = searchSamPrn(d->DDValue[i].TarPrn, e->nPrn, cb, e->GBNum[sysFlag]);
			if (v != -1)
			{
				Phi(3 + 2 * i + 0, 2 * v + 3  + 0) = 1;
				Phi(3 + 2 * i + 1, 2 * v + 3  + 1) = 1;
			}
			else { d->EkfChange[3 + 2 * i + 0] = d->EkfChange[3 + 2 * i + 1] = 1; continue; }
		}
	}
	else
	{
		int refInd = searchSamPrn(d->RefPrn[sysFlag], e->nPrn, cb, e->GBNum[sysFlag]);
		if (refInd != -1)
		{
			for (int i = rb; i < rb + d->DDSatNum[sysFlag]; i++)
			{
				v = searchSamPrn(d->DDValue[i].TarPrn, e->nPrn, cb, e->GBNum[sysFlag]);
				if (v != -1)
				{
					Phi(3 + 2 * i + 0, 3 + 2 * refInd + 0) = -1;
					Phi(3 + 2 * i + 1, 3 + 2 * refInd + 1) = -1;
					Phi(3 + 2 * i + 0, 3 + 2 * v  + 0) = 1;
					Phi(3 + 2 * i + 1, 3 + 2 * v  + 1) = 1;
				}
				else { d->EkfChange[3 + 2 * i + 0] = d->EkfChange[3 + 2 * i + 1] = 1;   continue; }
				if (d->DDValue[i].TarPrn == e->refPrn[sysFlag])/*If the previous reference star does not disappear, it is equivalent to a negative in terms of double difference ambiguity*/
				{
					Phi(3 + 2 * i + 0, 3 + 2 * refInd + 0) = -1;
					Phi(3 + 2 * i + 1, 3 + 2 * refInd + 1) = -1;
					d->EkfChange[3 + 2 * i + 0] = d->EkfChange[3 + 2 * i + 1] = 0;
				}
			}
		}
		else{
				for (int i = rb; i < rb + d->DDSatNum[sysFlag]; i++)
				{
					d->EkfChange[3 + 2 * i + 0] = d->EkfChange[3 + 2 * i + 1] = 1;
				}
			}
	}
}
void updateE(RTKEKF* e, DDCEPOCHOBS* d)
{
	memcpy(e->refPrn, d->RefPrn, 2 * sizeof(int));
	memcpy(e->GBNum, d->DDSatNum, 2 * sizeof(int));
	memset(e->nPrn, -1, MAXCHANNUM * sizeof(int));
	for (int i = 0; i < d->Sats; i++)
	{
		e->nPrn[i] = d->DDValue[i].TarPrn;
	}
	e->nSats = d->Sats;
}
/*****************
Build a Q Matrix:
The xyz part has an accuracy of 1e-2
The ambiguity part accuracy is set to 1e-5
******************/
void consEKFQ(DDCEPOCHOBS* d, XMatrix& Q)
{
	extern ROVERCFGINFO CFGINFO;
	for (int i = 0; i < 3; i++)
	{
		Q(i, i) = CFGINFO.PosQErr;
	}
	for (int i = 0; i < d->Sats; i++)
	{
		Q(2 * i + 0 + 3, 2 * i + 0 + 3) = CFGINFO.AmbQErr;
		Q(2 * i + 1 + 3, 2 * i + 1 + 3) = CFGINFO.AmbQErr;
	}
}

void consEKFP(RTKEKF* e, DDCEPOCHOBS* d, XMatrix &P,XMatrix &Q,XMatrix &Phi)
{
	/*Update the P-matrix with the old P-matrix first*/
	XMatrix t;
	t = Phi * P;
	Phi.MatrixTrans();
	P = t * Phi + Q;//At this point, P has become the number of rows and columns of the latest observations
	Phi.MatrixTrans();
	/*Weekly slips, new ascent satellites, and descent satellites were detected, and the P-matrix was partially reinitialized*/
	for (int i = 0; i < d->Sats; i++)
	{
		int sign = 0;
		/*Cycle slip or newly ascended satellites must be reinitialized*/
		if (d->DDValue[i].flag[0] == -1 || d->EkfChange[3 + 2 * i + 0] == 1)
		{
			EKFParryReIni(3+ 2 * i + 0, 3 +2* d->Sats, P, d->DDValue[i].Sys);
		}
		if (d->DDValue[i].flag[1] == -1 || d->EkfChange[3 + 2 * i + 1] == 1)
		{
			EKFParryReIni(3 + 2 * i + 1, 3 +2* d->Sats, P, d->DDValue[i].Sys);
		}
	}
}

/******************
Check the cycle slip of the double-difference observations
*****************/
void chkDDSlip(SDSATOBS* rs, SDSATOBS* ts, DDCOBS* d)
{
	if (rs->fFlag[0] == -1 || ts->fFlag[0] == -1)
	{
		d->flag[0] = -1;
	}
	if (rs->fFlag[1] == -1 || ts->fFlag[1] == -1)
	{
		d->flag[1] = -1;
	}
}

/****************
The row vector of P is reinitialized:
i: The line number that needs to be reinitialized
c: The total number of columns in the P array
*************/
void EKFParryReIni(int i,int c, XMatrix& P,GNSSSys sys)
{
	extern ROVERCFGINFO CFGINFO;
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	int sysFlag = (sys == GPS) ? 0 : 1;
	for (int j = 0; j < c; j++)
	{
		P(i, j) = 0;
	}
	if(i%2==1)
	{
		P(i, i) = CFGINFO.AmbPIni / Lambda[sysFlag * 2 + 0];
	}
	else
	{
		P(i, i) = CFGINFO.AmbPIni / Lambda[sysFlag * 2 + 1];
	}
}
/****************
Enter the L vector in the RTK Kalman filter
****************/
void RTKInputL(XMatrix B, XMatrix& L, POSRES* r, RtkAlignData BasSatData, RtkAlignData RovSatData, DDCEPOCHOBS* DDObs)
{
	double DDrho = 0;
	int SysFlag = 0;
	for (int i = 0; i < BasSatData.TarDis.size(); i++)
	{
		SysFlag = (BasSatData.Sys[i] == GPS) ? 0 : 1;
		RtkWElement(RovSatData.RefDis[SysFlag], RovSatData.TarDis[i], BasSatData.RefDis[SysFlag], BasSatData.TarDis[i], DDrho);
		L(4 * i + 0, 0) = DDObs->DDValue[i].ddP[0] - DDrho + B(4 * i + 0, 0) * r->Pos[0] + B(4 * i + 0, 1) * r->Pos[1] + B(4 * i + 0, 2) * r->Pos[2];
		L(4 * i + 1, 0) = DDObs->DDValue[i].ddP[1] - DDrho + B(4 * i + 1, 0) * r->Pos[0] + B(4 * i + 1, 1) * r->Pos[1] + B(4 * i + 1, 2) * r->Pos[2];
		L(4 * i + 2, 0) = DDObs->DDValue[i].ddL[0] - DDrho + B(4 * i + 2, 0) * r->Pos[0] + B(4 * i + 2, 1) * r->Pos[1] + B(4 * i + 2, 2) * r->Pos[2];
		L(4 * i + 3, 0) = DDObs->DDValue[i].ddL[1] - DDrho + B(4 * i + 3, 0) * r->Pos[0] + B(4 * i + 3, 1) * r->Pos[1] + B(4 * i + 3, 2) * r->Pos[2];
	}
}
/***************
R mat
***************/
void consEKFR(XMatrix& R, DDCEPOCHOBS* d)
{
	extern ROVERCFGINFO CFGINFO;
	for (int i = 0; i < d->DDValue.size(); i++)
	{
		R(4 * i + 0, 4 * i + 0) = CFGINFO.CodeNoise;
		R(4 * i + 1, 4 * i + 1) = CFGINFO.CodeNoise;
		R(4 * i + 2, 4 * i + 2) = CFGINFO.CPNoise;
		R(4 * i + 3, 4 * i + 3) = CFGINFO.CPNoise;
	}
}
/**************
K mat
*************/
void calEKFK(XMatrix& K, XMatrix P, XMatrix H, XMatrix R)
{
	XMatrix temp1,temp2;
	H.MatrixTrans();
	temp1 = P * H;
	H.MatrixTrans();
	temp2 = H * P;
	H.MatrixTrans();
	temp2 = temp2 * H + R;
	temp2.MatrixInv();
	K = temp1 * temp2;
}
/*****************
After the status is updated, if there is a new satellite or a weekly slip, it needs to be re-initialized
x: original state
x_1: State prediction
Phi: State Transition Matrix
d: Double-difference observation data
******************/
void UpdateX(XMatrix x, XMatrix &x_1, XMatrix Phi, DDCEPOCHOBS* d)
{
	x_1 = Phi * x;
	for (int i = 0; i < d->Sats; i++)
	{
		if (d->DDValue[i].flag[0] == -1 || d->EkfChange[3 + 2 * i + 0] == 1)
		{
			x_1(3 + 2 * i + 0, 0) = d->DDValue[i].ddN[0];

		}
		if(d->DDValue[i].flag[1] == -1|| d->EkfChange[3 + 2 * i + 1] == 1)
		{
			x_1(3 + 2 * i + 1, 0) = d->DDValue[i].ddN[1];
		}
	}

}
/*************
Initialization of the zeroing of the double-difference observations,
Note that in this function, the satellite number that records the occurrence of the last double-difference observation is implemented in passing
************/
void DDReini(DDCEPOCHOBS* d)
{
	memset(d->FormPrn, 0, MAXCHANNUM * sizeof(int));
	memset(d->FormSys, UNKS, MAXCHANNUM * sizeof(GNSSSys));
	RecordPrn(d, d->FormPrn, d->FormSys);
	d->DDValue.clear();
	d->Sats = 0;
	d->dPos[0] = d->dPos[1] = d->dPos[2] = 0.0;
	d->ResAmb[0] = d->ResAmb[1] = d->FixRMS[0] = d->FixRMS[1] = d->Ratio = 0.0;
	d->bFixed = false;
	for (int i = 0; i < 2; i++) 
	{
		d->DDSatNum[i] = 0;    // The number of double differences for each satellite system
		d->RefPos[i] = d->RefPrn[i] = -1;
	}
	for (int i = 0; i < MAXCHANNUM * 2; i++)
	{
		d->FixedAmb[2 * i + 0] =d->FixedAmb[2 * i + 1] = 0.0;
	}
	for (int i = 0; i < 3 + MAXCHANNUM * 2; i++)
	{
		d->EkfChange[i] = 0;
	}

}
/******************
The Qnn of the ambiguity portion is obtained by the P-matrix in EKF
*******************/
void EkfPGetQnn(XMatrix P, double Qnn[])
{
	XMatrix Qnn_m(P.row - 3, P.col - 3);
	for (int i = 0; i < P.row-3; i++)
	{
		for (int j = 0; j < P.col - 3; j++)
		{
			Qnn_m(i, j) = P(i + 3, j + 3);
		}
	}
	Matrix2Array(Qnn_m, Qnn);
}
/************
Get a set of floating-point solutions
*************/
void EkfXGetfN(XMatrix x, double fN[])
{
	for (int i = 3; i < x.row; i++)
	{
		fN[i - 3] = x(i, 0);
	}
}
/*************
A second observation update with a fixed solution
**************/
void TwiceUpdate(XMatrix& x_1, XMatrix& P, double fixedN[])
{
	extern ROVERCFGINFO CFGINFO;
	XMatrix L,H,R;
	XMatrix K, v, E, t;
	for (int i = 0; i < x_1.row ; i++)
	{
		R(i, i) = CFGINFO.AmbNoise;
		if (i < 3)
		{
			H(i, i) = 0;
			L(i, 0) = 0;
		}
		else { H(i, i) = 1; L(i, 0) = fixedN[i-3]; }
	}
	calEKFK(K, P, H, R);
	v = H * x_1;
	v = L - v;
	v = K * v;
	x_1 = x_1 + v;/*update finished*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
}
/**************
Record the PRN that appeared in the previous epoch
****************/
void RecordPrn(DDCEPOCHOBS* o, int r[], GNSSSys s[])
{
	int n = o->Sats;
	for (int i = 0; i < n; i++)
	{
		r[i] = o->DDValue[i].TarPrn;
		s[i] = o->DDValue[i].Sys;
	}
	r[n] = o->RefPrn[0]; r[n + 1] = o->RefPrn[1];/*Include reference stars*/
	s[n] = GPS; s[n + 1] =BDS;
}

/***************
Find the same Prn
***************/
bool SearchPrn(DDCEPOCHOBS d,int Prn,GNSSSys Sys)
{
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Prn==d.FormPrn[i] && Sys==d.FormSys[i])
		{
			return true;
		}
	}
	return false;
}

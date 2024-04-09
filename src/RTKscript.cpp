#include"mathf.h"
#include"RTK.h"
using namespace std;
/*********************
��������ߺ���:
���룺��վ��ʵ���꣬����վ�̶�������
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
ģ���ȳ�ʼ��������
����������۲����ݣ��������
********************/
void AmbigInitial(EPOCHOBSDATA* Obs, POSRES* Res)
{
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	int Prn = 0;
	int SysFlag = 0;
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Obs->Satobs[i].System == UNKS)/*ֻ����BDS��GPS˫Ƶ����*/
		{
			continue;
		}
		if (!Obs->SatPvT[i].Valid)/*����Ҳ���뿼�Ǵ�������Ҫ��PVT,�˴���BasEpk->SatPvT[SdObs->SdSatObs[i].nBas*/
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
�������ؾ��뺯�������в������ο������ؾ��룩
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
����ο����ǵ�������뺯��:
ֻҪ���ز������˵�����ڲο������ؾ���
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
��B�������Ԫ�ؽ�������:
���������vec��Ϊl,m,n
�����������Ϊ��
����վ���꣬�����������꣬�ο������꣬����վ�ο������ؾ࣬����վ���������ؾ࣬�������lmn
*************/
void RtkBElement(double Pos_r[], double TarSatPos[], double RefSatPos[], double TarDis, double RefDis, double vec[])
{
	for (int i = 0; i < 3; i++)
	{
		vec[i] = (Pos_r[i] - TarSatPos[i]) / TarDis - (Pos_r[i] - RefSatPos[i]) / RefDis;
	}
}
/*****************
����Rtk��С���˵�B����:
����λ����BasStation�ļ�����Ϊ׼��
��������վ�������ݣ�����վ�������ݣ�����վ���ݣ���վ,model:�������ԣ�0�����̶�����ԣ�1��
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
			int col = B.col - 1;/*ʵ�ʱ����������Ҫ-1*/
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
	/*�ȼ�GPS��P��*/
	/*�����Խ���*/
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
	/*�����������*/
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
	/*�ټ�BDS��P��*/
	int Allnum = GPSDDNum + BDSDDNum;
	/*�����Խ���*/
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
	/*�����������*/
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
����W����Ԫ�غ�����
������
����վ���ο��Ǽ��ξ��룬����վ��Tar�Ǽ��ξ��룬��վ���ο��Ǽ��ξ��룬��վ��Tar�Ǽ��ξ��룬˫��ξ���
*************/
void RtkWElement(double Rirho, double Rjrho, double Birho, double Bjrho, double& DDrho)
{
	DDrho = Rjrho - Rirho - Bjrho + Birho;
}
/*************
����W��������
������
��վ�������ݣ�����վ�������ݣ�˫�����ݣ�W����,model:�������ԣ�0�����̶�����ԣ�1��
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
������˫��ģ���������һ������
***************/
void GetDDNSet(double DDNSet[], DDCEPOCHOBS* DDObs)
{
	for (int i = 0; i < DDObs->Sats; i++)
	{
		DDNSet[2 * i + 0] = DDObs->DDValue[i].ddN[0];
		DDNSet[2 * i + 1] = DDObs->DDValue[i].ddN[1];
	}
}
/**************
����ratioֵ
****************/
void CalRatio(double Fixedrms[],double &ratio)
{
	ratio = Fixedrms[1] / Fixedrms[0];
}
/***************
�������ǵ÷�:
p:���ǽ������
d:���ǹ۲�����
s:�÷�
f:�������
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
��s���ҵ���d��ȵ�������,���򷵻�-1
n:������
b:����
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
����״̬ת�ƾ���
****************/
void consEKFPhi(RTKEKF* e, DDCEPOCHOBS* d,XMatrix &Phi)
{
	/*XYZ����״̬ת��Ϊ1������*/
	for (int i = 0; i < 3; i++)
	{
		Phi(i, i) = 1;
	}
	/*�ȹ���GPS����*/
	inputPhi(e, d, Phi, GPS);
	/*�ٹ���BDS����*/
	inputPhi(e, d, Phi, BDS);
}

void inputPhi(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& Phi, GNSSSys s)
{
	int sysFlag = 0;
	int rb = 0, cb = 0;/*������ʼ������*/
	if (s == BDS)
	{
		sysFlag = 1;
		rb = d->DDSatNum[0];
		cb = e->GBNum[0];
	}

	bool sign = (e->refPrn[sysFlag] == d->RefPrn[sysFlag]);
	int v = 0;
	/*ģ���Ȳ����ȿ��ο����Ƿ�ı�*/
	if (sign)/*�ο��ǲ���ʱ*/
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
				if (d->DDValue[i].TarPrn == e->refPrn[sysFlag])/*��ǰһ�ŵĲο��ǲ�û����ʧ������˫��ģ�������൱��ȡ��*/
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
����Q����
xyz���־���Ϊ1e-2
ģ���Ȳ��־�����Ϊ1e-5
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
/*****************
����P����
*****************/
void consEKFP(RTKEKF* e, DDCEPOCHOBS* d, XMatrix &P,XMatrix &Q,XMatrix &Phi)
{
	/*���Ծ�P�������P����*/
	XMatrix t;
	t = Phi * P;
	Phi.MatrixTrans();
	P = t * Phi + Q;//��ʱP�Ѿ���������µĹ۲�����������
	Phi.MatrixTrans();/*ת�û���*/
	//P.MatrixDis();
	/*����������������Ǻ��½����ǣ���P������в����س�ʼ��*/
	for (int i = 0; i < d->Sats; i++)
	{
		int sign = 0;
		/*���������������ǣ���Ҫ���³�ʼ��*/
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
���˫��۲�ֵ���������
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
P�����������³�ʼ��:
i����Ҫ�س�ʼ�����к�
c��P��������
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
����RTK�������˲��е�L����
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
�۲���������
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
����K����
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
״̬������������������ǻ�����������Ҫ���³�ʼ��
x:ԭ״̬
x_1:״̬Ԥ��
Phi:״̬ת�ƾ���
d:˫��۲�����
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
��˫��۲�ֵ�����ʼ����
ע���ڸú�����˳��ʵ�ּ�¼��һ��˫��۲�ֵ���ֵ����Ǻ�
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
		d->DDSatNum[i] = 0;    // ������ϵͳ��˫������
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
��EKF��ͨ��P������ģ���Ȳ��ֵ�Qnn
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
��ø���⼯��
*************/
void EkfXGetfN(XMatrix x, double fN[])
{
	for (int i = 3; i < x.row; i++)
	{
		fN[i - 3] = x(i, 0);
	}
}
/*************
���ù̶���ڶ��ι۲�ֵ����
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
	x_1 = x_1 + v;/*���������*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
}
/**************
��¼ǰһ����Ԫ���ֵ�PRN
****************/
void RecordPrn(DDCEPOCHOBS* o, int r[], GNSSSys s[])
{
	int n = o->Sats;
	for (int i = 0; i < n; i++)
	{
		r[i] = o->DDValue[i].TarPrn;
		s[i] = o->DDValue[i].Sys;
	}
	r[n] = o->RefPrn[0]; r[n + 1] = o->RefPrn[1];/*���ο���Ҳ������ȥ*/
	s[n] = GPS; s[n + 1] =BDS;
}

/***************
������ͬPrn
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

#include"mathf.h"
#include"RTK.h"
using namespace std;
/*********************
计算零基线函数:
输入：基站真实坐标，流动站固定解坐标
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
模糊度初始化函数：
输入参数：观测数据，结果数据
********************/
void AmbigInitial(EPOCHOBSDATA* Obs, POSRES* Res)
{
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	int Prn = 0;
	int SysFlag = 0;
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (Obs->Satobs[i].System == UNKS)/*只处理BDS和GPS双频数据*/
		{
			continue;
		}
		if (!Obs->SatPvT[i].Valid)/*这里也必须考虑待测卫星要有PVT,此处用BasEpk->SatPvT[SdObs->SdSatObs[i].nBas*/
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
计算卫地距离函数（其中不包含参考星卫地距离）
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
计算参考卫星到地面距离函数:
只要返回不是零就说明存在参考星卫地距离
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
对B矩阵具体元素进行输入:
这里输出的vec即为l,m,n
输入参数依次为：
流动站坐标，其他卫星坐标，参考星坐标，流动站参考星卫地距，流动站其他星卫地距，输出参数lmn
*************/
void RtkBElement(double Pos_r[], double TarSatPos[], double RefSatPos[], double TarDis, double RefDis, double vec[])
{
	for (int i = 0; i < 3; i++)
	{
		vec[i] = (Pos_r[i] - TarSatPos[i]) / TarDis - (Pos_r[i] - RefSatPos[i]) / RefDis;
	}
}
/*****************
构建Rtk最小二乘的B矩阵:
卫星位置以BasStation的计算结果为准。
参数：基站对齐数据，流动站对齐数据，流动站数据，基站,model:浮点解策略（0），固定解策略（1）
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
			int col = B.col - 1;/*实际编程序列中需要-1*/
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
	/*先加GPS的P阵*/
	/*先填充对角阵*/
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
	/*填充三角区域*/
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
	/*再加BDS的P阵*/
	int Allnum = GPSDDNum + BDSDDNum;
	/*先填充对角阵*/
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
	/*填充三角区域*/
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
输入W矩阵元素函数：
参数：
流动站到参考星几何距离，流动站到Tar星几何距离，基站到参考星几何距离，基站到Tar星几何距离，双差几何距离
*************/
void RtkWElement(double Rirho, double Rjrho, double Birho, double Bjrho, double& DDrho)
{
	DDrho = Rjrho - Rirho - Bjrho + Birho;
}
/*************
输入W矩阵函数：
参数：
基站对齐数据，流动站对齐数据，双差数据，W矩阵,model:浮点解策略（0），固定解策略（1）
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
将所有双差模糊度整理成一个数组
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
计算ratio值
****************/
void CalRatio(double Fixedrms[],double &ratio)
{
	ratio = Fixedrms[1] / Fixedrms[0];
}
/***************
计算卫星得分:
p:卫星结果数据
d:卫星观测数据
s:得分
f:周跳情况
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
在s中找到与d相等的索引号,否则返回-1
n:数量，
b:起点号
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
构建状态转移矩阵
****************/
void consEKFPhi(RTKEKF* e, DDCEPOCHOBS* d,XMatrix &Phi)
{
	/*XYZ部分状态转移为1，不变*/
	for (int i = 0; i < 3; i++)
	{
		Phi(i, i) = 1;
	}
	/*先构建GPS部分*/
	inputPhi(e, d, Phi, GPS);
	/*再构建BDS部分*/
	inputPhi(e, d, Phi, BDS);
}

void inputPhi(RTKEKF* e, DDCEPOCHOBS* d, XMatrix& Phi, GNSSSys s)
{
	int sysFlag = 0;
	int rb = 0, cb = 0;/*行列起始索引号*/
	if (s == BDS)
	{
		sysFlag = 1;
		rb = d->DDSatNum[0];
		cb = e->GBNum[0];
	}

	bool sign = (e->refPrn[sysFlag] == d->RefPrn[sysFlag]);
	int v = 0;
	/*模糊度部分先看参考星是否改变*/
	if (sign)/*参考星不变时*/
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
				if (d->DDValue[i].TarPrn == e->refPrn[sysFlag])/*若前一颗的参考星并没有消失，则在双差模糊度上相当于取负*/
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
构建Q矩阵：
xyz部分精度为1e-2
模糊度部分精度设为1e-5
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
构建P矩阵
*****************/
void consEKFP(RTKEKF* e, DDCEPOCHOBS* d, XMatrix &P,XMatrix &Q,XMatrix &Phi)
{
	/*先以旧P矩阵更新P矩阵*/
	XMatrix t;
	t = Phi * P;
	Phi.MatrixTrans();
	P = t * Phi + Q;//此时P已经变成了最新的观测数的行列数
	Phi.MatrixTrans();/*转置回来*/
	//P.MatrixDis();
	/*检测周跳，新升卫星和下降卫星，对P矩阵进行部分重初始化*/
	for (int i = 0; i < d->Sats; i++)
	{
		int sign = 0;
		/*周跳或者新升卫星，都要重新初始化*/
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
检查双差观测值的周跳情况
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
P的行向量重新初始化:
i：需要重初始化的行号
c：P阵总列数
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
输入RTK卡尔曼滤波中的L向量
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
观测噪声矩阵
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
计算K矩阵
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
状态更新完后若有新升卫星或者周跳，需要重新初始化
x:原状态
x_1:状态预测
Phi:状态转移矩阵
d:双差观测数据
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
对双差观测值清零初始化，
注意在该函数中顺便实现记录上一次双差观测值出现的卫星号
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
		d->DDSatNum[i] = 0;    // 各卫星系统的双差数量
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
在EKF中通过P矩阵获得模糊度部分的Qnn
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
获得浮点解集合
*************/
void EkfXGetfN(XMatrix x, double fN[])
{
	for (int i = 3; i < x.row; i++)
	{
		fN[i - 3] = x(i, 0);
	}
}
/*************
利用固定解第二次观测值更新
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
	x_1 = x_1 + v;/*最后更新完成*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
}
/**************
记录前一个历元出现的PRN
****************/
void RecordPrn(DDCEPOCHOBS* o, int r[], GNSSSys s[])
{
	int n = o->Sats;
	for (int i = 0; i < n; i++)
	{
		r[i] = o->DDValue[i].TarPrn;
		s[i] = o->DDValue[i].Sys;
	}
	r[n] = o->RefPrn[0]; r[n + 1] = o->RefPrn[1];/*将参考星也包含进去*/
	s[n] = GPS; s[n + 1] =BDS;
}

/***************
查找相同Prn
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

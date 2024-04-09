#include"SPP.h"
#include"mathf.h"
#include"RTK.h"
#include"sockets.h"

using namespace std;
/************
流动站基站时间同步函数:
1代表同步成功；
0代表同步失败,即基站观测数据时间在流动站之后；
-1代表文件读取结束；
***************/
int RtkObsSyn(FILE* fb, FILE* fr,RTKDATA *rawdata,POSRES *BasPos,POSRES *RovPos)
{
	extern ROVERCFGINFO CFGINFO;
	static unsigned char Rbuff[MAXRawLen];
	static unsigned char Bbuff[MAXRawLen];
	static int Rd=0;
	static int Bd=0;
	int RLen=0, BLen=0;
	bool fileFlag = false;
	while (!feof(fr))
	{
		/*先读流动站数据*/
		RLen = fread(Rbuff + Rd, 1, MAXRawLen - Rd, fr);/*len返回的是读取个数*/
		if (RLen < MAXRawLen - Rd)
		{
			fileFlag = true;
		}/*读到了结尾*/
		Rd = Rd + RLen;//读了文件后字节长度恢复MaxRawLen
		int RangeFlag = DecodeNovOem7Dat(Rbuff, Rd, &rawdata->RovEpk, rawdata->GpsEph, rawdata->BdsEph, RovPos);
		if (RangeFlag == 1) break;/*读到了观测数据*/
		if (fileFlag)
		{
			cout << "文件读取结束" << endl;
			return -1;/*读到了结尾*/
		}
	}
	/*先验证本次更新流动站数据后的dt是否小于阈值*/
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynFileThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
	{
		return 0;
	}
	/*解码基站数据并做数据对齐，结果分三种情况*/
	while (1)
	{
		while (!feof(fb))
		{
			BLen = fread(Bbuff + Bd, 1, MAXRawLen - Bd, fb);/*len返回的是读取个数*/
			if (BLen < MAXRawLen - Bd)
			{
				fileFlag = true;
			}
			Bd = BLen + Bd;
			int RangeFlag = DecodeNovOem7Dat(Bbuff, Bd, &rawdata->BasEpk, rawdata->GpsEph, rawdata->BdsEph, BasPos);//1代表有星历有观测数据，可以解算，-1代表数据截断，继续读取数据以便解算
			if (RangeFlag == 1) { break; }/*读到了观测数据*/
			if (fileFlag)
			{
				cout << "文件读取结束" << endl;
				return -1;/*读到了结尾*/
			}
		}

		if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynFileThre)
		{
			return 1;
		}
		if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
		{
			return 0;
		}
	}
}

int RtkObsSyn(SOCKET& bIp, SOCKET& rIp, RTKDATA* rawdata, POSRES* BasPos, POSRES* RovPos)
{
	extern ROVERCFGINFO CFGINFO;
	static unsigned char Rbuff[MAXRawLen];
	static unsigned char Rtemp[MAXRawLen];
	static unsigned char Bbuff[MAXRawLen];
	static unsigned char Btemp[MAXRawLen];
	static int Rd = 0;
	static int Bd = 0;
	int RLen = 0, BLen = 0;
	while (1)
	{
		//Sleep(980);
		/*先读流动站数据*/
		if ((RLen = recv(rIp, (char*)Rtemp, MAXRawLen, 0)) > 0)
		{
			memcpy(Rbuff + Rd, Rtemp, RLen);
			Rd = Rd + RLen;
			memset(Rtemp, 0, MAXRawLen);
			int RangeFlag = DecodeNovOem7Dat(Rbuff, Rd, &rawdata->RovEpk, rawdata->GpsEph, rawdata->BdsEph, RovPos);
			if (RangeFlag == 1)
			{
				cout << "Get RovObs" << endl;
				break;
			}/*读到了观测数据*/
		}
		else { cout << "未接受到流动站数据" << endl; return -1; }

	}
	/*先验证本次更新流动站数据后的dt是否小于阈值*/
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynSktThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)/*流动站数据在基站数据之前*/
	{
		return 0;
	}
	/*解码基站数据并做数据对齐，结果分三种情况*/
	while (1)
	{
		while (1)
		{
			//Sleep(980);
			/*先读流动站数据*/
			if ((BLen = recv(bIp, (char*)Btemp, MAXRawLen, 0)) > 0)
			{
				memcpy(Bbuff + Bd, Btemp, BLen);
				Bd = Bd + BLen;
				memset(Btemp, 0, MAXRawLen);
				int RangeFlag = DecodeNovOem7Dat(Bbuff, Bd, &rawdata->BasEpk, rawdata->GpsEph, rawdata->BdsEph, BasPos);
				if (RangeFlag == 1)
				{
					cout << "Get BasObs" << endl;
					break;
				}/*读到了观测数据*/
			}
			else { cout << "未接受到流动站数据" << endl; return -1; }
		}

		if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynSktThre)
		{
			return 1;
		}
		if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
		{
			return 0;
		}
	}
}
/****************
计算单差函数:
1、输入数据:基站数据，流动站数据，基站结果数据，流动站结果数据，单差数据
2、注意RTKDATA中obs的fFlag代表一个卫星一个频率的状况（-1有周跳，0无数据，1无周跳）
				sdobs的fFlag代表单差数据的频率有效性（-1有周跳，0无数据，1无周跳）
本函数只判断数据是否是双频并进行单差，周跳信息在CalSinDifCySlip函数计算
***************/
void CalStaSinDif(EPOCHOBSDATA* BasEpk,EPOCHOBSDATA *RovEpk,POSRES *BasPos,POSRES *RovPos,SDEPOCHOBS *SdObs)
{
	GPSTIME time = BasEpk->Time;
	memcpy(&SdObs->Time, &time, sizeof(GPSTIME));
	int Prn;
	GNSSSys Sys;
	double dP[2], dL[2];
	double DN[2];
	AmbigInitial(BasEpk, BasPos);/*基站模糊度初始化*/
	AmbigInitial(RovEpk, RovPos);/*流动站模糊度初始化*/
	/*整个循环的流程均以基站的次序为主*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if(BasEpk->Satobs[i].Valid) /*判断基站观测数据是否有效*/
		{
			Prn = BasEpk->Satobs[i].Prn;
			Sys= BasEpk->Satobs[i].System;
		}
		else { continue; }

		if (Sys== UNKS)
		{
			SdObs->SdSatObs[i].fFlag[0]
				=SdObs->SdSatObs[i].fFlag[1] 
				= 0;
			continue;
		}
		SdObs->SdSatObs[i].nBas = i;
		/*寻找流动站中对应的有效PRN号*/
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			/*检查PRN和系统能否对应上*/
			bool flag = (Prn == RovEpk->Satobs[j].Prn && Sys == RovEpk->Satobs[j].System) ? true : false;
			if (RovEpk->Satobs[j].Valid && flag)/*判断流动站有效且通过对齐*/
			{
				SdObs->SdSatObs[i].nRov = j;
				SdObs->SatNum++;
				SdObs->SdSatObs[i].Prn = Prn;
			    SdObs->SdSatObs[i].System = Sys;
				if (Sys == BDS)
				{
					Prn = Prn + MAXGPSNUM;
				}
				DN[0] = RovPos->AmbiL1[Prn] - BasPos->AmbiL1[Prn];
				DN[1] = RovPos->AmbiL2[Prn] - BasPos->AmbiL2[Prn];
				/*进行单差处理*/
				for (int m = 0; m < 2; m++)
				{
					dP[m] = RovEpk->Satobs[j].P[m] - BasEpk->Satobs[i].P[m];

					dL[m] = RovEpk->Satobs[j].L[m] - BasEpk->Satobs[i].L[m];

					
				/*判断是单频还是双频*/
					if (RovEpk->Satobs[j].fFlag[m]==0 || BasEpk->Satobs[i].fFlag[m]==0)
					{
						SdObs->SdSatObs[i].P[m] = 0.0;
						SdObs->SdSatObs[i].L[m] = 0.0;
						SdObs->SdSatObs[i].fFlag[m] = 0;
					}
					else 
					{ 
						SdObs->SdSatObs[i].P[m] = dP[m];
						SdObs->SdSatObs[i].L[m] = dL[m];
						SdObs->SdSatObs[i].DN[m] = DN[m];
						SdObs->SdSatObs[i].fFlag[m] = 1;
					}
				}
				Prn=0; /*将Prn恢复默认值*/
			}
			else { continue; }
		}
	}
}
/***********************
利用单差数据进一步周跳检测：
采用turboEdit方法进行周跳检验
*************************/
void DTSinDifCySlip(EPOCHOBSDATA* BasEpk, EPOCHOBSDATA* RovEpk, SDEPOCHOBS* SdObs)
{
	int BasInd,RovInd;
	int j = 0;
	bool FindFlag;
	double dGF, dMW;
	MWGF Com_Cur[MAXCHANNUM];
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		/*先根据非差观测值判断是否有周跳*/
		BasInd = SdObs->SdSatObs[i].nBas;
		RovInd = SdObs->SdSatObs[i].nRov;
		for (int m = 0; m < 2; m++)
		{
			if (BasEpk->Satobs[BasInd].fFlag[m] == -1 || RovEpk->Satobs[RovInd].fFlag[m] == -1)
			{
				SdObs->SdSatObs[i].fFlag[m] = -1;
			}
		}
		Com_Cur[i].Prn = SdObs->SdSatObs[i].Prn;
		Com_Cur[i].Sys = SdObs->SdSatObs[i].System;
		Com_Cur[i].n = 1;
		/*进行单差观测值的MWGF组合，判断是否有周跳*/
		CalMWGFPIF<MWGF, SDSATOBS>(Com_Cur[i], SdObs->SdSatObs[i]);
		FindFlag = false;
		for ( j = 0; j < MAXCHANNUM; j++)
		{
			if (Com_Cur[i].Prn == SdObs->SdCObs[j].Prn && Com_Cur[i].Sys == SdObs->SdCObs[j].Sys &&
				fabs(SdObs->SdCObs[j].GF) > 1e-8)
			{
				FindFlag = true;
				break;
			}
		}
		if (FindFlag)
		{
			dGF = Com_Cur[i].GF - SdObs->SdCObs[j].GF;
			dMW = Com_Cur[i].MW - SdObs->SdCObs[j].MW;/*先计算差值dMW再计算MW的当前历元平滑值并储存*/
			if (fabs(dGF) < 5e-2 && fabs(dMW) < 3)
			{
				//Obs->SatPvT[i].Valid = true;
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = 1;
				SdObs->SdCObs[i].Prn = SdObs->SdCObs[j].Prn;
				SdObs->SdCObs[i].Sys = SdObs->SdCObs[j].Sys;
				SdObs->SdCObs[i].MW = (SdObs->SdCObs[j].n * SdObs->SdCObs[j].MW + Com_Cur[i].MW) / (SdObs->SdCObs[j].n + 1);
				SdObs->SdCObs[i].n = SdObs->SdCObs[j].n + 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;//为了与obs的PRN顺序对齐，这里按照obs的顺序排列
			}
			else/*即此时发生了周跳，MW组合值应该开始新的平滑*/
			{
				//Obs->SatPvT[i].Valid = true;
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = -1;
				SdObs->SdCObs[i].Prn = Com_Cur[i].Prn;
				SdObs->SdCObs[i].Sys = Com_Cur[i].Sys;
				SdObs->SdCObs[i].MW = Com_Cur[i].MW;
				SdObs->SdCObs[i].n = 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;  //PIF与周跳是否发生无关
			}
			if (i != j)
			{
				memset(SdObs->SdCObs+ j, 0, sizeof(MWGF));//在处理完第i个序列的观测值后，将原第j个MWGF给重置
			}
		}
		else                                                                                                                        
		{
			memcpy(SdObs->SdCObs + i, Com_Cur + i, sizeof(MWGF));
			//Obs->SatPvT[i].Valid = true;
			SdObs->SdCObs[i].n = 1;
			SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;
		}
		//memset(Obs->Satobs+i, 0, sizeof(SATOBSDATA));

	}
}
/***************
 双差函数:
 参数：流动站数据，单差数据，双差输出数据
 ***************/
void CalStaDouDif(EPOCHOBSDATA* RovEpk,SDEPOCHOBS* SdObs, DDCEPOCHOBS* DDObs)
{
	int RefInd=0;
	int GPSnum=0, BDSnum = 0;
	double ScoreMax[2] = { 0.0,0.0 };
	int SatFlag = 0;/*判断该星是GPS（0）还是BDS（1）*/
	/*选取参考星，从流动站选取*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (SdObs->SdSatObs[i].System == UNKS)/*只处理BDS和GPS卫星数据*/
		{
			continue;
		}

		RefInd= SdObs->SdSatObs[i].nRov;
		if (!RovEpk->SatPvT[RefInd].Valid)
		{
			continue;
		}

		SatFlag = (SdObs->SdSatObs[i].System == GPS) ? 0 : 1;

		if (SatFlag == 0)
		{
			GPSnum++;
		}
		else {
			BDSnum++;
		}
		double score = 0;
		CalSatScore(RovEpk->SatPvT[RefInd], RovEpk->Satobs[RefInd], score, SdObs->SdSatObs[i].fFlag,*DDObs);
		if (score > ScoreMax[SatFlag])
		{
			ScoreMax[SatFlag] = score;
			DDObs->RefPrn[SatFlag] = SdObs->SdSatObs[i].Prn;/*BDS和GPS分别有一个参考星*/
			DDObs->RefPos[SatFlag] = i;/*此时的索引变成单差数组中的索引*/
		}
	}
	DDObs->DDSatNum[0] = (GPSnum > 1)? GPSnum-1:0;
	DDObs->DDSatNum[1] = (BDSnum > 1)? BDSnum-1:0;
	DDObs->Sats = DDObs->DDSatNum[0] + DDObs->DDSatNum[1];
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		DDCOBS tempObs;
		double tempDDN = 0.0;
		if (SdObs->SdSatObs[i].System == UNKS)/*只处理BDS和GPS双频数据*/
		{
			continue;
		}
		if (!RovEpk->SatPvT[SdObs->SdSatObs[i].nRov].Valid)/*这里也必须考虑待测卫星要有PVT,此处用BasEpk->SatPvT[SdObs->SdSatObs[i].nBas*/
		{
			continue;
		}
		SatFlag = (SdObs->SdSatObs[i].System == GPS) ? 0 : 1;
		RefInd = DDObs->RefPos[SatFlag];/*将refInd置为真正的参考卫星索引*/
		if(i== RefInd)/*若卫星重合则跳过*/
		{
			continue;
		}
		tempObs.Sys = SdObs->SdSatObs[i].System;
		tempObs.TarPrn = SdObs->SdSatObs[i].Prn;
		for(int m=0;m<2;m++)
		{
			tempObs.ddP[m] = SdObs->SdSatObs[i].P[m] - SdObs->SdSatObs[RefInd].P[m];
			tempObs.ddL[m] = SdObs->SdSatObs[i].L[m] - SdObs->SdSatObs[RefInd].L[m];
			tempObs.ddN[m] = SdObs->SdSatObs[i].DN[m] - SdObs->SdSatObs[RefInd].DN[m];
			tempObs.flag[0] = tempObs.flag[1] = 1;
		}
		chkDDSlip(&SdObs->SdSatObs[RefInd], &SdObs->SdSatObs[i],&tempObs);
		DDObs->DDValue.push_back(tempObs);
	}
}
bool RTKFixed(EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs, POSRES* RovPos, DDCEPOCHOBS* DDObs, RtkAlignData &RAlign, RtkAlignData BAlign)
{
	LSQ ls;
	int IterNum = 0;
	do/*最小二乘迭代*/
	{
		RtkAlignDataIni(&RAlign);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/* 此时RovSatDis的卫星排号与BasSatDis一致*/
		{
			CalStaSatDis(RovPos->Pos, RovObs, &DDObs->DDValue[i], &RAlign);
		}
		CalRefDis(RovPos->Pos, RovObs, DDObs->RefPrn, &RAlign);
		RtkInputB(BAlign, RAlign, RovPos->Pos, BasObs->SatPvT, ls.B, 1);
		//ls.B.MatrixDis();
		RtkInputP(DDObs->DDSatNum[0], DDObs->DDSatNum[1], ls.P,1);
		//ls.P.MatrixDis();
		RtkInputW(BAlign, RAlign, DDObs, ls.W, 1);
		//ls.W.MatrixDis();
		LSQCalx(ls.B, ls.P, ls.W, ls.x, ls.Q);
		RovPos->Pos[0] = RovPos->Pos[0] + ls.x(0, 0);
		RovPos->Pos[1] = RovPos->Pos[1] + ls.x(1, 0);
		RovPos->Pos[2] = RovPos->Pos[2] + ls.x(2, 0);
		IterNum++;
		if (IterNum > 10)
		{
			cout << "最小二乘未收敛" << endl;
			return false;
		}
	} while (fabs(ls.x(0, 0) + ls.x(1, 0) + ls.x(2, 0)) > 1e-6);


}
/*********************
相对定位最小二乘函数:
输入：流动站观测数据，基站观测数据，流动站位置，基站位置，
双差数据，单差数据
********************/
bool RTK(EPOCHOBSDATA *RovObs,EPOCHOBSDATA *BasObs, POSRES* RovPos, POSRES* BasPos,DDCEPOCHOBS *DDObs)
{
	/*流动站和基站位置初值就是函数入口的两个位置参数*/
	double DDNSet[2 * MAXCHANNUM];
	double* Qnn=new double[4 * DDObs->Sats * DDObs->Sats];
	memset(Qnn, 0, 4* DDObs->Sats * DDObs->Sats * sizeof(double));
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	LSQ ls;
	int IterNum = 0;
	RecordPrn(DDObs, DDObs->FormPrn, DDObs->FormSys);
	/*基站到待测星几何距离*/
	for (int i = 0; i < DDObs->Sats; i++)/*计算得到的卫星排序和*/
	{
		CalStaSatDis(BasPos->RealPos, BasObs, &DDObs->DDValue[i], &BasSatData);	
	}
	CalRefDis(BasPos->RealPos, BasObs, DDObs->RefPrn, &BasSatData);

	do/*最小二乘迭代*/
	{
		RtkAlignDataIni(&RovSatData);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/* 此时RovSatDis的卫星排号与BasSatDis一致*/
		{
			CalStaSatDis(RovPos->Pos, RovObs, &DDObs->DDValue[i], &RovSatData);
		}
		CalRefDis(RovPos->Pos, RovObs, DDObs->RefPrn, &RovSatData);
		RtkInputB(BasSatData, RovSatData, RovPos->Pos, BasObs->SatPvT, ls.B, 0);
		//ls.B.MatrixDis();
		RtkInputP(DDObs->DDSatNum[0], DDObs->DDSatNum[1], ls.P,0);
		//ls.P.MatrixDis();
		RtkInputW(BasSatData, RovSatData, DDObs, ls.W, 0);
		//ls.W.MatrixDis();
		LSQCalx(ls.B, ls.P, ls.W, ls.x, ls.Q);
		RovPos->Pos[0] = RovPos->Pos[0] + ls.x(0, 0);
		RovPos->Pos[1] = RovPos->Pos[1] + ls.x(1, 0);
		RovPos->Pos[2] = RovPos->Pos[2] + ls.x(2, 0);
		for (int m = 0; m < DDObs->DDValue.size(); m++)
		{
			cout.precision(5);
			//cout << DDObs->DDValue[m].ddN[0] << endl;
			//cout << DDObs->DDValue[m].ddN[1] << endl;

			DDObs->DDValue[m].ddN[0] = DDObs->DDValue[m].ddN[0] + ls.x(3 + m * 2 + 0, 0);
			DDObs->DDValue[m].ddN[1] = DDObs->DDValue[m].ddN[1] + ls.x(3 + m * 2 + 1, 0);
		}
		IterNum++;
		if (IterNum > 10)
		{
			cout << "最小二乘未收敛" << endl;
			return false;
		}
	}while (fabs(ls.x(0, 0) + ls.x(1, 0) + ls.x(2, 0)) > 1e-6);
	LSQCalPrCis(ls, ls.theta, ls.PDOP, 1);
	GetDDNSet(DDNSet, DDObs);
	
	Matrix2Array(ls.Qnn, Qnn);
	lambda(2 * DDObs->Sats, 2, DDNSet, Qnn, DDObs->FixedAmb, DDObs->FixRMS);
	RTKFixed(RovObs, BasObs, RovPos, DDObs, RovSatData, BasSatData);
	CalZeroLine(BasPos->RealPos, RovPos->Pos, DDObs->dPos,DDObs->denu);
	CalRatio(DDObs->FixRMS, DDObs->Ratio);
	//ls.Q.MatrixDis();
	//cout << "迭代次数：" << IterNum << endl;
	cout.flags(ios::fixed);
	cout.precision(8);
	//cout <<"X:  "<< DDObs->dPos[0]<<" Y:  " << DDObs->dPos[1] << " Z:  "<<DDObs->dPos[2] << endl;
	delete[] Qnn;
	return true;
}
void EKFinitial(RTKEKF *e, DDCEPOCHOBS* DDObs, POSRES *Sppr,XMatrix &ekfP)
{
	extern ROVERCFGINFO CFGINFO;
	double Lambda[4] = { CLight / GPS_L1,CLight / GPS_L2,CLight / BDS_B1I,CLight / BDS_B3I };
	memcpy(e->refPrn, DDObs->RefPrn, 2 * sizeof(int));
	memcpy(e->GBNum, DDObs->DDSatNum, 2 * sizeof(int));
	for (int i = 0; i < 3; i++)
	{
		e->X0[i] = Sppr->Pos[i];
		ekfP(i, i) = CFGINFO.PosPIni;
	}
	for (int i = 0; i < DDObs->Sats; i++)
	{
		e->X0[3 + 2 * i + 0] = DDObs->DDValue[i].ddN[0];
		e->X0[3 + 2 * i + 1] = DDObs->DDValue[i].ddN[1];
		e->nPrn[i] = DDObs->DDValue[i].TarPrn;
		if (i < e->GBNum[0])
		{
			ekfP(3 + 2 * i+0, 3 + 2 * i+0) = CFGINFO.AmbPIni/ Lambda[0];
			ekfP(3 + 2 * i+1, 3 + 2 * i+1) = CFGINFO.AmbPIni / Lambda[1];
		}
		else
		{
			ekfP(3 + 2 * i + 0, 3 + 2 * i + 0) = CFGINFO.AmbPIni / Lambda[2];
			ekfP(3 + 2 * i + 1, 3 + 2 * i + 1) = CFGINFO.AmbPIni / Lambda[3];
		}
	}
	e->IsInit = true;
	e->nSats = DDObs->Sats;
	memcpy(e->X, e->X0, 197 * sizeof(double));
}
/************
牢记进行一次EKF后要更新e
*************/
void EKF(RTKEKF *e,DDCEPOCHOBS *d,POSRES *r,POSRES *b,XMatrix &P, EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs)
{
	/*先构建状态转移矩阵Phi*/
	XMatrix Phi(3 + 2 * d->Sats, 3 + 2 * e->nSats);
	XMatrix Q(3 + 2 * d->Sats, 3 + 2 * d->Sats);
	double* Qnn = new double[2 * d->Sats * 2 * d->Sats];
	double ddNSet[2 * MAXCHANNUM];
	XMatrix x(e->X, 3 + 2 * e->nSats, 1);
	XMatrix x_1;
	consEKFPhi(e, d, Phi);
	//Phi.MatrixDis();
	/*构建Q矩阵*/
	consEKFQ(d, Q);
	/*构建P矩阵*/
	consEKFP(e, d, P, Q, Phi);  
	UpdateX(x, x_1, Phi, d);					//状态方程部分构建结束
	/*构建观测方程*/
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	XMatrix H,L,R,K,v,E,t;
	RtkAlignDataIni(&RovSatData);
	for (int i = 0; i < d->Sats; i++)/*计算得到的卫星排序和*/
	{
		CalStaSatDis(b->RealPos, BasObs, &d->DDValue[i], &BasSatData);
		CalStaSatDis(r->Pos, RovObs, &d->DDValue[i], &RovSatData);
	}
	CalRefDis(b->RealPos, BasObs, d->RefPrn, &BasSatData);
	CalRefDis(r->Pos, RovObs, d->RefPrn, &RovSatData);
	RtkInputB(BasSatData, RovSatData, r->Pos, BasObs->SatPvT, H , 0);
	RTKInputL(H, L, r, BasSatData, RovSatData, d);
	consEKFR(R, d);
	calEKFK(K, P, H, R);
	v= H * x_1;
	v = L - v;
	v = K * v;
	x_1 = x_1 + v;/*最后更新完成*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
	//P.MatrixDis();
	EkfPGetQnn(P, Qnn);
	EkfXGetfN(x_1, ddNSet);
	lambda(2 * d->Sats, 2, ddNSet, Qnn, d->FixedAmb, d->FixRMS);
	/*第二次观测值更新，用固定的模糊度进行更新*/
	TwiceUpdate(x_1, P, d->FixedAmb);
	Matrix2Array(x_1, e->X);
	memcpy(r->Pos, e->X, 3 * sizeof(double));
	CalZeroLine(b->RealPos, r->Pos, d->dPos, d->denu);
	updateE(e, d);
	delete[] Qnn;
}

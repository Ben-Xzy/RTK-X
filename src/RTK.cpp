#include"SPP.h"
#include"mathf.h"
#include"RTK.h"
#include"sockets.h"

using namespace std;
/************
����վ��վʱ��ͬ������:
1����ͬ���ɹ���
0����ͬ��ʧ��,����վ�۲�����ʱ��������վ֮��
-1�����ļ���ȡ������
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
		/*�ȶ�����վ����*/
		RLen = fread(Rbuff + Rd, 1, MAXRawLen - Rd, fr);/*len���ص��Ƕ�ȡ����*/
		if (RLen < MAXRawLen - Rd)
		{
			fileFlag = true;
		}/*�����˽�β*/
		Rd = Rd + RLen;//�����ļ����ֽڳ��Ȼָ�MaxRawLen
		int RangeFlag = DecodeNovOem7Dat(Rbuff, Rd, &rawdata->RovEpk, rawdata->GpsEph, rawdata->BdsEph, RovPos);
		if (RangeFlag == 1) break;/*�����˹۲�����*/
		if (fileFlag)
		{
			cout << "�ļ���ȡ����" << endl;
			return -1;/*�����˽�β*/
		}
	}
	/*����֤���θ�������վ���ݺ��dt�Ƿ�С����ֵ*/
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynFileThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
	{
		return 0;
	}
	/*�����վ���ݲ������ݶ��룬������������*/
	while (1)
	{
		while (!feof(fb))
		{
			BLen = fread(Bbuff + Bd, 1, MAXRawLen - Bd, fb);/*len���ص��Ƕ�ȡ����*/
			if (BLen < MAXRawLen - Bd)
			{
				fileFlag = true;
			}
			Bd = BLen + Bd;
			int RangeFlag = DecodeNovOem7Dat(Bbuff, Bd, &rawdata->BasEpk, rawdata->GpsEph, rawdata->BdsEph, BasPos);//1�����������й۲����ݣ����Խ��㣬-1�������ݽضϣ�������ȡ�����Ա����
			if (RangeFlag == 1) { break; }/*�����˹۲�����*/
			if (fileFlag)
			{
				cout << "�ļ���ȡ����" << endl;
				return -1;/*�����˽�β*/
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
		/*�ȶ�����վ����*/
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
			}/*�����˹۲�����*/
		}
		else { cout << "δ���ܵ�����վ����" << endl; return -1; }

	}
	/*����֤���θ�������վ���ݺ��dt�Ƿ�С����ֵ*/
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynSktThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)/*����վ�����ڻ�վ����֮ǰ*/
	{
		return 0;
	}
	/*�����վ���ݲ������ݶ��룬������������*/
	while (1)
	{
		while (1)
		{
			//Sleep(980);
			/*�ȶ�����վ����*/
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
				}/*�����˹۲�����*/
			}
			else { cout << "δ���ܵ�����վ����" << endl; return -1; }
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
���㵥���:
1����������:��վ���ݣ�����վ���ݣ���վ������ݣ�����վ������ݣ���������
2��ע��RTKDATA��obs��fFlag����һ������һ��Ƶ�ʵ�״����-1��������0�����ݣ�1��������
				sdobs��fFlag���������ݵ�Ƶ����Ч�ԣ�-1��������0�����ݣ�1��������
������ֻ�ж������Ƿ���˫Ƶ�����е��������Ϣ��CalSinDifCySlip��������
***************/
void CalStaSinDif(EPOCHOBSDATA* BasEpk,EPOCHOBSDATA *RovEpk,POSRES *BasPos,POSRES *RovPos,SDEPOCHOBS *SdObs)
{
	GPSTIME time = BasEpk->Time;
	memcpy(&SdObs->Time, &time, sizeof(GPSTIME));
	int Prn;
	GNSSSys Sys;
	double dP[2], dL[2];
	double DN[2];
	AmbigInitial(BasEpk, BasPos);/*��վģ���ȳ�ʼ��*/
	AmbigInitial(RovEpk, RovPos);/*����վģ���ȳ�ʼ��*/
	/*����ѭ�������̾��Ի�վ�Ĵ���Ϊ��*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if(BasEpk->Satobs[i].Valid) /*�жϻ�վ�۲������Ƿ���Ч*/
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
		/*Ѱ������վ�ж�Ӧ����ЧPRN��*/
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			/*���PRN��ϵͳ�ܷ��Ӧ��*/
			bool flag = (Prn == RovEpk->Satobs[j].Prn && Sys == RovEpk->Satobs[j].System) ? true : false;
			if (RovEpk->Satobs[j].Valid && flag)/*�ж�����վ��Ч��ͨ������*/
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
				/*���е����*/
				for (int m = 0; m < 2; m++)
				{
					dP[m] = RovEpk->Satobs[j].P[m] - BasEpk->Satobs[i].P[m];

					dL[m] = RovEpk->Satobs[j].L[m] - BasEpk->Satobs[i].L[m];

					
				/*�ж��ǵ�Ƶ����˫Ƶ*/
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
				Prn=0; /*��Prn�ָ�Ĭ��ֵ*/
			}
			else { continue; }
		}
	}
}
/***********************
���õ������ݽ�һ��������⣺
����turboEdit����������������
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
		/*�ȸ��ݷǲ�۲�ֵ�ж��Ƿ�������*/
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
		/*���е���۲�ֵ��MWGF��ϣ��ж��Ƿ�������*/
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
			dMW = Com_Cur[i].MW - SdObs->SdCObs[j].MW;/*�ȼ����ֵdMW�ټ���MW�ĵ�ǰ��Ԫƽ��ֵ������*/
			if (fabs(dGF) < 5e-2 && fabs(dMW) < 3)
			{
				//Obs->SatPvT[i].Valid = true;
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = 1;
				SdObs->SdCObs[i].Prn = SdObs->SdCObs[j].Prn;
				SdObs->SdCObs[i].Sys = SdObs->SdCObs[j].Sys;
				SdObs->SdCObs[i].MW = (SdObs->SdCObs[j].n * SdObs->SdCObs[j].MW + Com_Cur[i].MW) / (SdObs->SdCObs[j].n + 1);
				SdObs->SdCObs[i].n = SdObs->SdCObs[j].n + 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;//Ϊ����obs��PRN˳����룬���ﰴ��obs��˳������
			}
			else/*����ʱ������������MW���ֵӦ�ÿ�ʼ�µ�ƽ��*/
			{
				//Obs->SatPvT[i].Valid = true;
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = -1;
				SdObs->SdCObs[i].Prn = Com_Cur[i].Prn;
				SdObs->SdCObs[i].Sys = Com_Cur[i].Sys;
				SdObs->SdCObs[i].MW = Com_Cur[i].MW;
				SdObs->SdCObs[i].n = 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;  //PIF�������Ƿ����޹�
			}
			if (i != j)
			{
				memset(SdObs->SdCObs+ j, 0, sizeof(MWGF));//�ڴ������i�����еĹ۲�ֵ�󣬽�ԭ��j��MWGF������
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
 ˫���:
 ����������վ���ݣ��������ݣ�˫���������
 ***************/
void CalStaDouDif(EPOCHOBSDATA* RovEpk,SDEPOCHOBS* SdObs, DDCEPOCHOBS* DDObs)
{
	int RefInd=0;
	int GPSnum=0, BDSnum = 0;
	double ScoreMax[2] = { 0.0,0.0 };
	int SatFlag = 0;/*�жϸ�����GPS��0������BDS��1��*/
	/*ѡȡ�ο��ǣ�������վѡȡ*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (SdObs->SdSatObs[i].System == UNKS)/*ֻ����BDS��GPS��������*/
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
			DDObs->RefPrn[SatFlag] = SdObs->SdSatObs[i].Prn;/*BDS��GPS�ֱ���һ���ο���*/
			DDObs->RefPos[SatFlag] = i;/*��ʱ��������ɵ��������е�����*/
		}
	}
	DDObs->DDSatNum[0] = (GPSnum > 1)? GPSnum-1:0;
	DDObs->DDSatNum[1] = (BDSnum > 1)? BDSnum-1:0;
	DDObs->Sats = DDObs->DDSatNum[0] + DDObs->DDSatNum[1];
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		DDCOBS tempObs;
		double tempDDN = 0.0;
		if (SdObs->SdSatObs[i].System == UNKS)/*ֻ����BDS��GPS˫Ƶ����*/
		{
			continue;
		}
		if (!RovEpk->SatPvT[SdObs->SdSatObs[i].nRov].Valid)/*����Ҳ���뿼�Ǵ�������Ҫ��PVT,�˴���BasEpk->SatPvT[SdObs->SdSatObs[i].nBas*/
		{
			continue;
		}
		SatFlag = (SdObs->SdSatObs[i].System == GPS) ? 0 : 1;
		RefInd = DDObs->RefPos[SatFlag];/*��refInd��Ϊ�����Ĳο���������*/
		if(i== RefInd)/*�������غ�������*/
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
	do/*��С���˵���*/
	{
		RtkAlignDataIni(&RAlign);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/* ��ʱRovSatDis�������ź���BasSatDisһ��*/
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
			cout << "��С����δ����" << endl;
			return false;
		}
	} while (fabs(ls.x(0, 0) + ls.x(1, 0) + ls.x(2, 0)) > 1e-6);


}
/*********************
��Զ�λ��С���˺���:
���룺����վ�۲����ݣ���վ�۲����ݣ�����վλ�ã���վλ�ã�
˫�����ݣ���������
********************/
bool RTK(EPOCHOBSDATA *RovObs,EPOCHOBSDATA *BasObs, POSRES* RovPos, POSRES* BasPos,DDCEPOCHOBS *DDObs)
{
	/*����վ�ͻ�վλ�ó�ֵ���Ǻ�����ڵ�����λ�ò���*/
	double DDNSet[2 * MAXCHANNUM];
	double* Qnn=new double[4 * DDObs->Sats * DDObs->Sats];
	memset(Qnn, 0, 4* DDObs->Sats * DDObs->Sats * sizeof(double));
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	LSQ ls;
	int IterNum = 0;
	RecordPrn(DDObs, DDObs->FormPrn, DDObs->FormSys);
	/*��վ�������Ǽ��ξ���*/
	for (int i = 0; i < DDObs->Sats; i++)/*����õ������������*/
	{
		CalStaSatDis(BasPos->RealPos, BasObs, &DDObs->DDValue[i], &BasSatData);	
	}
	CalRefDis(BasPos->RealPos, BasObs, DDObs->RefPrn, &BasSatData);

	do/*��С���˵���*/
	{
		RtkAlignDataIni(&RovSatData);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/* ��ʱRovSatDis�������ź���BasSatDisһ��*/
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
			cout << "��С����δ����" << endl;
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
	//cout << "����������" << IterNum << endl;
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
�μǽ���һ��EKF��Ҫ����e
*************/
void EKF(RTKEKF *e,DDCEPOCHOBS *d,POSRES *r,POSRES *b,XMatrix &P, EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs)
{
	/*�ȹ���״̬ת�ƾ���Phi*/
	XMatrix Phi(3 + 2 * d->Sats, 3 + 2 * e->nSats);
	XMatrix Q(3 + 2 * d->Sats, 3 + 2 * d->Sats);
	double* Qnn = new double[2 * d->Sats * 2 * d->Sats];
	double ddNSet[2 * MAXCHANNUM];
	XMatrix x(e->X, 3 + 2 * e->nSats, 1);
	XMatrix x_1;
	consEKFPhi(e, d, Phi);
	//Phi.MatrixDis();
	/*����Q����*/
	consEKFQ(d, Q);
	/*����P����*/
	consEKFP(e, d, P, Q, Phi);  
	UpdateX(x, x_1, Phi, d);					//״̬���̲��ֹ�������
	/*�����۲ⷽ��*/
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	XMatrix H,L,R,K,v,E,t;
	RtkAlignDataIni(&RovSatData);
	for (int i = 0; i < d->Sats; i++)/*����õ������������*/
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
	x_1 = x_1 + v;/*���������*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
	//P.MatrixDis();
	EkfPGetQnn(P, Qnn);
	EkfXGetfN(x_1, ddNSet);
	lambda(2 * d->Sats, 2, ddNSet, Qnn, d->FixedAmb, d->FixRMS);
	/*�ڶ��ι۲�ֵ���£��ù̶���ģ���Ƚ��и���*/
	TwiceUpdate(x_1, P, d->FixedAmb);
	Matrix2Array(x_1, e->X);
	memcpy(r->Pos, e->X, 3 * sizeof(double));
	CalZeroLine(b->RealPos, r->Pos, d->dPos, d->denu);
	updateE(e, d);
	delete[] Qnn;
}

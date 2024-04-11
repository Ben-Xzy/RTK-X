#include"SPP.h"
#include"mathf.h"
#include"RTK.h"
#include"sockets.h"

using namespace std;
/************
Rover base station time synchronization function:
1 represents the success of synchronization;
0 indicates synchronization failure, that is, the time of the base station observation data is after the rover;
-1 represents the end of the file read;
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
		/*Read the rover data first*/
		RLen = fread(Rbuff + Rd, 1, MAXRawLen - Rd, fr);
		if (RLen < MAXRawLen - Rd)
		{
			fileFlag = true;
		}
		Rd = Rd + RLen;
		int RangeFlag = DecodeNovOem7Dat(Rbuff, Rd, &rawdata->RovEpk, rawdata->GpsEph, rawdata->BdsEph, RovPos);
		if (RangeFlag == 1) break;/*Get Obs data*/
		if (fileFlag)
		{
			cout << "end of file" << endl;
			return -1;
		}
	}
	/*First, verify whether the DT after the rover data is less than the threshold*/
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynFileThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
	{
		return 0;
	}
	/*Decode the base station data and do data alignment, and the results are divided into three situations*/
	while (1)
	{
		while (!feof(fb))
		{
			BLen = fread(Bbuff + Bd, 1, MAXRawLen - Bd, fb);
			if (BLen < MAXRawLen - Bd)
			{
				fileFlag = true;
			}
			Bd = BLen + Bd;
			int RangeFlag = DecodeNovOem7Dat(Bbuff, Bd, &rawdata->BasEpk, rawdata->GpsEph, rawdata->BdsEph, BasPos);
			if (RangeFlag == 1) { break; }
			if (fileFlag)
			{
				cout << "end of file" << endl;
				return -1;
			}
		}

		if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynFileThre)
		{
			return 1;
		}
		if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
		{
			return 0;/*Rover data precedes base station data*/
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
			}
		}
		else { cout << "socket Get no RovObs!" << endl; return -1; }

	}
	if (fabs(rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek) < CFGINFO.RtkSynSktThre)
	{
		return 1;
	}
	if (rawdata->RovEpk.Time.SecOfWeek - rawdata->BasEpk.Time.SecOfWeek < 0)
	{
		return 0;
	}
	while (1)
	{
		while (1)
		{
			//Sleep(980);

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
				}
			}
			else { cout << "socket Get no BasObs!" << endl; return -1; }
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
To calculate the single-difference function:
1. Input data: base station data, rover data, base station result data, rover result data, single difference data
2. Note that the fFlag of obs in RTKDATA represents a satellite with a frequency (-1 has a cycle, 0 has no data, and 1 has no cycle).
				The fFlag of sdobs represents the frequency validity of single-difference data (-1 with cycle slip, 0 with no data, 1 without cycle slip)
This function only determines whether the data is dual-frequency and performs a single difference, and the cycle hop information is calculated in the CalSinDifCySlip function
***************/
void CalStaSinDif(EPOCHOBSDATA* BasEpk,EPOCHOBSDATA *RovEpk,POSRES *BasPos,POSRES *RovPos,SDEPOCHOBS *SdObs)
{
	GPSTIME time = BasEpk->Time;
	memcpy(&SdObs->Time, &time, sizeof(GPSTIME));
	int Prn;
	GNSSSys Sys;
	double dP[2], dL[2];
	double DN[2];
	AmbigInitial(BasEpk, BasPos);
	AmbigInitial(RovEpk, RovPos);
	/*The flow of the whole cycle is dominated by the sequence of base stations*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if(BasEpk->Satobs[i].Valid) 
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
		/*Find the corresponding valid PRN number in the rover*/
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			/*Check whether the PRN and the system are compatible*/
			bool flag = (Prn == RovEpk->Satobs[j].Prn && Sys == RovEpk->Satobs[j].System) ? true : false;
			if (RovEpk->Satobs[j].Valid && flag)/*Judge that the rover is valid and aligned*/
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
				/*Perform single-difference processing*/
				for (int m = 0; m < 2; m++)
				{
					dP[m] = RovEpk->Satobs[j].P[m] - BasEpk->Satobs[i].P[m];

					dL[m] = RovEpk->Satobs[j].L[m] - BasEpk->Satobs[i].L[m];

					
				/*Determine whether it is single or dual frequency*/
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
				Prn=0; /*Restore the Prn to its default value*/
			}
			else { continue; }
		}
	}
}
/***********************
Further cycle slip detection with single difference data:
The turboEdit method was used to perform the cycle slip test
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
		/*First, determine whether there is a weekly slip based on the non-difference observations*/
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
		/*The MWGF combination of single difference observations is performed to determine whether there is a weekly slip*/
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
			dMW = Com_Cur[i].MW - SdObs->SdCObs[j].MW;
			if (fabs(dGF) < 5e-2 && fabs(dMW) < 3)
			{
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = 1;
				SdObs->SdCObs[i].Prn = SdObs->SdCObs[j].Prn;
				SdObs->SdCObs[i].Sys = SdObs->SdCObs[j].Sys;
				SdObs->SdCObs[i].MW = (SdObs->SdCObs[j].n * SdObs->SdCObs[j].MW + Com_Cur[i].MW) / (SdObs->SdCObs[j].n + 1);
				SdObs->SdCObs[i].GF = Com_Cur[i].GF;
				SdObs->SdCObs[i].n = SdObs->SdCObs[j].n + 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;
			}
			else/*At this point, a weekly slip has occurred, and the combined MW value should start a new smoothing*/
			{
				SdObs->SdSatObs[i].fFlag[0] = SdObs->SdSatObs[i].fFlag[1] = -1;
				SdObs->SdCObs[i].Prn = Com_Cur[i].Prn;
				SdObs->SdCObs[i].Sys = Com_Cur[i].Sys;
				SdObs->SdCObs[i].MW = Com_Cur[i].MW;
				SdObs->SdCObs[i].GF = Com_Cur[i].GF;
				SdObs->SdCObs[i].n = 1;
				SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;  //PIF has nothing to do with whether a cycle slip occurs or not
			}
			if (i != j)
			{
				memset(SdObs->SdCObs+ j, 0, sizeof(MWGF));//After processing the observations of the ith sequence, the original j-th MWGF was reset
			}
		}
		else                                                                                                                        
		{
			memcpy(SdObs->SdCObs + i, Com_Cur + i, sizeof(MWGF));
			SdObs->SdCObs[i].n = 1;
			SdObs->SdCObs[i].PIF = Com_Cur[i].PIF;
		}
	}
}
/***************
 Double-difference function:
 Parameters: rover data, single-difference data, double-difference output data
 ***************/
void CalStaDouDif(EPOCHOBSDATA* RovEpk,SDEPOCHOBS* SdObs, DDCEPOCHOBS* DDObs)
{
	int RefInd=0;
	int GPSnum=0, BDSnum = 0;
	double ScoreMax[2] = { 0.0,0.0 };
	int SatFlag = 0;/*GPS(0),BDS(1)*/
	/*Select the reference star, selected from the rover*/
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (SdObs->SdSatObs[i].System == UNKS)/*Only BDS and GPS satellite data are processed*/
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
			DDObs->RefPrn[SatFlag] = SdObs->SdSatObs[i].Prn;/*BDS and GPS each have a reference star*/
			DDObs->RefPos[SatFlag] = i;/*The index becomes an index in a single-difference array*/
		}
	}
	DDObs->DDSatNum[0] = (GPSnum > 1)? GPSnum-1:0;
	DDObs->DDSatNum[1] = (BDSnum > 1)? BDSnum-1:0;
	DDObs->Sats = DDObs->DDSatNum[0] + DDObs->DDSatNum[1];
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		DDCOBS tempObs;
		double tempDDN = 0.0;
		if (SdObs->SdSatObs[i].System == UNKS)/*Only BDS and GPS dual-band data are processed*/
		{
			continue;
		}
		if (!RovEpk->SatPvT[SdObs->SdSatObs[i].nRov].Valid)/*Here it must also be considered that the satellite to be tested must have PVT, which is use BasEpk->SatPvT[SdObs->SdSatObs[i].nBas*/
		{
			continue;
		}
		SatFlag = (SdObs->SdSatObs[i].System == GPS) ? 0 : 1;
		RefInd = DDObs->RefPos[SatFlag];/*Make refInd a true reference satellite index*/
		if(i== RefInd)/*If the satellites coincide, skip it*/
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
	do/*Least-squares iteration*/
	{
		RtkAlignDataIni(&RAlign);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/* At this time, the satellite number of RovSatDis is the same as that of BasSatDis*/
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
			cout << "Fixed RTK LSQ are not converging" << endl;
			return false;
		}
	} while (fabs(ls.x(0, 0) + ls.x(1, 0) + ls.x(2, 0)) > 1e-6);
	return true;

}
/*********************
Relative Positioning Least Squares Function:
Input: Rover Observation Data, Base Station Observation Data, Rover Location, Base Station Location,
Double-difference data, single-difference data
********************/
bool RTK(EPOCHOBSDATA *RovObs,EPOCHOBSDATA *BasObs, POSRES* RovPos, POSRES* BasPos,DDCEPOCHOBS *DDObs)
{
	/*The initial values of the rover and base station are the two positional parameters of the function entrance*/
	double DDNSet[2 * MAXCHANNUM];
	double* Qnn=new double[4 * DDObs->Sats * DDObs->Sats];
	memset(Qnn, 0, 4* DDObs->Sats * DDObs->Sats * sizeof(double));
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	LSQ ls;
	int IterNum = 0;
	RecordPrn(DDObs, DDObs->FormPrn, DDObs->FormSys);
	/*The geometric distance from the base station to the star to be measured*/
	for (int i = 0; i < DDObs->Sats; i++)/*The calculated satellite sorts and sums*/
	{
		CalStaSatDis(BasPos->RealPos, BasObs, &DDObs->DDValue[i], &BasSatData);	
	}
	CalRefDis(BasPos->RealPos, BasObs, DDObs->RefPrn, &BasSatData);

	do/*Least-squares iteration*/
	{
		RtkAlignDataIni(&RovSatData);
		memset(&ls, 0, sizeof(LSQ));
		for (int i = 0; i < DDObs->Sats; i++)/*At this time, the satellite number of RovSatDis is the same as that of BasSatDis*/
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
			DDObs->DDValue[m].ddN[0] = DDObs->DDValue[m].ddN[0] + ls.x(3 + m * 2 + 0, 0);
			DDObs->DDValue[m].ddN[1] = DDObs->DDValue[m].ddN[1] + ls.x(3 + m * 2 + 1, 0);
		}
		IterNum++;
		if (IterNum > 10)
		{
			cout << "RTK LSQ are not converging" << endl;
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
	//cout << "num of iter��" << IterNum << endl;
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
Remember to update e after doing EKF once
*************/
void EKF(RTKEKF *e,DDCEPOCHOBS *d,POSRES *r,POSRES *b,XMatrix &P, EPOCHOBSDATA* RovObs, EPOCHOBSDATA* BasObs)
{
	/*First, construct the state transition matrix Phi*/
	XMatrix Phi(3 + 2 * d->Sats, 3 + 2 * e->nSats);
	XMatrix Q(3 + 2 * d->Sats, 3 + 2 * d->Sats);
	double* Qnn = new double[2 * d->Sats * 2 * d->Sats];
	double ddNSet[2 * MAXCHANNUM];
	XMatrix x(e->X, 3 + 2 * e->nSats, 1);
	XMatrix x_1;
	consEKFPhi(e, d, Phi);
	//Phi.MatrixDis();
	consEKFQ(d, Q);
	consEKFP(e, d, P, Q, Phi);  
	UpdateX(x, x_1, Phi, d);					//The equation of state section is constructed
	/*Construct observational equations*/
	RtkAlignData BasSatData;
	RtkAlignData RovSatData;
	XMatrix H,L,R,K,v,E,t;
	RtkAlignDataIni(&RovSatData);
	for (int i = 0; i < d->Sats; i++)/*The calculated satellite sorts and sums*/
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
	x_1 = x_1 + v;/*end of update*/
	EyeMat(K.row, E);
	t = K * H;
	t = E - t;
	P = t * P;
	//P.MatrixDis();
	EkfPGetQnn(P, Qnn);
	EkfXGetfN(x_1, ddNSet);
	lambda(2 * d->Sats, 2, ddNSet, Qnn, d->FixedAmb, d->FixRMS);
	/*The second observation is updated, with a fixed ambiguity*/
	TwiceUpdate(x_1, P, d->FixedAmb);
	Matrix2Array(x_1, e->X);
	memcpy(r->Pos, e->X, 3 * sizeof(double));
	CalZeroLine(b->RealPos, r->Pos, d->dPos, d->denu);
	CalRatio(d->FixRMS, d->Ratio);
	updateE(e, d);
	delete[] Qnn;
}

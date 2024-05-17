#ifndef DATACLASSSET_H
#define DATACLASSSET_H
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include"TimeCordination.h"
#include<vector>
#include"param.h"
using namespace std;
/*先声明所有结构体的存在*/
struct SATOBSDATA;
struct EPOCHOBSDATA;
struct GPSEPHREC;
struct POSRES;
struct SATMIDRES;
struct MWGF;


enum  GNSSSys
{
	UNKS = 0, GPS, BDS
};
/*Definition of the observed data for each satellite*/
struct SATOBSDATA 
{
	short Prn;
	GNSSSys System;
	double P[2], L[2], D[2];
	bool Valid;/*Determine whether the observation data is complete*/
	double SD_Psr[2], SD_Adr[2], SNR[2], locktime[2];/*Pseudorange standard deviation, phase standard deviation, signal-to-noise ratio, tracking duration (no cycle jump), previous epoch tracking duration*/
	int fFlag[2],Parity[2];/*Judging the observation data: -1 has a cycle slip, 0 has no data, 1 has no cycle slip with data, and the half-cycle slip quality standard*/

	SATOBSDATA()
	{
		Prn = 0;
		System = UNKS;
		for (int i = 0; i < 2; i++)
			P[i] = L[i] = D[i] = 0.0;
		Valid = false;
		fFlag[0]=fFlag[1] = 0;

	}
};
struct MWGF
{
	short Prn;
	GNSSSys Sys;
	double MW;
	double GF;
	double Ntheta;
	double PIF;
	int n; //Smooth counting
	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
		Ntheta = 0.15;
	}

};
/* Intermediate calculations for each satellite's position, speed, clock error, etc */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;/*Satellite clock clock difference, clock speed*/
	double Elevation, Azimuth;
	double Ek;/*Near point corners*/
	double Ek_dot;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid; //false=There is no ephemeris or the ephemeris is expired, true - the calculation is successful, if there is a weekly jump, it will also become false
	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = 90;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};
/*Definition of the observation data for each epoch*/
struct EPOCHOBSDATA 
{
	/********************************************
	1. Before gross error detection, ComObs saves the combined results of the previous epoch. After gross error detection, the result of the current epoch is saved.
	When decoding data, you can only memset the SatObs array.
	2. The satellite order stored in the ComObs and SatPVT arrays is the same as that of the SatObs array, i.e. with the same loop i, the guard can be found
	Observations, satellite positions, and availability of stars.
	***************************************************/
	double FormLocktime[2*MAXCHANNUM];
	GPSTIME Time;
	short SatNum;
	SATOBSDATA Satobs[MAXCHANNUM];/*Only memset on this array*/
	MWGF ComObs[MAXCHANNUM];  /*Save the MW value of the previous epoch and update the MW value of the current epoch*/
	SATMIDRES SatPvT[MAXCHANNUM];/*It should be in the same order as the satellites of ComObs, so that it can be circulated in the same cycle*/
	EPOCHOBSDATA()
	{
		SatNum = 0;
		for (int i = 0; i < 2*MAXCHANNUM; i++)
		{
			FormLocktime[i] = 0.0;
		}
	}
};
/*卫星星历结构体*/
struct GPSEPHREC
{
	unsigned int PRN;
	GNSSSys System;
	GPSTIME TOC, TOE;
	double ClkBias, ClkDrift, ClkDriftRate;
	unsigned int IODE, IODC;
	double A, M0, e, OMEGA0, i0, omega;
	double Crs, Cuc, Cus, Cic, Cis, Crc;
	double DeltaN, OMEGADot, iDot;
	int SVHealth;/*health(0)*/
	double TGD1;
	
};

/*The positioning result structure for each epoch*/
struct POSRES
{
	GPSTIME Time;
	double Pos[3], Vel[3],ReceiTGPS,ReceiTBDS;
	double RealPos[3];
	bool unduFlag;/*Elevation anomaly enablement flags*/
	double PDOP, SigmaPos, SigmaVel;
	double ReceiT_dot;
	double AmbiL1[MAXCHANNUM];
	double AmbiL2[MAXCHANNUM];
	int SatNum;
	bool Valid;
	bool realPosFlag;
	POSRES() 
	{
		Valid = false; 
		unduFlag = true;
		realPosFlag = false;
		RealPos[0]= RealPos[1]= RealPos[2]=0.0;
	}
};

/*  Definition of monodyne observations for each satellite  */
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	int      fFlag[2];/*0 has no data, 1 means no cycle slip, and -1 means there is a cycle slip*/
	double   P[2], L[2];   // m
	double   DN[2];/*Single-difference ambiguity*/
	short    nBas, nRov;   // Store the datum and rover index numbers corresponding to single difference observations

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		P[0] = L[0] = P[1] = L[1] = 0.0;
		DN[0] = DN[1] = 0.0;
		fFlag[0]=fFlag[1] = 0;
	}
};

/*  Definition of one-difference observation data for each epoch  */
struct SDEPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];
	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};
struct DDCOBS
{
	int TarPrn;/*Tar sat*/
	GNSSSys Sys;/*sys of Tar sat*/
	double ddP[2], ddL[2];
	double ddN[2];
	int flag[2];/*-1 represents a weekly cycle, 0 represents no data, and 1 represents data*/
	DDCOBS()
	{
		for (int i = 0; i < 2; i++)
		{
			ddP[i] = ddL[i] = ddN[i] = 0.0;
			flag[i] = 0;
		}
	}
};

/*  Definition of data related to double difference  */
struct DDCEPOCHOBS
{
	int RefPrn[2], RefPos[2];         // Reference satellite number and storage location, 0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // The number of double-difference ambiguity to be estimated, 0=GPS; 1=BDS
	vector<DDCOBS> DDValue;
	double FixedAmb[MAXCHANNUM * 4];  // Including dual-frequency optimal solution [0, AmbNum] and suboptimal solution [AmbNum, 2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA  Ambiguity residuals in floating-point solutions
	double  FixRMS[2];                 // RMS error in fixed solution positioning
	double dPos[3],denu[3];                   // Baseline vectors
	bool bFixed;                      // true is fixed, false is unfixed
	int EkfChange[3 + 2 * MAXCHANNUM]; //Whether the ambiguity of the state of the current epoch needs to be initialized
	int FormPrn[MAXCHANNUM];             //Prn and Sys of the previous epoch
	GNSSSys FormSys[MAXCHANNUM];

	DDCEPOCHOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // The number of double differences for each satellite system
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // The total number of double-difference satellites
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = Ratio = 0.0;
		bFixed = false;
		for (i = 0; i < MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
		for (i = 0; i <3 + MAXCHANNUM * 2; i++)
		{
			EkfChange[i] = 0; 
		}
		for (i = 0; i < 3 + MAXCHANNUM * 2; i++)
		{
			FormPrn[i] = 0; FormSys[i] = UNKS;
		}
	}
};
/*It is convenient to do RTK least-squares, EKF aligned data structures*/
struct RtkAlignData
{
	vector<double> TarDis;
	double RefDis[2];/*Reference star to station distance, 0 is GPS, 1 is BDS*/
	int RefInd[2]; /*The index value of the reference star (corresponding to the PVT index to obs), 0 is GPS, and 1 is BDS*/
	vector<int> PvtInd;/*The set of index values corresponding to each Dis of the aligned data*/
	vector<int>Prn;/*The set of PRNs corresponding to each Dis of the aligned data*/
	vector<GNSSSys> Sys;
	RtkAlignData()
	{
		RefInd[0] = RefInd[1] = -1;
		RefDis[0] = RefDis[1] = 0.0;
	}

};
/*  RTK定位的数据定义  */
struct RTKDATA {
	EPOCHOBSDATA BasEpk;
	EPOCHOBSDATA RovEpk;
	SDEPOCHOBS SdObs;
	DDCEPOCHOBS DDObs;
	GPSEPHREC GpsEph[MAXGPSPRN], BdsEph[MAXBDSPRN];
};

struct RTKEKF
{
	GPSTIME Time;
	int refPrn[2];
	double X[3 + MAXCHANNUM * 2];
	XMatrix P;
	int GBNum[2], nSats, nPrn[MAXCHANNUM];//Index number, number of satellites
	int FixAmb[MAXCHANNUM];          // The ambiguity of the previous epoch that has been fixed and passed after the time update, 1=fixed, -1=unfixed or with a cycle jump
	DDCEPOCHOBS DDObs, CurDDObs;           // Information about the double-difference observations of the previous epoch and the current epoch
	SDEPOCHOBS SDObs;                 // The single difference observation of the previous epoch
	double X0[3 + MAXCHANNUM * 2], P_0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // State backup
	bool IsInit;                      // Whether the filter is initialized
	RTKEKF() {
		IsInit = false;
		nSats = 0;
		refPrn[0] = refPrn[1] = -1;
		GBNum[0] = GBNum[1] = -1;
		for (int i = 0; i < MAXCHANNUM; i++) nPrn[i] = FixAmb[i] = -1;
		for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
			X[i] = X0[i] = 0.0;
			for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P_0[i * (3 + MAXCHANNUM * 2) + j] = 0.0;
		}
	}
};

struct ROVERCFGINFO   // 配置信息
{
	string  IsFileData, RTKProcMode;      // 1=FILE, 0=COM, 1=EKF, 2=LSQ
	string   BasNetIP, RovNetIP;   // ip address
	int  BasNetPort, RovNetPort;       // port
	double CodeNoise, CPNoise, AmbNoise;           // Phase, pseudorange, fixed ambiguity noise (R)
	double PosPIni, AmbPIni;               //Position, Ambiguity, P-Array Initialization (P)
	double PosQErr, AmbQErr;				//Position, Ambiguity Error Transfer Matrix (Q)
	double ElevThreshold;                // Height angle threshold
	double RatioThres;                   // Ratio test threshold

	double RtkSynFileThre, RtkSynSktThre;//RTK alignment time threshold
	string DisModel;                    //Display settings
	bool EmaFlag;						//Whether to enable height angle culling

	string  BasObsDatFile, RovObsDatFile;    //  The file name of the observation data
	string  ResFile;            //  The name of the resulting data file


	ROVERCFGINFO()
	{
		IsFileData = "file";
		RTKProcMode = "EKF";
		DisModel = "screen";
		EmaFlag = false;
		RtkSynFileThre = 0.1;
		RtkSynSktThre = 2.0;
		BasNetPort = RovNetPort = 0;
		CodeNoise = CPNoise = ElevThreshold =AmbNoise= 0.0;
		PosPIni = AmbPIni = 0.0;
		PosQErr = AmbQErr = 0.0;
		RatioThres = 3.0;
	}
};

struct LSQ
{
public:
	XMatrix B;
	XMatrix P;
	XMatrix W;
	XMatrix v;   /*Residuals*/
	XMatrix Q;   /*inv(BTPB)*/
	XMatrix Qnn; /*In the RTK solution, it is the ambiguity covariance matrix*/
	XMatrix x;   /*Number of corrections*/
	double theta; /*Medium error*/
	double PDOP; 
	LSQ()
	{
		theta = 0.0;
		PDOP = 0.0;
	}
};
#endif
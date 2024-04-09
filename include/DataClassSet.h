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
/*���������нṹ��Ĵ���*/
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
/*ÿ�����ǵĹ۲����ݶ���*/
struct SATOBSDATA 
{
	short Prn;
	GNSSSys System;
	double P[2], L[2], D[2];
	bool Valid;/*�жϹ۲������Ƿ�����*/
	double SD_Psr[2],SD_Adr[2], SNR[2], locktime[2],FormLocktime[2];/*α���׼���λ��׼�����ȣ����ٳ���ʱ�䣨��������,��һ��Ԫ����ʱ��*/
	int fFlag[2],Parity[2];/*�жϹ۲�����:-1��������0�����ݣ�1������������,������������׼*/

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
	short Prn;//���Ǻ�
	GNSSSys Sys;
	double MW;
	double GF;
	double PIF;
	int n; //ƽ������
	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};
/* ÿ������λ�á��ٶȺ��Ӳ�ȵ��м������ */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;/*�������Ӳ�,����*/
	double Elevation, Azimuth;
	double Ek;/*ƫ�����*/
	double Ek_dot;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid; //false=û����������������,true-����ɹ�������������Ҳ��false
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
/*ÿ����Ԫ�Ĺ۲����ݶ���*/
struct EPOCHOBSDATA 
{
	/********************************************
	1. �ڴֲ�̽��֮ǰ��ComObs�����ϸ���Ԫ����Ͻ�����ֲ�̽��֮�󣬱��浱ǰ��Ԫ�Ľ����
	���ݽ���ʱ��ֻ�ܶ�SatObs����memset��
	2. ComObs��SatPVT����洢������˳����SatObs������ͬ��������ͬѭ��i�������ҵ�����
	�ǵĹ۲�ֵ������λ�úͿ����ԡ�
	***************************************************/
	GPSTIME Time;
	short SatNum;
	SATOBSDATA Satobs[MAXCHANNUM];/*ֻ�Դ�����memset*/
	MWGF ComObs[MAXCHANNUM];  /*������һ����Ԫ��MWֵ�������±��浱ǰ��Ԫ��MWֵ*/
	SATMIDRES SatPvT[MAXCHANNUM];/*Ҫ��ComObs������˳����ͬ������ͬһ��ѭ��*/
	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};
/*���������ṹ��*/
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
	int SVHealth;/*0�ǽ������������쳣*/
	double TGD1;/*��������������*/
	
};

/*ÿ����Ԫ�Ķ�λ����ṹ��*/
struct POSRES
{
	GPSTIME Time;
	double Pos[3], Vel[3],ReceiTGPS,ReceiTBDS;
	double RealPos[3];
	bool unduFlag;/*�߳��쳣���ñ�־*/
	double PDOP, SigmaPos, SigmaVel;
	double ReceiT_dot;
	double AmbiL1[MAXCHANNUM];
	double AmbiL2[MAXCHANNUM];
	int SatNum;
	bool Valid;
	bool realPosValid;
	POSRES() 
	{
		Valid = false; 
		unduFlag = true;
		realPosValid = false;
		RealPos[0]= RealPos[1]= RealPos[2]=0.0;
	}
};

/*  ÿ�����ǵĵ���۲����ݶ���  */
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	int      fFlag[2];/*0�����ݣ�1������������-1����������*/
	double   P[2], L[2];   // m
	double   DN[2];/*����ģ����*/
	short    nBas, nRov;   // �洢����۲�ֵ��Ӧ�Ļ�׼������վ����ֵ������

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		P[0] = L[0] = P[1] = L[1] = 0.0;
		DN[0] = DN[1] = 0.0;
		fFlag[0]=fFlag[1] = 0;
	}
};

/*  ÿ����Ԫ�ĵ���۲����ݶ���  */
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
	int TarPrn;/*������*/
	GNSSSys Sys;/*�����Ƕ�Ӧϵͳ*/
	double ddP[2], ddL[2];
	double ddN[2];
	int flag[2];/*-1����������0���������ݣ�1����������*/
	DDCOBS()
	{
		for (int i = 0; i < 2; i++)
		{
			ddP[i] = ddL[i] = ddN[i] = 0.0;
			flag[i] = 0;
		}
	}
};

/*  ˫����ص����ݶ���  */
struct DDCEPOCHOBS
{
	int RefPrn[2], RefPos[2];         // �ο������Ǻ���洢λ�ã�0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // ������˫��ģ����������0=GPS; 1=BDS
	vector<DDCOBS> DDValue;
	double FixedAmb[MAXCHANNUM * 4];  // ����˫Ƶ���Ž�[0,AmbNum]�ʹ��Ž�[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA������е�ģ���Ȳв�
	double  FixRMS[2];                 // �̶��ⶨλ��rms���
	double dPos[3],denu[3];                   // ��������
	bool bFixed;                      // trueΪ�̶���falseΪδ�̶�
	int EkfChange[3 + 2 * MAXCHANNUM]; //��ǰ��Ԫ��״̬��ģ�����Ƿ���Ҫ��ʼ��
	int FormPrn[MAXCHANNUM];             //ǰһ����Ԫ��Prn��Sys
	GNSSSys FormSys[MAXCHANNUM];

	DDCEPOCHOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // ������ϵͳ��˫������
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // ˫����������
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
/*������RTK��С����,EKF�Ķ������ݽṹ*/
struct RtkAlignData
{
	vector<double> TarDis;
	double RefDis[2];/*�ο��ǵ�վ����,0ΪGPS��1ΪBDS*/
	int RefInd[2]; /*�ο��ǵ�����ֵ(��Ӧ��obs��PVT������,0ΪGPS��1ΪBDS*/
	vector<int> PvtInd;/*����֮�������ÿ��Dis��Ӧ������ֵ����*/
	vector<int>Prn;/*����֮�������ÿ��Dis��Ӧ��PRN����*/
	vector<GNSSSys> Sys;
	RtkAlignData()
	{
		RefInd[0] = RefInd[1] = -1;
		RefDis[0] = RefDis[1] = 0.0;
	}

};
/*  RTK��λ�����ݶ���  */
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
	double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];// ����Ҫ������δ�õ����ڴ治��
	int GBNum[2], nSats, nPrn[MAXCHANNUM];//�����ţ�������������
	int FixAmb[MAXCHANNUM];          // ʱ����º��ϸ���Ԫ�Ѿ��̶������ݵ�ģ���ȣ� 1=�ѹ̶���-1=δ�̶���������
	DDCEPOCHOBS DDObs, CurDDObs;           // ��һ����Ԫ�͵�ǰ��Ԫ��˫��۲�ֵ��Ϣ
	SDEPOCHOBS SDObs;                 // ��һ����Ԫ�ĵ���۲�ֵ
	double X0[3 + MAXCHANNUM * 2], P_0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // ״̬����
	bool IsInit;                      // �˲��Ƿ��ʼ��
	RTKEKF() {
		IsInit = false;
		nSats = 0;
		refPrn[0] = refPrn[1] = -1;
		GBNum[0] = GBNum[1] = -1;
		for (int i = 0; i < MAXCHANNUM; i++) nPrn[i] = FixAmb[i] = -1;
		for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
			X[i] = X0[i] = 0.0;
			for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i * (3 + MAXCHANNUM * 2) + j] = P_0[i * (3 + MAXCHANNUM * 2) + j] = 0.0;
		}
	}
};

struct ROVERCFGINFO   // ������Ϣ
{
	string  IsFileData, RTKProcMode;      // 1=FILE, 0=COM, 1=EKF, 2=LSQ
	string   BasNetIP, RovNetIP;   // ip address
	int  BasNetPort, RovNetPort;       // port
	double CodeNoise, CPNoise, AmbNoise;           // ��λ��α��,�̶�ģ��������(R)
	double PosPIni, AmbPIni;               //λ�ã�ģ����P���ʼ��(P)
	double PosQErr, AmbQErr;				//λ�ã�ģ�������ת�ƾ���Q��
	double ElevThreshold;                // �߶Ƚ���ֵ
	double RatioThres;                   // Ratio������ֵ

	double RtkSynFileThre, RtkSynSktThre;//RTK����ʱ����ֵ
	string DisModel;                    //��ʾ����
	bool EmaFlag;						//�Ƿ����ø߶Ƚ��޳�

	string  BasObsDatFile, RovObsDatFile;    //  �۲����ݵ��ļ���
	string  ResFile;            //  ��������ļ���


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
typedef struct {/*processing options type*/
	int mode;/* positioning mode (0:SPP 1:RTK) */
	int soltype;/* filter type (0;forward,1:backward,2:combined) */
	int navsys;/* navigation system (1:GPS 2:BDS 3:GPS+BDS)*/
	int nf;/* number of frequencies (1:L1,2:L1+L2)*/
	int obstsys;/* observation time system */
	double elmin;/* elevation mask angle (rad) */
	int ionoopt;/* ionosphere option */
	int tropopt;/* troposphere option */
	double eratio;/* code/phase error ratio */
	double err;/* measurement prior error factor */
	double rb[3];/* base position for relative mode {x,y,z) (ecef)}*/
} prcopt_t;

struct LSQ
{
public:
	XMatrix B;
	XMatrix P;
	XMatrix W;
	XMatrix v;   /*�в�*/
	XMatrix Q;   /*BTPB����*/
	XMatrix Qnn; /*RTK������Ϊģ����Э�������*/
	XMatrix x;   /*������*/
	double theta; /*�����*/
	double PDOP; /*���ξ�������*/
	LSQ()
	{
		theta = 0.0;
		PDOP = 0.0;
	}
};
#endif
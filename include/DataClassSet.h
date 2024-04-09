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
/*每颗卫星的观测数据定义*/
struct SATOBSDATA 
{
	short Prn;
	GNSSSys System;
	double P[2], L[2], D[2];
	bool Valid;/*判断观测数据是否完整*/
	double SD_Psr[2],SD_Adr[2], SNR[2], locktime[2],FormLocktime[2];/*伪距标准差，相位标准差，信噪比，跟踪持续时间（无周跳）,上一历元跟踪时长*/
	int fFlag[2],Parity[2];/*判断观测数据:-1有周跳，0无数据，1无周跳有数据,半周跳质量标准*/

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
	short Prn;//卫星号
	GNSSSys Sys;
	double MW;
	double GF;
	double PIF;
	int n; //平滑计数
	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};
/* 每颗卫星位置、速度和钟差等的中间计算结果 */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;/*卫星钟钟差,钟速*/
	double Elevation, Azimuth;
	double Ek;/*偏近点角*/
	double Ek_dot;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid; //false=没有星历或星历过期,true-计算成功，若有周跳，也变false
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
/*每个历元的观测数据定义*/
struct EPOCHOBSDATA 
{
	/********************************************
	1. 在粗差探测之前，ComObs保存上个历元的组合结果。粗差探测之后，保存当前历元的结果。
	数据解码时，只能对SatObs数组memset。
	2. ComObs和SatPVT数组存储的卫星顺序，与SatObs数组相同，即用相同循环i，可以找到该卫
	星的观测值、卫星位置和可用性。
	***************************************************/
	GPSTIME Time;
	short SatNum;
	SATOBSDATA Satobs[MAXCHANNUM];/*只对此数组memset*/
	MWGF ComObs[MAXCHANNUM];  /*保存上一个历元的MW值，并更新保存当前历元的MW值*/
	SATMIDRES SatPvT[MAXCHANNUM];/*要与ComObs的卫星顺序相同，便于同一个循环*/
	EPOCHOBSDATA()
	{
		SatNum = 0;
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
	int SVHealth;/*0是健康，其他是异常*/
	double TGD1;/*北斗会有两个钟*/
	
};

/*每个历元的定位结果结构体*/
struct POSRES
{
	GPSTIME Time;
	double Pos[3], Vel[3],ReceiTGPS,ReceiTBDS;
	double RealPos[3];
	bool unduFlag;/*高程异常启用标志*/
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

/*  每颗卫星的单差观测数据定义  */
struct SDSATOBS
{
	short    Prn;
	GNSSSys  System;
	int      fFlag[2];/*0无数据，1代表无周跳，-1代表有周跳*/
	double   P[2], L[2];   // m
	double   DN[2];/*单差模糊度*/
	short    nBas, nRov;   // 存储单差观测值对应的基准和流动站的数值索引号

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		P[0] = L[0] = P[1] = L[1] = 0.0;
		DN[0] = DN[1] = 0.0;
		fFlag[0]=fFlag[1] = 0;
	}
};

/*  每个历元的单差观测数据定义  */
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
	int TarPrn;/*其他星*/
	GNSSSys Sys;/*其他星对应系统*/
	double ddP[2], ddL[2];
	double ddN[2];
	int flag[2];/*-1代表周跳，0代表无数据，1代表有数据*/
	DDCOBS()
	{
		for (int i = 0; i < 2; i++)
		{
			ddP[i] = ddL[i] = ddN[i] = 0.0;
			flag[i] = 0;
		}
	}
};

/*  双差相关的数据定义  */
struct DDCEPOCHOBS
{
	int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
	vector<DDCOBS> DDValue;
	double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
	double  FixRMS[2];                 // 固定解定位中rms误差
	double dPos[3],denu[3];                   // 基线向量
	bool bFixed;                      // true为固定，false为未固定
	int EkfChange[3 + 2 * MAXCHANNUM]; //当前历元的状态的模糊度是否需要初始化
	int FormPrn[MAXCHANNUM];             //前一个历元的Prn和Sys
	GNSSSys FormSys[MAXCHANNUM];

	DDCEPOCHOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
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
/*便于做RTK最小二乘,EKF的对齐数据结构*/
struct RtkAlignData
{
	vector<double> TarDis;
	double RefDis[2];/*参考星到站距离,0为GPS，1为BDS*/
	int RefInd[2]; /*参考星的索引值(对应到obs的PVT索引）,0为GPS，1为BDS*/
	vector<int> PvtInd;/*对齐之后的数据每个Dis对应的索引值集合*/
	vector<int>Prn;/*对齐之后的数据每个Dis对应的PRN集合*/
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
	double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];// 矩阵要连续，未用到的内存不管
	int GBNum[2], nSats, nPrn[MAXCHANNUM];//索引号，卫星数，。。
	int FixAmb[MAXCHANNUM];          // 时间更新后上个历元已经固定并传递的模糊度， 1=已固定，-1=未固定或有周跳
	DDCEPOCHOBS DDObs, CurDDObs;           // 上一个历元和当前历元的双差观测值信息
	SDEPOCHOBS SDObs;                 // 上一个历元的单差观测值
	double X0[3 + MAXCHANNUM * 2], P_0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // 状态备份
	bool IsInit;                      // 滤波是否初始化
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

struct ROVERCFGINFO   // 配置信息
{
	string  IsFileData, RTKProcMode;      // 1=FILE, 0=COM, 1=EKF, 2=LSQ
	string   BasNetIP, RovNetIP;   // ip address
	int  BasNetPort, RovNetPort;       // port
	double CodeNoise, CPNoise, AmbNoise;           // 相位，伪距,固定模糊度噪声(R)
	double PosPIni, AmbPIni;               //位置，模糊度P阵初始化(P)
	double PosQErr, AmbQErr;				//位置，模糊度误差转移矩阵（Q）
	double ElevThreshold;                // 高度角阈值
	double RatioThres;                   // Ratio检验阈值

	double RtkSynFileThre, RtkSynSktThre;//RTK对齐时间阈值
	string DisModel;                    //显示设置
	bool EmaFlag;						//是否启用高度角剔除

	string  BasObsDatFile, RovObsDatFile;    //  观测数据的文件名
	string  ResFile;            //  结果数据文件名


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
	XMatrix v;   /*残差*/
	XMatrix Q;   /*BTPB的逆*/
	XMatrix Qnn; /*RTK解算中为模糊度协方差矩阵*/
	XMatrix x;   /*改正数*/
	double theta; /*中误差*/
	double PDOP; /*几何精度因子*/
	LSQ()
	{
		theta = 0.0;
		PDOP = 0.0;
	}
};
#endif
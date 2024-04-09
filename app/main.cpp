#include<iostream>
#include"Matrix.h"
#include"TimeCordination.h"
#include<vector>
#include"ReadBinary.h"
#include"DataClassSet.h"
#include"SPP.h"
#include"sockets.h"
#include<ostream>
#include"RTK.h"
#include"ReConfig.h"

ROVERCFGINFO CFGINFO;
using namespace std;
int main()
{	
	RTKDATA rawdata;
	POSRES RovSppRes;
	POSRES BasSppRes;
	RTKEKF ekf;
	XMatrix ekfP;
	FILE* fb=NULL;
	FILE* fr=NULL;
	SOCKET NetGpsBas, NetGpsRov;
	ofstream outfile;
	string ConfigFile = "../../config/config.ini";
	if (!rr::GetCfgInfo(CFGINFO, ConfigFile))
	{
		cout << "Config file load failed!" << endl;
		return 0;
	}
	else
	{
		cout << "Config file load successfully!" << endl;
	}
	outfile.open(CFGINFO.ResFile, ios::out | ios::binary);
	outfile.flags(ios::fixed);
	outfile.precision(4);
	int Recordflag = 0;
	bool SuccFlag = true;
	int RtkFlag=0;
	if (CFGINFO.IsFileData == "file")
	{
		fb = fopen(CFGINFO.BasObsDatFile.c_str(), "rb");
		if (!fb)
		{
			cout << "open base-station file error!" << endl;
			return 0;
		}
		fr = fopen(CFGINFO.RovObsDatFile.c_str(), "rb");
		if (!fr)
		{
			cout << "open rover-station file error!" << endl;
			return 0;
		}
	}
	else if (CFGINFO.IsFileData == "socket")
	{
		if (OpenSocket(NetGpsBas, CFGINFO.BasNetIP.c_str(), CFGINFO.BasNetPort) == false)
		{
			printf("This Basip & port was not opened.\n");
			return 0;
		}
		if (OpenSocket(NetGpsRov, CFGINFO.RovNetIP.c_str(), CFGINFO.RovNetPort) == false)
		{
			printf("This Rovip & port was not opened.\n");
			return 0;
		}
	}

	while (1)     
	{
		if (CFGINFO.IsFileData == "file")
		{
			RtkFlag = RtkObsSyn(fb, fr, &rawdata, &BasSppRes, &RovSppRes);
		}
		else if (CFGINFO.IsFileData == "socket")
		{
			RtkFlag = RtkObsSyn(NetGpsBas, NetGpsRov, &rawdata, &BasSppRes, &RovSppRes);
		}

		if(RtkFlag==1)
		{
			cout << "Time has syn" << endl;
		}
		else if (RtkFlag == 0)
		{
			cout << "Time syn fail!" << endl;
			continue;
		}
		else if (RtkFlag == -1)
		{
			break;
		}
		DetectOutlier(&rawdata.RovEpk);
		DetectOutlier(&rawdata.BasEpk);
		SuccFlag = SPP(&rawdata.RovEpk, rawdata.GpsEph, rawdata.BdsEph, &RovSppRes);
		if (!SuccFlag)
		{
			continue;
		}
		SPV(&rawdata.RovEpk, &RovSppRes);
		//OutPutResult(&rawdata.RovEpk, RovSppRes, outfile, Recordflag);
		SPP(&rawdata.BasEpk, rawdata.GpsEph, rawdata.BdsEph, &BasSppRes);
		if (BasSppRes.RealPos[0] == 0.0)
		{
			cout << "No bestPos, use Bas result of SPP" << endl;
			memcpy(BasSppRes.RealPos, BasSppRes.Pos, 3 * sizeof(double));
		}
		//SPV(&rawdata.BasEpk, &BasSppRes);
		cout << endl;
		CalStaSinDif(&rawdata.BasEpk,&rawdata.RovEpk,&BasSppRes,&RovSppRes,&rawdata.SdObs);
		DTSinDifCySlip(&rawdata.BasEpk, &rawdata.RovEpk, &rawdata.SdObs);
		CalStaDouDif(&rawdata.RovEpk, &rawdata.SdObs, &rawdata.DDObs);
		if(CFGINFO.RTKProcMode == "LSQ")
		{ 
			RTK(&rawdata.RovEpk, &rawdata.BasEpk,&RovSppRes,&BasSppRes, &rawdata.DDObs);
			OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile, "enu");
		}
		//OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile,"xyz");
		//OutPutResult(&rawdata.RovEpk, RovSppRes, outfile, Recordflag);//ostream��Ϊ��������ʱ����������ô���
		if (CFGINFO.RTKProcMode == "EKF")
		{
			if (!ekf.IsInit)
			{
				EKFinitial(&ekf, &rawdata.DDObs, &RovSppRes, ekfP);
			}
			else
			{
				EKF(&ekf, &rawdata.DDObs, &RovSppRes, &BasSppRes, ekfP, &rawdata.RovEpk, &rawdata.BasEpk);
				OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile, "xyz");
			}
		}
		DDReini(&rawdata.DDObs);
		memset(&rawdata.SdObs, 0, sizeof(SDEPOCHOBS));
	}
	closesocket(NetGpsBas);
	closesocket(NetGpsRov);
}
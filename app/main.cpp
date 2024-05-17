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
	FILE* fb=NULL;
	FILE* fr=NULL;
	SOCKET NetGpsBas, NetGpsRov;
	ofstream outfile;
	string ConfigFile = "config.ini";
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
	bool realPosflag = false;
	int iter = 0;
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
	//BasSppRes.RealPos[0] = -2267806.1676;
	//BasSppRes.RealPos[1] = 5009345.5601;
	//BasSppRes.RealPos[2] = 3220997.5054;
	//BasSppRes.realPosFlag = true;
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
			//cout << "Time has syn" << endl;
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
		//cout << rawdata.RovEpk.Time.SecOfWeek << endl;
		if (!SuccFlag)
		{
			continue;
		}
		SPV(&rawdata.RovEpk, &RovSppRes);
		//OutPutResult(&rawdata.RovEpk, RovSppRes, outfile, Recordflag);//ostream作为函数参数时必须采用引用传递
		SPP(&rawdata.BasEpk, rawdata.GpsEph, rawdata.BdsEph, &BasSppRes);
		if (!BasSppRes.realPosFlag)
		{
			cout << "No bestPos，use Bas result of SPP" << endl;
			memcpy(BasSppRes.RealPos, BasSppRes.Pos, 3 * sizeof(double));
			if (iter > 5)
			{
				BasSppRes.realPosFlag = true;
			}
		}
		if ((!realPosflag) && BasSppRes.realPosFlag)
		{
			outfile.flags(ios::fixed);
			outfile.precision(4);
			outfile << "% ref pos   :" << BasSppRes.RealPos[0] << "   " << BasSppRes.RealPos[1] << "   " << BasSppRes.RealPos[2] << endl;
			realPosflag = true;
			outfile << "%"<<endl;
			outfile << "% (x/y/z-ecef=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites)" << endl;
			outfile << "%  GPST                      x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio" << endl;
		}
		//SPV(&rawdata.BasEpk, &BasSppRes);
		//cout << endl;
		CalStaSinDif(&rawdata.BasEpk,&rawdata.RovEpk,&BasSppRes,&RovSppRes,&rawdata.SdObs);
		DTSinDifCySlip(&rawdata.BasEpk, &rawdata.RovEpk, &rawdata.SdObs);
		CalStaDouDif(&rawdata.RovEpk, &rawdata.SdObs, &rawdata.DDObs);/*不要忘记把DDObs的列表给clear掉*/
		if(CFGINFO.RTKProcMode == "LSQ")
		{ 
			RTK(&rawdata.RovEpk, &rawdata.BasEpk,&RovSppRes,&BasSppRes, &rawdata.DDObs);
			LibOutput(rawdata.BasEpk.Time, rawdata.DDObs, outfile, RovSppRes);
			//OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile,"enu", RovSppRes);
		}
		//OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile,"xyz");
		//OutPutResult(&rawdata.RovEpk, RovSppRes, outfile, Recordflag);//ostream作为函数参数时必须采用引用传递
		if (CFGINFO.RTKProcMode == "EKF")
		{
			if (!ekf.IsInit)
			{
				EKFinitial(&ekf, &rawdata.DDObs, &RovSppRes, ekf.P);
			}
			else
			{
				EKF(&ekf, &rawdata.DDObs, &RovSppRes, &BasSppRes, ekf.P, &rawdata.RovEpk, &rawdata.BasEpk);
				LibOutput(rawdata.BasEpk.Time, rawdata.DDObs, outfile,ekf);
				//OutputRTK(rawdata.BasEpk.Time, rawdata.DDObs, outfile, "enu", RovSppRes);
			}
		}
		DDReini(&rawdata.DDObs);
		SDObsReIni(&rawdata.SdObs);
		iter++;
	}
	closesocket(NetGpsBas);
	closesocket(NetGpsRov);
	delete fb, fr;
}
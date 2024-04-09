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
//#include <filesystem>
//namespace fs = std::filesystem;
ROVERCFGINFO CFGINFO;
using namespace std;
int main()
{	
	//EPOCHOBSDATA obs;
	//GPSEPHREC GPSep[MAXGPSPRN];
	//GPSEPHREC BDSep[MAXBDSPRN];
	//int Choice;
	//int Recordflag = 0;
	//int SppFlag=0;//�Ƿ�λ����ı�־,1��������obs���ݣ�0��������״̬��-1�������������ݿ��Ѷ���
	//POSRES posres;
	//int d = 0; int len=0;
	//bool SuccFlag;
	//ofstream outfile;
	//outfile.open("result.txt",ios::out| ios::binary);
	//outfile.flags(ios::fixed);
	//outfile.precision(4);
	//unsigned char buff[MAXRawLen]; /*���ݻ�����*/
	//cout << "*******************��ѡ���ļ���ȡ��0����ʵʱ��������ȡ��1��********************" << endl;
	//cin >> Choice;
	//cout << "*************************��Ļ��ӡ��0��Ĭ�ϣ�|| �ļ������1��***********************************" << endl;
	//cin >> Recordflag;
	//if (Choice == 0)
	//{
	//	char filename[] = "C:\\Users\\���װ�������\\Music\\Desktop\\���ǳ������\\OEM719-1126\\202310301810.oem719";//202010261820.oem719
	//	//char filename[] = "C:\\Users\\���װ�������\\Music\\Desktop\\���ǳ������\\OEM7dataV1.1\\20240301\\rove.log";
	//	FILE* file = fopen(filename, "rb");
	//	if (!file)
	//	{
	//		cout << "open file error!" << endl;
	//		return 0;
	//	}
	//	while (!feof(file))
	//	{

	//		len = fread(buff + d, 1, MAXRawLen - d, file);/*len���ص��Ƕ�ȡ����*/
	//		if (len < MAXRawLen - d)
	//		{
	//			cout << "��ȡ�ļ�����" << endl;
	//		}
	//		d = 0;//d�������кܶ࣬��Сѭ���б�Ƕ�ȡobs�������λ�ã���ѭ���б�ʾ�ض����ݿ��С��
	//		SppFlag= DecodeNovOem7Dat(buff, d, &obs, GPSep, BDSep, &posres);
	//		if (SppFlag == 1)
	//		{
	//			DetectOutlier(&obs);
	//			SuccFlag=SPP(&obs, GPSep, BDSep, &posres);
	//			if (!SuccFlag)
	//			{
	//				continue;
	//			}
	//			SPV(&obs, &posres);
	//			OutPutResult(&obs, posres, outfile,Recordflag);//ostream��Ϊ��������ʱ����������ô���
	//		}
	//		SppFlag = 0;
	//	}
	//	fclose(file);
	//}
	//else if (Choice == 1)
	//{
	//	SOCKET NetGps;
	//	if (OpenSocket(NetGps, "47.114.134.129", 7190) == false)
	//	/*if (OpenSocket(NetGps, "8.140.46.126", 5002) == false)*/
	//	{
	//		printf("This ip & port was not opened.\n");
	//		return 0;
	//	}
	//	while (1)
	//	{
	//		Sleep(980);
	//		if ((len = recv(NetGps, (char*)buff+d, MAXRawLen-d, 0)) > 0)
	//		{
	//	/*		cout << len + d << endl;*/
	//			d = 0;
	//			SppFlag = DecodeNovOem7Dat(buff, d, &obs, GPSep, BDSep,&posres);
	//			if (SppFlag == 1)
	//			{
	//				DetectOutlier(&obs);
	//				SuccFlag=SPP(&obs, GPSep, BDSep, &posres);
	//				if(!SuccFlag)
	//				{
	//					continue;
	//				}
	//				SPV(&obs, &posres);
	//				OutPutResult(&obs, posres,outfile,Recordflag);
	//			}
	//			
	//			SppFlag = 0;
	//		}
	//	}
	//}
	//outfile.close();
	RTKDATA rawdata;
	POSRES RovSppRes;
	POSRES BasSppRes;
	RTKEKF ekf;
	XMatrix ekfP;
	FILE* fb=NULL;
	FILE* fr=NULL;
	SOCKET NetGpsBas, NetGpsRov;
	ofstream outfile;
	//fs::path p = fs::current_path();
	//p=p.parent_path();
	//string p_str{p.u8string()};
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
		//OutPutResult(&rawdata.RovEpk, RovSppRes, outfile, Recordflag);//ostream��Ϊ��������ʱ����������ô���
		SPP(&rawdata.BasEpk, rawdata.GpsEph, rawdata.BdsEph, &BasSppRes);
		if (BasSppRes.RealPos[0] == 0.0)
		{
			//cout << "����ֵ���꣨bestPos)����Bas��SPP�����Ϊ��ֵ����" << endl;
			memcpy(BasSppRes.RealPos, BasSppRes.Pos, 3 * sizeof(double));
		}
		//SPV(&rawdata.BasEpk, &BasSppRes);
		cout << endl;
		CalStaSinDif(&rawdata.BasEpk,&rawdata.RovEpk,&BasSppRes,&RovSppRes,&rawdata.SdObs);
		DTSinDifCySlip(&rawdata.BasEpk, &rawdata.RovEpk, &rawdata.SdObs);
		CalStaDouDif(&rawdata.RovEpk, &rawdata.SdObs, &rawdata.DDObs);/*��Ҫ���ǰ�DDObs���б���clear��*/
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
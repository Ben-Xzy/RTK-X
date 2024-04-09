
#include"TimeCordination.h"
/*地球椭球采用WGS-84椭球体，第一偏心率已在头文件定义*/
using namespace std;
/********************
COMMENTIME内部成员函数定义和构造函数定义
*********************/
void Com2JD(COMMONTIME *COM,JDTIME *JD)
{
	unsigned short y, m;
	double TempJD, UT;
	unsigned short Month = COM->Month;
	if (Month <= 2) { y = COM->Year - 1; m = COM->Month + 12; }
	else { y = COM->Year; m = COM->Month; }
	UT = COM->Hour + COM->Minute / 60.0 + COM->Second / 3600.0;
	TempJD= floor((365.25 * y)) + floor((30.6001 * (m + 1))) + COM->Day + UT / 24.0 + 1720981.5;
	JD->Days = (int)(TempJD);
	JD->FracDay= (TempJD)-JD->Days;
	FromJDGetMJD(JD);
}
/********************
GPSTIME内部成员函数定义和构造函数定义
*********************/
void GPST2JD(GPSTIME* GPST, JDTIME* JD)//GPST先转化到简化儒略日
{
	double tempMJD;
	double WS;
	WS= GPST->Week * 7 + GPST->SecOfWeek / 86400;
	tempMJD = 44244 + WS;
	JD->MJDDays = (int)(tempMJD);
	JD->MJDFracDay = fmod(WS, 1.0);
	FromMJDGetJD(JD);
}
/********************
MJDTIME内部成员函数定义和构造函数定义
*********************/
void JD2GPST(JDTIME* JD, GPSTIME* GPS)//内部采用简化儒略日准到GPST
{
	if (JD->MJDDays + JD->MJDFracDay < 1)
	{
		FromJDGetMJD(JD);
	}
	else if (JD->Days+JD->FracDay<1&&JD->MJDDays+JD->MJDFracDay<1)
	{
		cout << "未初始化儒略日结构体" << endl;
	}
	else
	{
		double RecordDay = JD->MJDDays + JD->MJDFracDay - 44244;
		GPS->Week = (int)(RecordDay / 7.0);
		GPS->SecOfWeek = (RecordDay - double(GPS->Week*7)) * 86400+18;//最后GPS时结果需要闰秒
	}
}
void JD2Com(JDTIME *JD,COMMONTIME *COM)//儒略日到通用时
{
	int a, b, c, d, e;
	a = (int)(JD->Days + JD->FracDay + 0.5);
	b = a + 1537;
	c = (int)((1.0 * b - 122.1) / 365.25);
	d = (int)(365.25 * c);
	e = (int)((b - d) * 1.0 / 30.6001);
	COM->Day = b - d - (int)(30.6001 * e) + fmod(JD->FracDay  + 0.5, 1.0);
	COM->Month= e - 1 - 12 * (int)(e * 1.0 / 14);
	COM->Year = c - 4715 - (int)((7 + COM->Month) / 10);
	COM->Hour= 12+int(JD->FracDay * 24);
	COM->Minute= int((JD->FracDay * 24 - COM->Hour+12) * 60);
	COM->Second = (JD->FracDay * 24  - COM->Hour +12)* 3600 - COM->Minute * 60;
}
void FromJDGetMJD(JDTIME *JD)
{
	if (JD->FracDay < 0.5)
	{
		JD->MJDDays = JD->Days - 2400001;
		JD->MJDFracDay = JD->FracDay + 0.5;
	}
	else
	{
		JD->MJDDays = JD->Days - 2400000;
		JD->MJDFracDay = JD->FracDay - 0.5;
	}
}
void FromMJDGetJD(JDTIME* JD)
{
	if (JD->MJDFracDay < 0.5)
	{
		JD->Days = JD->MJDDays + 2400000;
		JD->FracDay = JD->MJDFracDay + 0.5;
	}
	else
	{
		JD->Days = JD->Days + 2400001;
		JD->FracDay =JD->MJDFracDay - 0.5;
	}
}
/********************
BLH内部成员函数定义和构造函数定义
*********************/
void Blh2Xyz(double* blh, double* xyz)
{

	double b = blh[0]*rad, l = blh[1]*rad, h = blh[2];
	
	double a = 6378137.0;//椭球长半轴
	double N = a / (sqrt(1 - eSquared * sin(b) * sin(b)));
	xyz[0] = (N + h) * cos(b) * cos(l);
	xyz[1] = (N + h) * cos(b) * sin(l);
	xyz[2] = (N * (1 - eSquared) + h) * sin(b);
}
/********************
XYZ内部成员函数定义和构造函数定义
*********************/
double CountW(double B)
/*blh2Xyz函数中计算W值*/
{
	return  sqrt(1 - eSquared * sin(B) * sin(B));
}
void Xyz2Blh(double* xyz, double* blh)
{
	double a = 6378137.0;//椭球长半轴
	double X = xyz[0];
	double Y = xyz[1];
	double Z = xyz[2];
	double R = sqrt(X * X + Y * Y + Z * Z);
	double phi = atan(Z/ sqrt(X * X + Y * Y));
	double b = blh[0], l = blh[1], h = blh[2];
	/* atan取值范围为-90到90，atan2是-180到180 */
	blh[1] = atan2(Y, X);//-180到180
	//初始化B为0，迭代出B的最终结果
	double tempB = atan(tan(phi) * (1 + a * eSquared * sin(0) / (Z * CountW(0))));
	blh[0] = atan(tan(phi) * (1 + a * eSquared * sin(tempB) / (Z * CountW(tempB))));;//-90到90
	while (abs(blh[0] - tempB) > 1e-10)
	{
		tempB = blh[0];
		blh[0] = atan(tan(phi) * (1 + a * eSquared * sin(tempB) / (Z * CountW(tempB))));
	}
	double W = CountW(blh[0]);
	double N = a / W;
	blh[2] = R * cos(phi) / cos(blh[0]) - N;
}
/********************
NEU内部成员函数定义和构造函数定义
*********************/
void InPutR(double *blh, XMatrix &Rdata)
/*Pos2NEU函数中计算旋转矩阵*/
{
	double b = blh[0], l = blh[1], h = blh[2];
	Rdata(0,0) = -sin(l); Rdata(0,1) = cos(l); Rdata(0,2) = 0;
	Rdata(1,0) = -sin(b) * cos(l); Rdata(1,1) = -sin(b) * sin(l); Rdata(1,2) = cos(b);
	Rdata(2,0) = cos(b) * cos(l); Rdata(2,1) = cos(b) * sin(l); Rdata(2,2) = sin(b);
};//R矩阵的数据
void XyzMatToBLH(XMatrix Mat,double *blh)
{
	double xyz[3];
	xyz[0]= Mat.matrix[0];
	xyz[1] = Mat.matrix[1];
	xyz[2] = Mat.matrix[2];
	Xyz2Blh(xyz, blh);
}
void CompEnudPos(ENU* enu)
/********

********/
{
	double blh[3];//坐标原点blh
	if (enu->Valid == true)
	{
		XyzMatToBLH(enu->PosStation, blh);
		XMatrix R(3, 3);
		InPutR(blh, R);
		XMatrix temp = (enu->PosMat - enu->PosStation);
		XMatrix resMat = R * temp;
		copy(resMat.matrix.begin(), resMat.matrix.end(), enu->dEnu);
	}
	else { cout << "缺少基站原点，转化失败" << endl; }
}
void CompSatElAz(ENU* enu)
{
	CompEnudPos(enu);
	enu->Elev = atan(enu->dEnu[2]/sqrt(enu->dEnu[1] * enu->dEnu[1] + enu->dEnu[0] * enu->dEnu[0]));
	enu->Azim = atan2(enu->dEnu[0], enu->dEnu[1]);
	if (enu->Azim < 0)
		enu->Azim += 2 * PI;
	if (enu->Azim > 2 * PI)
		enu->Azim -= 2 * PI;
}
void BDST2GPST(GPSTIME* GPS,GPSTIME* BDS)
{
	GPS->SecOfWeek = BDS->SecOfWeek + 14;
	GPS->Week = BDS->Week + 1356;
}
void GPST2BDST(GPSTIME* BDS, GPSTIME* GPS)
{
	BDS->SecOfWeek = GPS->SecOfWeek - 14;
	BDS->Week = GPS->Week - 1356;
}
double GPSTMius(GPSTIME* T1, GPSTIME* T2)
{
	double res = T1->SecOfWeek - T2->SecOfWeek;
	if (res > 302400)
	{
		res = res - 604800;
	}
	else if (res < -302400)
	{
		res = res + 604800;
	}
	else
	{
		res = res;
	}
	return res;
}

#include"TimeCordination.h"
/*The Earth ellipsoid uses a WGS-84 ellipsoid, and the first eccentricity has been defined in the header file*/
using namespace std;

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

void GPST2JD(GPSTIME* GPST, JDTIME* JD)//GPST was first converted to simplified Julian Day
{
	double tempMJD;
	double WS;
	WS= GPST->Week * 7 + GPST->SecOfWeek / 86400;
	tempMJD = 44244 + WS;
	JD->MJDDays = (int)(tempMJD);
	JD->MJDFracDay = fmod(WS, 1.0);
	FromMJDGetJD(JD);
}

void JD2GPST(JDTIME* JD, GPSTIME* GPS)//The interior is simplified Julian to GPST
{
	if (JD->MJDDays + JD->MJDFracDay < 1)
	{
		FromJDGetMJD(JD);
	}
	else if (JD->Days+JD->FracDay<1&&JD->MJDDays+JD->MJDFracDay<1)
	{
		cout << "The Julian structure is not initialized" << endl;
	}
	else
	{
		double RecordDay = JD->MJDDays + JD->MJDFracDay - 44244;
		GPS->Week = (int)(RecordDay / 7.0);
		GPS->SecOfWeek = (RecordDay - double(GPS->Week*7)) * 86400+18;//The last GPS time results take leap seconds
	}
}
void JD2Com(JDTIME *JD,COMMONTIME *COM)//Julian Day to Universal Time
{
	int a, b, c, d, e;
	int count;
	a = (int)(JD->Days + JD->FracDay + 0.5);
	b = a + 1537;
	c = (int)((1.0 * b - 122.1) / 365.25);
	d = (int)(365.25 * c);
	e = (int)((b - d) * 1.0 / 30.6001);
	COM->Day = b - d - (int)(30.6001 * e) + (int)fmod(JD->FracDay  + 0.5, 1.0);
	COM->Month= e - 1 - 12 * (int)(e * 1.0 / 14);
	COM->Year = c - 4715 - (int)((7 + COM->Month) / 10);
	COM->Hour= 12+int(JD->FracDay * 24);
	COM->Minute = int((JD->FracDay * 24 - COM->Hour + 12) * 60);
	COM->Second = (JD->FracDay * 24 - COM->Hour + 12) * 3600 - COM->Minute * 60;
	if ((count=COM->Hour/24)>0)
	{
		COM->Hour = COM->Hour - count * 24;
		COM->Day = COM->Day + count - 1;
	}
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
		JD->Days = JD->MJDDays + 2400001;
		JD->FracDay =JD->MJDFracDay - 0.5;
	}
}

void Blh2Xyz(double* blh, double* xyz)
{

	double b = blh[0]*rad, l = blh[1]*rad, h = blh[2];
	
	double a = 6378137.0;
	double N = a / (sqrt(1 - eSquared * sin(b) * sin(b)));
	xyz[0] = (N + h) * cos(b) * cos(l);
	xyz[1] = (N + h) * cos(b) * sin(l);
	xyz[2] = (N * (1 - eSquared) + h) * sin(b);
}

double CountW(double B)
/*W in blh2Xyz function*/
{
	return  sqrt(1 - eSquared * sin(B) * sin(B));
}
void Xyz2Blh(double* xyz, double* blh)
{
	double a = 6378137.0;
	double X = xyz[0];
	double Y = xyz[1];
	double Z = xyz[2];
	double R = sqrt(X * X + Y * Y + Z * Z);
	double phi = atan(Z/ sqrt(X * X + Y * Y));
	double b = blh[0], l = blh[1], h = blh[2];
	/* The value of atan can range from -90 to 90, and the value of atan2 can range from -180 to 180 */
	blh[1] = atan2(Y, X);//-180µ½180

	double tempB = atan(tan(phi) * (1 + a * eSquared * sin(0) / (Z * CountW(0))));
	blh[0] = atan(tan(phi) * (1 + a * eSquared * sin(tempB) / (Z * CountW(tempB))));;
	while (abs(blh[0] - tempB) > 1e-10)
	{
		tempB = blh[0];
		blh[0] = atan(tan(phi) * (1 + a * eSquared * sin(tempB) / (Z * CountW(tempB))));
	}
	double W = CountW(blh[0]);
	double N = a / W;
	blh[2] = R * cos(phi) / cos(blh[0]) - N;
}

void InPutR(double *blh, XMatrix &Rdata)
/*Rotate mat in Pos2NEU function*/
{
	double b = blh[0], l = blh[1], h = blh[2];
	Rdata(0,0) = -sin(l); Rdata(0,1) = cos(l); Rdata(0,2) = 0;
	Rdata(1,0) = -sin(b) * cos(l); Rdata(1,1) = -sin(b) * sin(l); Rdata(1,2) = cos(b);
	Rdata(2,0) = cos(b) * cos(l); Rdata(2,1) = cos(b) * sin(l); Rdata(2,2) = sin(b);
};
void XyzMatToBLH(XMatrix Mat,double *blh)
{
	double xyz[3];
	xyz[0]= Mat.matrix[0];
	xyz[1] = Mat.matrix[1];
	xyz[2] = Mat.matrix[2];
	Xyz2Blh(xyz, blh);
}
void CompEnudPos(ENU* enu)
{
	double blh[3];
	if (enu->Valid == true)
	{
		XyzMatToBLH(enu->PosStation, blh);
		XMatrix R(3, 3);
		InPutR(blh, R);
		XMatrix temp = (enu->PosMat - enu->PosStation);
		XMatrix resMat = R * temp;
		copy(resMat.matrix.begin(), resMat.matrix.end(), enu->dEnu);
	}
	else { cout << "no Pos of Bas,fail!" << endl; }
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
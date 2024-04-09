#include"SPP.h"
#include"DataClassSet.h"
#include<iostream>
#include<fstream>
#include"mathf.h"
using namespace std;
bool CompSatClkOff(GPSTIME* t, GPSEPHREC* RealEph, SATMIDRES* Mid, bool flag)
{
	int health = RealEph->SVHealth;
	if (health != 0) return false;/*�ж������Ƿ񽡿�*/
	GPSTIME toc;
	double F;
	if (RealEph->System == GPS)
	{
		toc = RealEph->TOC;
		F= -2 * sqrt(GPS_GM) / (CLight * CLight);
	}
	else if (RealEph->System == BDS)
	{
		BDST2GPST(&toc, &RealEph->TOC);
		F = -2 * sqrt(BDS_GM) / (CLight * CLight);
	}
	/*�����Ӳ�*/
	double tk = GPSTMius(t, &toc);
	double delta_tr = (flag) ? F * RealEph->e * sqrt(RealEph->A) * sin(Mid->Ek) : 0;
	double delta_tsv = RealEph->ClkBias + RealEph->ClkDrift * tk + RealEph->ClkDriftRate * tk * tk+ delta_tr;
	Mid->SatClkOft = delta_tsv;
	/*��������*/
	double delta_trDot = (flag) ? F * RealEph->e * sqrt(RealEph->A) * cos(Mid->Ek) * Mid->Ek_dot : 0;
	double delta_tsvDot =RealEph->ClkDrift  +2* RealEph->ClkDriftRate *tk+ delta_trDot;
	Mid->SatClkSft = delta_tsvDot;
	Mid->Tgd1 = RealEph->TGD1;

}
int CompGPSSatPVT(const int Prn,  GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	/*��������λ��*/
	double A = Eph->A; /*������*/
	double n0 = sqrt(GPS_GM / (A * A * A));/*ƽ���˶����ٶ�*/
	GPSTIME toe = Eph->TOE;
	double tk = GPSTMius(t,&toe);/*�����������Ԫ�Ĳο�ʱ��*/
	double n = n0 + Eph->DeltaN;/*��ƽ�����ٶȽ��и���*/
	double Mk = Eph->M0 + n * tk;/*����ƽ�����*/
	/*��������ƫ�����*/
	const double Threshold = 1e-2 / 4e9;/*��ֵ��Ϊ1mm�Ļ������룬��1mm/4000km�ĽǶȱ仯*/
	double Ek = Mk, E0 = 0;
	while (fabs(Ek - E0) > Threshold)
	{
		E0 = Ek;
		Ek = Mk + Eph->e * sin(E0);
	}
	double vk = Countvk(Eph->e, Ek);	/*��ƫ����Ǽ���������*/
	double phik = vk + Eph->omega; /*���������Ǿ�*/
	double delta_uk = Eph->Cus * sin(2 * phik) + Eph->Cuc * cos(2 * phik);/*������׵��͸�����*/
	double delta_rk = Eph->Crs * sin(2 * phik) + Eph->Crc * cos(2 * phik);
	double delta_ik = Eph->Cis * sin(2 * phik) + Eph->Cic * cos(2 * phik);
	double uk = phik + delta_uk;/*������Ǿ�*/
	double rk = A * (1 - Eph->e * cos(Ek)) + delta_rk;/*������*/
	double ik = Eph->i0 + delta_ik + (Eph->iDot) * tk;/*�����Ĺ�����*/
	double Orbit_xk = rk * cos(uk);/*�����ڹ��ƽ���ϵ����*/
	double Orbit_yk = rk * sin(uk);
	double OMEGAk = Eph->OMEGA0 + (Eph->OMEGADot - GPS_OMEGAEARTH) * tk - GPS_OMEGAEARTH * toe.SecOfWeek;
	Mid->SatPos[0] = Orbit_xk * cos(OMEGAk) - Orbit_yk * cos(ik) * sin(OMEGAk);
	Mid->SatPos[1]= Orbit_xk * sin(OMEGAk) + Orbit_yk * cos(ik) * cos(OMEGAk);
	Mid->SatPos[2] = Orbit_yk * sin(ik);
	Mid->Ek = Ek;

	/*���������ٶ�*/
	double e = Eph->e;
	double Ek_dot = n / (1 - e * cos(Ek));
	double phi_dot = sqrt((1 + e) / (1 - e)) * pow((cos(vk / 2) / cos(Ek / 2)), 2) * Ek_dot;
	double uk_dot = 2 * (Eph->Cus * cos(2 * phik) - Eph->Cuc * sin(2 * phik)) * phi_dot + phi_dot;
	double rk_dot = A * e * sin(Ek) * Ek_dot + 2 * (Eph->Crs * cos(2 * phik) - Eph->Crc * sin(2 * phik))*phi_dot;
	double Ik_dot = Eph->iDot + 2 * (Eph->Cis * cos(2 * phik) - Eph->Cic * sin(2 * phik)) * phi_dot;
	double OMEGAk_dot = Eph->OMEGADot - GPS_OMEGAEARTH;
	XMatrix RDot(3,4),Vec(4, 1),Vel(3,1);
	inputRDot(RDot, OMEGAk, ik, Orbit_xk, Orbit_yk);
	Vec(0, 0) = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
	Vec(1, 0) = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
	Vec(2, 0) = OMEGAk_dot;
	Vec(3, 0) = Ik_dot;

	Vel = RDot * Vec;
	Mid->SatVel[0] = Vel(0, 0);
	Mid->SatVel[1] = Vel(1, 0);
	Mid->SatVel[2] = Vel(2, 0);
	Mid->Ek_dot = Ek_dot;

	return 1;
}
int CompBDSSatPVT(const int Prn, GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	/*��������λ��*/
	double A = Eph->A; /*������*/
	double n0 = sqrt(BDS_GM / (A * A * A));/*ƽ���˶����ٶ�*/
	GPSTIME toe = Eph->TOE;
	GPSTIME BDS2GPSt;
	GPST2BDST(&BDS2GPSt, t);
	double tk = GPSTMius(&BDS2GPSt,&toe);/*�����������Ԫ�Ĳο�ʱ��*/
	double n = n0 + Eph->DeltaN;/*��ƽ�����ٶȽ��и���*/
	double Mk = Eph->M0 + n * tk;/*����ƽ�����*/
	/*��������ƫ�����*/
	const double Threshold = 1e-2 / 4e11;/*��ֵ��Ϊ1mm�Ļ������룬��1mm/4000km�ĽǶȱ仯*/
	double Ek = Mk, E0 = 0;
	while (fabs(Ek - E0) > Threshold)
	{
		E0 = Ek;
		Ek = Mk + Eph->e * sin(E0);
	}
	double vk = Countvk(Eph->e, Ek);	/*��ƫ����Ǽ���������*/
	double phik = vk + Eph->omega; /*���������Ǿ�*/
	double delta_uk = Eph->Cus * sin(2 * phik) + Eph->Cuc * cos(2 * phik);/*������׵��͸�����*/
	double delta_rk = Eph->Crs * sin(2 * phik) + Eph->Crc * cos(2 * phik);
	double delta_ik = Eph->Cis * sin(2 * phik) + Eph->Cic * cos(2 * phik);
	double uk = phik + delta_uk;/*������Ǿ�*/
	double rk = A * (1 - Eph->e * cos(Ek)) + delta_rk;/*������*/
	double ik = Eph->i0 + delta_ik + (Eph->iDot) * tk;/*�����Ĺ�����*/
	double Orbit_xk = rk * cos(uk);/*�����ڹ��ƽ���ϵ����*/
	double Orbit_yk = rk * sin(uk);
	/*��Ҫ�ж���GEO(0)����MEO��IGSO����(1)*/
	int SatKind = 0; 
	/*˫���ж�*/
	if ((Prn > 0 && Prn < 6) || (Prn < 64 && Prn>58)) SatKind = 0;
	else SatKind = 1;

	double OMEGAk,OMEGAk_dot;
	/*���������ٶ��������*/
	double e = Eph->e;
	double Ek_dot = n / (1 - e * cos(Ek));
	double phi_dot = sqrt((1 + e) / (1 - e)) * pow((cos(vk / 2) / cos(Ek / 2)), 2) * Ek_dot;
	double uk_dot = 2 * (Eph->Cus * cos(2 * phik) - Eph->Cuc * sin(2 * phik)) * phi_dot + phi_dot;
	double rk_dot = A * e * sin(Ek) * Ek_dot + 2 * (Eph->Crs * cos(2 * phik) - Eph->Crc * sin(2 * phik)) * phi_dot;
	double Ik_dot = Eph->iDot + 2 * (Eph->Cis * cos(2 * phik) - Eph->Cic * sin(2 * phik)) * phi_dot;
	switch (SatKind)
	{
	case 1: 
	{
		/*λ�ò���*/
		OMEGAk = Eph->OMEGA0 + (Eph->OMEGADot - BDS_OMEGAEARTH) * tk - BDS_OMEGAEARTH * toe.SecOfWeek;
		Mid->SatPos[0] = Orbit_xk * cos(OMEGAk) - Orbit_yk * cos(ik) * sin(OMEGAk);
		Mid->SatPos[1] = Orbit_xk * sin(OMEGAk) + Orbit_yk * cos(ik) * cos(OMEGAk);
		Mid->SatPos[2] = Orbit_yk * sin(ik);
		/*�ٶȲ���*/
		OMEGAk_dot = Eph->OMEGADot - BDS_OMEGAEARTH;
		XMatrix RDot(3, 4), Vec(4, 1), Vel(3, 1);
		inputRDot(RDot, OMEGAk, ik, Orbit_xk, Orbit_yk);
		Vec(0, 0) = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		Vec(1, 0) = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
		Vec(2, 0) = OMEGAk_dot;
		Vec(3, 0) = Ik_dot;
		Vel = RDot * Vec;
		Mid->SatVel[0] = Vel(0, 0);
		Mid->SatVel[1] = Vel(1, 0);
		Mid->SatVel[2] = Vel(2, 0);
		
		break; 
	}
	case 0:
	{
		/*λ�ò���*/
		OMEGAk = Eph->OMEGA0 + Eph->OMEGADot * tk - BDS_OMEGAEARTH * toe.SecOfWeek;
		XMatrix PosGK(3, 1), Rx(3, 3), Rz(3, 3), ResPos(3, 1);
		PosGK(0, 0) = Orbit_xk * cos(OMEGAk) - Orbit_yk * cos(ik) * sin(OMEGAk);
		PosGK(1, 0) = Orbit_xk * sin(OMEGAk) + Orbit_yk * cos(ik) * cos(OMEGAk);
		PosGK(2, 0) = Orbit_yk * sin(ik);
		inputRx(Rx,  -5.0 / 180.0 * PI );
		inputRz(Rz,BDS_OMEGAEARTH * tk);
		XMatrix temp = Rz * Rx;
		ResPos = temp * PosGK;
		Mid->SatPos[0] = ResPos(0, 0);
		Mid->SatPos[1] = ResPos(1, 0);
		Mid->SatPos[2] = ResPos(2, 0);
		/*�ٶȲ���*/
		OMEGAk_dot = Eph->OMEGADot;
		XMatrix temp2, temp3,Vec1(4,1),ResVel(3,1),RzDot(3,3),RDot(3,4);
		Vec1(0, 0) = rk_dot * cos(uk) - rk * uk_dot * sin(uk);
		Vec1(1, 0) = rk_dot * sin(uk) + rk * uk_dot * cos(uk);
		Vec1(2, 0) = OMEGAk_dot;
		Vec1(3, 0) = Ik_dot;
		inputRzDot(RzDot, BDS_OMEGAEARTH,tk);
		inputRDot(RDot, OMEGAk, ik, Orbit_xk, Orbit_yk);
		temp2 = temp * RDot;
		temp2 = temp2 * Vec1;
		temp3 = RzDot * Rx;
		temp3 = temp3 * PosGK;
		ResVel = temp2 + temp3;
		Mid->SatVel[0] = ResVel(0, 0);
		Mid->SatVel[1] = ResVel(1, 0);
		Mid->SatVel[2] = ResVel(2, 0);
		break;
	}
	default:break;
	}
	Mid->Ek = Ek;
	Mid->Ek_dot = Ek_dot;
	Mid->Tgd1 = Eph->TGD1;/*�����򱱶�ֻ��Ҫ�õ�TGD1������������*/
	return 1;
}
double Hopfield( double H, double Elev)
/**************
		������ģ��,
�������������:�����������վ�߶ȡ��߶Ƚ�(deg)
����ֵ:���������ֵ,���������㷶Χ����0
*****************/
{	

	if (H > 1e4||H<-422) return 0;/*H���������㷵����*/
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double hw = 11000.0;
	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double p = P0 * pow((1 - 0.0000226 * (H - H0)), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double Kw = (155.2e-7) * 4810 / (T * T) * e * (hw - H);
	double Kd = (155.2e-7) * p / T * (hd - H);
	double Delta_d = Kd / sin(sqrt(Elev * Elev + 6.25) * PI / 180.0);
	double Delta_w=Kw/ sin(sqrt(Elev * Elev + 2.25) * PI / 180.0);
	double DeltaTrop = Delta_d + Delta_w;
	return DeltaTrop;
}
void DetectOutlier(EPOCHOBSDATA* Obs)
{
	int i, j, Thenum = 0;
	bool FindFlag;
	double dGF, dMW;
	MWGF Com_Cur[MAXCHANNUM];
	for (i = 0; i < Obs->SatNum; i++)
	{
		if (Obs->Satobs[i].System == UNKS) continue;/*�����ǲ�����GPS����BDS��ֱ������*/
		DTCycleSlipIni(&Obs->Satobs[i]);/*�����ڲ�������׼�ȳ�����������*/

		/*���ж�˫ƴα�����λ�����Ƿ�����*/
		if (Obs->Satobs[i].fFlag[0]==0 || Obs->Satobs[i].fFlag[1] == 0)
		{
			Obs->Satobs[i].Valid = false;  //����λ����Ч���������޹أ����Ժ����������ж�����λ����Ч��
			Obs->SatPvT[i].Valid = false;
			//memset(Obs->ComObs + i, 0, sizeof(MWGF));    �����ܽ��������գ����ｫobsδ�۲⵽��ֵ��Ϊ��Ч����
			continue;
		}
		else if(Obs->Satobs[i].fFlag[0] == -1 || Obs->Satobs[i].fFlag[1] == -1)
		{
			continue;
		}
		Obs->Satobs[i].Valid = true;
		Obs->SatPvT[i].Valid = true;//����obs�۲����ݣ������һ�����Լ�������λ�á�
		Com_Cur[i].Prn = Obs->Satobs[i].Prn;
		Com_Cur[i].Sys = Obs->Satobs[i].System;
		Com_Cur[i].n = 1;
		/*����ϵͳ�����жϼ���MW��GFֵ*/
		CalMWGFPIF<MWGF,SATOBSDATA>(Com_Cur[i], Obs->Satobs[i]);
		FindFlag = false;
		for (j = 0; j < MAXCHANNUM; j++)
		{
			if(Com_Cur[i].Prn==Obs->ComObs[j].Prn && Com_Cur[i].Sys == Obs->ComObs[j].Sys &&
				fabs(Obs->ComObs[j].GF) > 1e-8)
			{
				FindFlag=true;
					break;
			}
		}
		if(FindFlag)
		{
			dGF = Com_Cur[i].GF - Obs->ComObs[j].GF;
			dMW = Com_Cur[i].MW - Obs->ComObs[j].MW;/*�ȼ����ֵdMW�ټ���MW�ĵ�ǰ��Ԫƽ��ֵ������*/
			if (fabs(dGF) < 5e-2 && fabs(dMW) < 3)
			{
				//Obs->SatPvT[i].Valid = true;
				Obs->Satobs[i].fFlag[0]= Obs->Satobs[1].fFlag[1] = 1;
				Obs->ComObs[i].Prn = Obs->ComObs[j].Prn;
				Obs->ComObs[i].Sys = Obs->ComObs[j].Sys;
				Obs->ComObs[i].MW = (Obs->ComObs[j].n * Obs->ComObs[j].MW + Com_Cur[i].MW) / (Obs->ComObs[j].n + 1);
				Obs->ComObs[i].n = Obs->ComObs[j].n + 1;
				Obs->ComObs[i].PIF = Com_Cur[i].PIF;//Ϊ����obs��PRN˳����룬���ﰴ��obs��˳������
			}
			else/*����ʱ������������MW���ֵӦ�ÿ�ʼ�µ�ƽ��*/
			{
				//Obs->SatPvT[i].Valid = true;
				Obs->Satobs[i].fFlag[0] = Obs->Satobs[i].fFlag[1] = -1;
				Obs->ComObs[i].Prn = Com_Cur[i].Prn;
				Obs->ComObs[i].Sys = Com_Cur[i].Sys;
				Obs->ComObs[i].MW = Com_Cur[i].MW;
				Obs->ComObs[i].n =  1;
				Obs->ComObs[i].PIF = Com_Cur[i].PIF;  //PIF�������Ƿ����޹�
			}
			if (i != j)
			{
				memset(Obs->ComObs + j, 0, sizeof(MWGF));//�ڴ������i�����еĹ۲�ֵ�󣬽�ԭ��j��MWGF������
			}
		}
		else 
		{
			memcpy(Obs->ComObs + i, Com_Cur + i, sizeof(MWGF));
			//Obs->SatPvT[i].Valid = true;
			Obs->ComObs[i].n = 1;
			Obs->ComObs[i].PIF = Com_Cur[i].PIF;
		}
		//memset(Obs->Satobs+i, 0, sizeof(SATOBSDATA));
	}
}
void ComputeSatPVTAtSignalTrans( EPOCHOBSDATA* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[], bool flag)//flag���Ѿ���ʼ�����ջ�վ������ɵı�־���Ϳ��԰����Ǹ߶Ƚ�С��15��ȥ��
{
	extern ROVERCFGINFO CFGINFO;
	GPSTIME t_tr, t_r;/*���ǽ����źű���ʱ�����*/
	GPSEPHREC NeedEph;
	bool EphTimeflag,EphExistFlag;//�ж������Ƿ����
	double P;
	int index = 0;/*�����ǵĵ�һ��Ƶ�μ��㷢��ʱ��*/
	SATMIDRES deltaT_j;/*������Ƕ�λ������Ӳ�*/
	t_r = Epk->Time;
	deltaT_j.SatClkOft = 0;/*��ʼ�����Ӳ���Ϊ��*/
	for (int i = 0; i < Epk->SatNum; i++)
	{
		EphExistFlag=AlignEphObs(Epk->Satobs + i, GPSEph, BDSEph, &NeedEph);
		if (!EphExistFlag)//���������������ڣ�������һ��ѭ��
		{
			Epk->SatPvT[i].Valid = false;
			continue;
		}
		EphTimeflag = JudgeEphEffect(Epk->Satobs[i].System, Epk->Time, NeedEph);
		if (!EphTimeflag)//�������������ڣ��������һ��ѭ��
		{
			Epk->SatPvT[i].Valid = false;
			continue;
		}
		if (Epk->SatPvT[i].Valid == false)
		{
			/*cout << "������û�й۲�����" << endl;*/
			continue;
		}
		if (flag == true && Epk->SatPvT[i].Elevation < CFGINFO.ElevThreshold)
		{
			Epk->SatPvT[i].Valid = false;
			continue;
		}
		for (int j = 0; j < 3; j++)//��������,�������ȷ���Ƿ���ʱ��
		{
			P = Epk->Satobs[i].P[index];
			t_tr.SecOfWeek = t_r.SecOfWeek - P / CLight - deltaT_j.SatClkOft;
			t_tr.Week = t_r.Week;
			if (t_tr.SecOfWeek < 0)
			{
				t_tr.SecOfWeek = 604800 + t_tr.SecOfWeek;
				t_tr.Week = t_tr.Week - 1;
			}
			CompSatClkOff( &t_tr,&NeedEph, &deltaT_j,false);/*��ʱ�����������ЧӦ*/
		}/*���ˣ������źŷ���ʱ�̼������*/
		if (Epk->Satobs[i].System == BDS)
		{
			CompBDSSatPVT(Epk->Satobs[i].Prn, &t_tr, &NeedEph, &deltaT_j);/*��������λ���ٶ�*/
		}
		else if (Epk->Satobs[i].System == GPS)
		{
			CompGPSSatPVT(Epk->Satobs[i].Prn, &t_tr, &NeedEph, &deltaT_j);/*��������λ���ٶ�*/
		}
		CompSatClkOff(&t_tr, &NeedEph, &deltaT_j, true);/*���������ЧӦ*/
		EarthRotateCorrect(Epk->Satobs[i].System, &deltaT_j, UserPos);/*������ת����*/
		memcpy(Epk->SatPvT+i, &deltaT_j, sizeof(SATMIDRES));/*�����*/
		Epk->SatPvT[i].Valid = true;
	}
}
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)
{
	extern ROVERCFGINFO CFGINFO;
	LSQ ls;
	int s, t;//�ܹ۲�������Ҫ�۲���
	//ofstream outfile;
	bool flag =false; //λ�ó�ʼ����־
	//outfile.open("result.txt", ios::app);
	double X0_R[3],deltaT_r1,deltaT_r2,i;/*��λ��ֵ�ͽ��ջ��Ӳ��ֵ,deltaT_r1,deltaT_r2�ֱ�ΪGPS��BDS���Ӳ�*/
	if (Epoch->SatNum < 4)
	{
		cout << "����ʧ�ܣ�������������" << endl;
		return false;
	}

	if (Res->Valid == false)
	{
		X0_R[0] = X0_R[1] = X0_R[2] = InitialValue;
		
	}
	else {
		memcpy(X0_R, Res->Pos, 3 * sizeof(double)); 
	}

	if (abs(X0_R[0] > 1e13)|| abs(X0_R[1] > 1e13)|| abs(X0_R[2] > 1e13)||isnan(X0_R[0]))
	{
		X0_R[0] = X0_R[1] = X0_R[2] = InitialValue;
	}

	deltaT_r1 = deltaT_r2 = 0;
	i = 0;

	do
	{
		memset(&ls, 0, sizeof(LSQ));
		ComputeSatPVTAtSignalTrans(Epoch, GPSEph, BDSEph, X0_R, flag);
		SPPintputB(ls.B, Epoch, X0_R);
		//B.MatrixDis();
		int Prow = ls.B.row;/*P������Ϊ��λ��*/
		if(Prow<4)  
		{
			cout << "�����������㣬SPP����ʧ��" << endl;
			return false;
		}
		ls.P.MatrixResize(Prow, Prow);//�Ȱ�P��������������
		InputP(Prow, Epoch, false, ls.P);
		//P.MatrixDis();
		SPPinputW(ls.W, Epoch, X0_R);
		//W.MatrixDis();
		Res->SatNum = Prow;//ֱ��������Ͱ���Ч����д��
		LSQCalx(ls.B, ls.P, ls.W, ls.x, ls.Q);
		X0_R[0] = X0_R[0] + ls.x(0, 0); 
		X0_R[1] = X0_R[1] + ls.x(1, 0);
		X0_R[2] = X0_R[2] + ls.x(2, 0);
		Res->Valid = true;
		if (i >= 5 && fabs(ls.x(0,0)+ ls.x(1, 0)+ ls.x(2, 0))>1e-2)
		{
			cout << "��С����δ��������λ����ʧ��" << endl;
			Res->Valid = false;
			return false;
		}
		if (i > 0)
		{
			flag = CFGINFO.EmaFlag ;//�ж��Ƿ��Ѿ�������ʼ����
		}
		i++;
	} while (fabs(ls.x(0, 0) + ls.x(1, 0) + ls.x(2, 0) ) > 1e-5);
	deltaT_r1 = deltaT_r1 + ls.x(3, 0);
	t = 4;
	if (ls.x.row > 4)//����BDS���Ӳ�ʱ
	{
		deltaT_r2 = deltaT_r2 + ls.x(4, 0);
		t = 5;//����BDSʱ����Ҫ�۲�����Ϊ5������Ϊ4
	}
	LSQCalPrCis(ls, ls.theta, ls.PDOP, 0);
	memcpy(Res->Pos, X0_R, 3 * sizeof(double));
	Res->ReceiTGPS = deltaT_r1;
	Res->ReceiTBDS = deltaT_r2;
	Res->PDOP = ls.PDOP;
	Res->SigmaPos = ls.theta;
	return true;
}
bool SPV(EPOCHOBSDATA* Epoch, POSRES* Res)
{
	/*���Է����飬�������*/
	LSQ ls;
	int s, t;
	SPVInputB(ls.B, Epoch, Res);
	SPVInputW(ls.W,Epoch, Res);
	int Prow = ls.B.row;/*P������Ϊ��λ��*/
	if (Prow < 4)
	{
		cout << "�����������㣬SPV����ʧ��" << endl;
		return false;
	}
	ls.P.MatrixResize(Prow, Prow);//�Ȱ�P��������������
	for (int j = 0; j < Prow; j++)
	{
		ls.P(j, j) = 1;
	}
	LSQCalx(ls.B, ls.P, ls.W, ls.x, ls.Q);
	LSQCalPrCis(ls, ls.theta, ls.PDOP, 0);
	Res->Vel[0] = ls.x(0, 0);
	Res->Vel[1] = ls.x(1, 0);
	Res->Vel[2] = ls.x(2, 0);
	Res->ReceiT_dot = ls.x(3, 0) / CLight;
	Res->SigmaVel = ls.theta;
	return true;
}
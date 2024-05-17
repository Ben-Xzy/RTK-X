#include"output.h"
using namespace std;

void OutPutResult(EPOCHOBSDATA* EachObs, POSRES Res, ofstream& outfile, int flag)
{
	ENU enu(Res.Pos, Res.RealPos);
	CompEnudPos(&enu);
	double BLH[3];
	Xyz2Blh(Res.Pos, BLH);
	BLH[0] = BLH[0] * deg;
	BLH[1] = BLH[1] * deg;
	if (flag == 0)
	{
		cout << "ภ๚ิช:";
		cout << EachObs->Time.Week << " " << EachObs->Time.SecOfWeek << " ";
		cout.flags(ios::fixed);
		cout.precision(4);
		cout << "ECEF-X:" << Res.Pos[0] << " ECEF-Y:" << Res.Pos[1] << " ECEF-Z:" << Res.Pos[2] << " ";
		//cout<< "RefECEF-X:" << Res.RealPos[0] << " RefECEF-Y:" << Res.RealPos[1] << "  RefECEF-Z:" << Res.RealPos[2] << " ";
		cout.precision(8);
		cout << "  B:" << BLH[0] << "  L:" << BLH[1] << "  H:" << BLH[2] << " ";
		cout.precision(4);
		cout << " VX:" << std::left << std::setw(7) << Res.Vel[0] << " VY:" << std::left << std::setw(7) << Res.Vel[1] << " VZ:" << std::left << std::setw(7) << Res.Vel[2] << " ";
		cout << " PDOP:" << std::left << std::setw(7) << Res.PDOP << " SigmaP:" << std::left << std::setw(7) << Res.SigmaPos << " SigmaV:" << std::left << std::setw(7) << Res.SigmaVel;
		if (Res.realPosFlag)
		{
			cout << " DE:" << std::left << std::setw(7) << enu.dEnu[0] << " DN:" << enu.dEnu[1] << " DU:" << enu.dEnu[2] << " ";
		}
		cout << endl;
		cout.precision(0);
		//Sleep(80);
	}
	else if (flag == 1)
	{
		outfile << EachObs->Time.Week << " " << EachObs->Time.SecOfWeek << " ";
		//outfile << Res.Pos[0] << "   " << Res.Pos[1] << "   " << Res.Pos[2] << "   ";
		//outfile << Res.Vel[0] << "   " << Res.Vel[1] << "   " << Res.Vel[2] << "   ";
		if (Res.realPosFlag)
		{
			cout << enu.dEnu[0] << "   " << enu.dEnu[1] << "   " << enu.dEnu[2] << "   ";
			outfile << enu.dEnu[0] << " " << enu.dEnu[1] << " " << enu.dEnu[2] << " ";
		}
		outfile << Res.PDOP << " " << Res.SigmaPos << " " << Res.SigmaVel << " " << Res.SatNum << " ";
		for (int i = 0; i < EachObs->SatNum; i++)
		{
			if (EachObs->SatPvT[i].Valid)
			{
				string Prn = EachObs->Satobs[i].System == GPS ? " G" : " C";
				outfile << Prn << EachObs->Satobs[i].Prn << ": " << " " << EachObs->SatPvT[i].Azimuth << " " << EachObs->SatPvT[i].Elevation << " ";
			}
		}
		outfile << endl;
		cout << Res.PDOP << "   " << Res.SigmaPos << "   " << Res.SigmaVel << endl;
	}
	else { cout << "instructions error!" << endl; }
}
void OutputRTK(GPSTIME T,DDCEPOCHOBS D,ofstream& outfile, string s ,POSRES& r)
{
	 int nG, nB = 0;
	 nG = (D.DDSatNum[0] == 0) ? 0 : (D.DDSatNum[0] + 1);
	 nB = (D.DDSatNum[1] == 0) ? 0 : (D.DDSatNum[1] + 1);
	 cout.flags(ios::fixed);
	 outfile.flags(ios::fixed);
	 outfile.precision(4);
	 outfile << T.Week << " " << T.SecOfWeek << " ";
	 cout.precision(1);
	 cout << T.Week << " " << T.SecOfWeek << " ";
	 cout.precision(5);
	 if (s == "enu")
	 {
		 cout << " dE:" << setw(9) << D.denu[0] << "   dN:"  << D.denu[1] << "   dU:" << D.denu[2] << "   Ratio:" << D.Ratio << "   nG:";
		 cout << setw(2) << setfill('0') << nG << "   nB:";
		 cout << setw(2) << setfill('0') << nB;
		 cout << "  PDop:" << r.PDOP;
		 outfile.precision(8);
		 outfile << D.denu[0] << " " << D.denu[1] << " " << D.denu[2] << " " << D.Ratio;
		 outfile << endl;
	 }
	 else if (s == "xyz")
	 {
		 cout << " dX:" << D.dPos[0] << "   dY:" << D.dPos[1] << "   dZ:" << D.dPos[2] << "   Ratio:" << D.Ratio << "   nG:";
		 cout << setw(2) << setfill('0') << nG << "   nB:";
		 cout << setw(2) << setfill('0') << nB;
		 cout << "  PDop:" << r.PDOP;
		 outfile.precision(8);
		 outfile << D.dPos[0] << " " << D.dPos[1] << " " << D.dPos[2] << " " << D.Ratio;
		 outfile << endl;
	 }
	 string ss[2] = { "G","C" };
	 cout << "      ref Sat PRN: ";
	 for (int i = 0; i < 2; i++)
	 {
		 if (D.RefPrn[i] != -1)
		 {
			 cout  <<ss[i]<< D.RefPrn[i] << " ";
		 }
	 }
	 cout << endl;
}
void LibOutput(GPSTIME t, DDCEPOCHOBS &D, ofstream& outfile, RTKEKF &e)
{
	JDTIME t_j;
	GPST2JD(&t, &t_j);
	COMMONTIME t_c;
	JD2Com(&t_j, &t_c);
	outfile.flags(ios::fixed);
	outfile.precision(3);
	//cout << t_c.Year << "/" << t_c.Month << "/" << t_c.Day << " " << t_c.Hour << ":" << t_c.Minute << ":" << t_c.Second << " ";
	outfile << t_c.Year << "/";
	outfile << setw(2) << setfill('0') << t_c.Month << "/";
	outfile << setw(2) << setfill('0') << t_c.Day << " ";
	outfile << setw(2) << setfill('0') << t_c.Hour << ":";
	outfile << setw(2) << setfill('0') << t_c.Minute << ":";
	outfile << fixed <<setw(6) << setfill('0') << t_c.Second << "  ";

	outfile.precision(4);
	outfile << e.X[0] << "  " << e.X[1] << "  " << e.X[2] << "   ";
	if (D.Ratio > 3.0)
	{
		outfile << "1" << "  ";
	}
	else { outfile << "2" << "   "; }
	outfile << 2 * e.nSats << "   ";
	outfile << e.P(0, 0) << "   " << e.P(1, 1) << "   " << e.P(2, 2) << "   " << e.P(0, 1) << "   " << e.P(1, 2) << "   " << e.P(0, 2) << "   ";
	outfile << "0.00" << "    " << D.Ratio << endl;

}
void LibOutput(GPSTIME t, DDCEPOCHOBS& D, ofstream& outfile, POSRES& r)
{
	JDTIME t_j;
	GPST2JD(&t, &t_j);
	COMMONTIME t_c;
	JD2Com(&t_j, &t_c);
	outfile.flags(ios::fixed);
	outfile.precision(3);
	//cout << t_c.Year << "/" << t_c.Month << "/" << t_c.Day << " " << t_c.Hour << ":" << t_c.Minute << ":" << t_c.Second << " ";
	outfile << t_c.Year << "/";
	outfile << setw(2) << setfill('0') << t_c.Month << "/";
	outfile << setw(2) << setfill('0') << t_c.Day << " ";
	outfile << setw(2) << setfill('0') << t_c.Hour << ":";
	outfile << setw(2) << setfill('0') << t_c.Minute << ":";
	outfile << fixed << setw(6) << setfill('0') << t_c.Second << "  ";

	outfile.precision(4);
	outfile << r.Pos[0] << "  " << r.Pos[1] << "  " << r.Pos[2] << "   ";
	if (D.Ratio > 3.0)
	{
		outfile << "1" << "  ";
	}
	else { outfile << "2" << "   "; }
	outfile << 2 * D.Sats << "   ";
	outfile << "0.00" << "   " << "0.00" << "   " << "0.00" << "   " << "0.00" << "   " << "0.00" << "   " << "0.00" << "   ";
	outfile << "0.00" << "    " << D.Ratio << endl;
}